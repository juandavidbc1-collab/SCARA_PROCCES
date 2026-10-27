/*
 * LABORATORIO 4 EN EL ROBOT: observadores de Luenberger y Kalman con el MOTOR 1 (Pololu 37D 100:1).
 *
 *   - Lazo de 1 kHz: aplica el PWM pedido (con rampa) y vigila topes y motor trabado.
 *   - Cada 10 ms (Ts del laboratorio): velocidad medida por el encoder, Luenberger y Kalman.
 *   - Por el puerto serie (115200 baudios) manda una línea cada 20 ms (una de cada 2 muestras); lab4_panel.py la grafica.
 *     Los observadores corren igual cada 10 ms; solo se envía la mitad para no saturar el puerto.
 *
 * Órdenes por el puerto serie (las manda lab4_panel.py):
 *   p <porcentaje>   PWM de -100 a 100 (0 = parar)
 *   r                reiniciar observadores (x = 0, P = P0)
 * Si no llega ninguna orden en 1,5 s, el motor se detiene (por si se cierra el programa del PC).
 *
 * Modelo (el del laboratorio, referido al eje de salida):  x = [i (A); w (rad/s)],  y = w
 *   La di/dt = V - Ra i - Kv w      J dw/dt = Kv i - B w
 *   Ra = 50, La = 0.060, Kv = 0.167092, B = 2.79197e-3, J = 3.63232e-4
 * Discretizado con ZOH, Ts = 10 ms (A, B y L calculados con expm y Ackermann; mismos del laboratorio).
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BAUD         115200
#define ENVIAR_CADA  2                /* muestras: 2 -> una línea cada 20 ms */

/* ------------------------------------------------------------------ pines de la tarjeta (config.h del robot) */
#define PIN_STBY     GPIO_NUM_17
#define PIN_M1_PWM   GPIO_NUM_2
#define PIN_M1_IN1   GPIO_NUM_16
#define PIN_M1_IN2   GPIO_NUM_4
#define PIN_ENC1_A   GPIO_NUM_33
#define PIN_ENC1_B   GPIO_NUM_25
#define PIN_FC_POS   GPIO_NUM_36      /* final de carrera que se toca girando en sentido + (topes cruzados) */
#define PIN_FC_NEG   GPIO_NUM_39      /* final de carrera que se toca girando en sentido - */
#define FC_PRESIONADO 0
/* motores 2 y 3: se dejan apagados (salidas en 0) */
static const gpio_num_t OTROS[] = { GPIO_NUM_19, GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_21, GPIO_NUM_23, GPIO_NUM_22 };

#define MOTOR_SIGN   (-1)             /* signos del robot: con PWM > 0 la velocidad medida es > 0 */
#define ENC_SIGN     (-1)
#define CUENTAS      6424.0f          /* cuentas del encoder por vuelta del eje de salida */

/* ------------------------------------------------------------------ control */
#define PWM_MAX      1023
#define RAMPA        3                /* PWM por ms: de 0 a 100 % en 0,34 s */
#define VBUS         12.0f
#define V0           0.8f             /* zona muerta del TB6612 + fricción seca [V] (L298N del lab: 1,197) */
#define VGAIN        1.0f             /* ganancia del driver (L298N del lab: 0,9782) */
#define TS           0.01f
#define RPM          (3.14159265f / 30.0f)
#define TIMEOUT_MS   1500
#define TRABADO_PWM  350              /* con más de ~34 % de PWM ... */
#define TRABADO_MS   300              /* ... y menos de 3 cuentas en este tiempo -> motor trabado */

/* ------------------------------------------------------------------ observadores */
static const float A[2][2] = { { -0.00147851f, -0.00308602f }, { 0.50976057f, 0.91345392f } };
static const float B[2] = { 0.01976948f, 0.07782986f };
static const float L[2] = { 0.24358162f, 0.41197541f };      /* polos de A - LC: 0,25 +/- 0,25j */
#define SIG_I   0.010f                                       /* Q: 10 mA */
#define SIG_W   (0.30f * RPM)                                /* Q: 0,30 rpm */
#define SIG_Y   (0.81f * RPM)                                /* R: 0,81 rpm (R = 0,655 rpm^2 medido en el lab) */
#define P0_I    0.10f
#define P0_W    (1.0f * RPM)

typedef struct {
    float xl[2], y_prev;                     /* Luenberger */
    float xp[2], Pm[2][2];                   /* Kalman a priori */
    float xk[2], Pk[2][2], K[2];             /* Kalman a posteriori */
    int primero;
} obs_t;

static obs_t O;
static volatile int pedir_reset = 0;

static void obs_reset(obs_t *o)
{
    memset(o, 0, sizeof *o);
    o->Pm[0][0] = o->Pk[0][0] = P0_I * P0_I;
    o->Pm[1][1] = o->Pk[1][1] = P0_W * P0_W;
    o->primero = 1;
}

/* u_prev: voltaje medio del intervalo anterior [V];  y: velocidad medida ahora [rad/s] */
static void obs_paso(obs_t *o, float u_prev, float y)
{
    if (o->primero) {                        /* primera muestra: igual que en MATLAB (x = 0, P = P0, sin corregir) */
        o->y_prev = y;
        o->xp[0] = o->xk[0]; o->xp[1] = o->xk[1];
        o->primero = 0;
        return;
    }
    /* Luenberger (predictor): x(k) = A x(k-1) + B u(k-1) + L [y(k-1) - C x(k-1)] */
    float x0 = o->xl[0], x1 = o->xl[1], inn = o->y_prev - x1;
    o->xl[0] = A[0][0] * x0 + A[0][1] * x1 + B[0] * u_prev + L[0] * inn;
    o->xl[1] = A[1][0] * x0 + A[1][1] * x1 + B[1] * u_prev + L[1] * inn;
    o->y_prev = y;

    /* Kalman, predicción: x- = A x+ + B u ;  P- = A P+ A' + Q */
    x0 = o->xk[0]; x1 = o->xk[1];
    o->xp[0] = A[0][0] * x0 + A[0][1] * x1 + B[0] * u_prev;
    o->xp[1] = A[1][0] * x0 + A[1][1] * x1 + B[1] * u_prev;
    float p00 = o->Pk[0][0], p01 = o->Pk[0][1], p11 = o->Pk[1][1];
    float m00 = A[0][0] * p00 + A[0][1] * p01, m01 = A[0][0] * p01 + A[0][1] * p11;
    float m10 = A[1][0] * p00 + A[1][1] * p01, m11 = A[1][0] * p01 + A[1][1] * p11;
    o->Pm[0][0] = m00 * A[0][0] + m01 * A[0][1] + SIG_I * SIG_I;
    o->Pm[0][1] = o->Pm[1][0] = m00 * A[1][0] + m01 * A[1][1];
    o->Pm[1][1] = m10 * A[1][0] + m11 * A[1][1] + SIG_W * SIG_W;

    /* Kalman, corrección con y(k):  K = P- C' / (C P- C' + R),  C = [0 1] */
    float S = o->Pm[1][1] + SIG_Y * SIG_Y;
    o->K[0] = o->Pm[0][1] / S;
    o->K[1] = o->Pm[1][1] / S;
    float e = y - o->xp[1];
    o->xk[0] = o->xp[0] + o->K[0] * e;
    o->xk[1] = o->xp[1] + o->K[1] * e;
    o->Pk[0][0] = o->Pm[0][0] - o->K[0] * o->Pm[1][0];
    o->Pk[0][1] = o->Pk[1][0] = o->Pm[0][1] - o->K[0] * o->Pm[1][1];
    o->Pk[1][1] = o->Pm[1][1] - o->K[1] * o->Pm[1][1];
}

static float voltaje(float duty_frac)
{
    float v = VGAIN * VBUS * duty_frac;
    float a = fabsf(v) - V0;
    return a <= 0.0f ? 0.0f : copysignf(a, v);
}

/* ------------------------------------------------------------------ hardware */
static pcnt_unit_handle_t enc;
static int dir_actual = 0;

static void pwm_set(int d)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, d);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void motor(int duty)
{
    duty *= MOTOR_SIGN;
    int d = abs(duty);
    if (d > PWM_MAX) d = PWM_MAX;
    if (d == 0) { pwm_set(0); gpio_set_level(PIN_M1_IN1, 0); gpio_set_level(PIN_M1_IN2, 0); dir_actual = 0; return; }
    int dir = duty > 0 ? 1 : -1;
    if (dir != dir_actual) {                 /* al invertir: PWM 0, luego IN1/IN2 */
        pwm_set(0);
        gpio_set_level(PIN_M1_IN1, dir > 0);
        gpio_set_level(PIN_M1_IN2, dir < 0);
        dir_actual = dir;
    }
    pwm_set(d);
}

/* freno: los dos lados del motor a la misma tensión (TB6612 en "short brake") */
static void motor_freno(void)
{
    gpio_set_level(PIN_M1_IN1, 1);
    gpio_set_level(PIN_M1_IN2, 1);
    pwm_set(PWM_MAX);
    dir_actual = 0;
}

/* texto al PC sin bloquear el lazo: va al búfer del driver de la UART */
static void enviar(const char *fmt, ...)
{
    char b[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    if (n > 0) uart_write_bytes(UART_NUM_0, b, n < (int)sizeof b ? n : (int)sizeof b - 1);
}

static int32_t cuentas(void)
{
    int v = 0;
    pcnt_unit_get_count(enc, &v);
    return ENC_SIGN * v;
}

static void hw_init(void)
{
    gpio_config_t st = { .pin_bit_mask = 1ULL << PIN_STBY, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&st);
    gpio_set_level(PIN_STBY, 0);

    uint64_t mask = (1ULL << PIN_M1_IN1) | (1ULL << PIN_M1_IN2);
    for (size_t k = 0; k < sizeof OTROS / sizeof OTROS[0]; k++) mask |= 1ULL << OTROS[k];
    gpio_config_t io = { .pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    gpio_set_level(PIN_M1_IN1, 0);
    gpio_set_level(PIN_M1_IN2, 0);
    for (size_t k = 0; k < sizeof OTROS / sizeof OTROS[0]; k++) gpio_set_level(OTROS[k], 0);

    ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                              .timer_num = LEDC_TIMER_0, .freq_hz = 20000, .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t c = { .gpio_num = PIN_M1_PWM, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
                                .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&c));

    gpio_config_t fc = { .pin_bit_mask = (1ULL << PIN_FC_POS) | (1ULL << PIN_FC_NEG), .mode = GPIO_MODE_INPUT };
    gpio_config(&fc);                        /* GPIO 34-39: sin pull interno, la tarjeta trae las resistencias */

    pcnt_unit_config_t uc = { .low_limit = -30000, .high_limit = 30000, .flags.accum_count = 1 };
    ESP_ERROR_CHECK(pcnt_new_unit(&uc, &enc));
    pcnt_glitch_filter_config_t f = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(enc, &f));
    pcnt_channel_handle_t ca = NULL, cb = NULL;
    pcnt_chan_config_t cfa = { .edge_gpio_num = PIN_ENC1_A, .level_gpio_num = PIN_ENC1_B };
    pcnt_chan_config_t cfb = { .edge_gpio_num = PIN_ENC1_B, .level_gpio_num = PIN_ENC1_A };
    ESP_ERROR_CHECK(pcnt_new_channel(enc, &cfa, &ca));
    ESP_ERROR_CHECK(pcnt_new_channel(enc, &cfb, &cb));
    pcnt_channel_set_edge_action(ca, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ca, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(cb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(cb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_add_watch_point(enc, uc.high_limit);
    pcnt_unit_add_watch_point(enc, uc.low_limit);
    ESP_ERROR_CHECK(pcnt_unit_enable(enc));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc));
    ESP_ERROR_CHECK(pcnt_unit_start(enc));

    gpio_set_level(PIN_STBY, 1);             /* driver habilitado, motor en 0 */
}

/* ------------------------------------------------------------------ órdenes del PC */
static volatile int pwm_pedido = 0;          /* -1023 .. 1023 */
static volatile int64_t t_ultima_orden = 0;

static void tarea_serial(void *arg)
{
    (void)arg;
    char linea[48];
    int n = 0;
    uint8_t buf[32];
    for (;;) {
        int r = uart_read_bytes(UART_NUM_0, buf, sizeof buf, pdMS_TO_TICKS(20));
        for (int k = 0; k < r; k++) {
            char ch = (char)buf[k];
            if (ch == '\r') continue;
            if (ch != '\n') { if (n < (int)sizeof linea - 1) linea[n++] = ch; continue; }
            linea[n] = 0;
            n = 0;
            float p;
            if (sscanf(linea, "p %f", &p) == 1) {
                if (p > 100) p = 100;
                if (p < -100) p = -100;
                pwm_pedido = (int)lroundf(p * PWM_MAX / 100.0f);
                t_ultima_orden = esp_timer_get_time();
            } else if (linea[0] == 'r') {
                pedir_reset = 1;
                t_ultima_orden = esp_timer_get_time();
            }
        }
    }
}

/* ------------------------------------------------------------------ lazo de 1 kHz */
/* El ritmo lo da un temporizador de hardware de 1 ms (no depende del tick de FreeRTOS). */
static TaskHandle_t h_control;
static void cada_ms(void *arg) { (void)arg; xTaskNotifyGive(h_control); }

static void tarea_control(void *arg)
{
    (void)arg;
    int duty = 0, objetivo = 0, n = 0, suma = 0, bloqueo = 0, freno_ms = 0;
    int32_t c_prev = cuentas(), c_trab = c_prev;
    int ms_trab = 0;
    float u_prev = 0.0f;
    int64_t t0 = esp_timer_get_time();
    int envio = 0;
    h_control = xTaskGetCurrentTaskHandle();
    obs_reset(&O);
    esp_timer_handle_t tmr;
    esp_timer_create_args_t ta = { .callback = cada_ms, .arg = NULL, .name = "1ms" };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &tmr));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tmr, 1000));

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* orden del PC (0 si dejó de llegar) */
        int64_t ahora = esp_timer_get_time();
        objetivo = pwm_pedido;
        if (objetivo != 0 && ahora - t_ultima_orden > TIMEOUT_MS * 1000LL) {
            pwm_pedido = objetivo = 0;
            enviar("#INFO: no llegan ordenes del PC, motor detenido\n");
        }
        if (bloqueo && (objetivo == 0 || (objetivo > 0) != (bloqueo > 0))) bloqueo = 0;   /* se libera al cambiar de sentido o parar */
        if (bloqueo) objetivo = 0;

        /* protecciones: final de carrera en el sentido del giro y motor trabado */
        if ((objetivo > 0 || duty > 0) && gpio_get_level(PIN_FC_POS) == FC_PRESIONADO) {
            objetivo = duty = 0; bloqueo = +1; freno_ms = 200;
            enviar("#FALLA: final de carrera (sentido +). Frenado; solo se permite girar en sentido -\n");
        }
        if ((objetivo < 0 || duty < 0) && gpio_get_level(PIN_FC_NEG) == FC_PRESIONADO) {
            objetivo = duty = 0; bloqueo = -1; freno_ms = 200;
            enviar("#FALLA: final de carrera (sentido -). Frenado; solo se permite girar en sentido +\n");
        }

        int32_t c = cuentas();
        if (abs(duty) > TRABADO_PWM) {
            if (++ms_trab >= TRABADO_MS) {
                if (abs(c - c_trab) < 3) {
                    bloqueo = duty > 0 ? +1 : -1;
                    objetivo = duty = 0; freno_ms = 200;
                    enviar("#FALLA: motor trabado (PWM alto y el encoder no cuenta). Motor detenido\n");
                }
                ms_trab = 0; c_trab = c;
            }
        } else { ms_trab = 0; c_trab = c; }

        /* rampa (o freno durante 200 ms tras una falla) */
        if (freno_ms > 0) {
            freno_ms--;
            duty = 0;
            if (freno_ms > 0) motor_freno(); else motor(0);
        } else {
            if (objetivo > duty + RAMPA) duty += RAMPA;
            else if (objetivo < duty - RAMPA) duty -= RAMPA;
            else duty = objetivo;
            motor(duty);
        }

        /* observadores cada 10 ms */
        suma += duty;
        if (++n < 10) continue;
        n = 0;
        if (pedir_reset) { obs_reset(&O); pedir_reset = 0; }
        float y = (float)(c - c_prev) / CUENTAS * 2.0f * 3.14159265f / TS;     /* rad/s */
        c_prev = c;
        float u = voltaje(suma / (10.0f * PWM_MAX));                            /* voltaje medio del intervalo */
        suma = 0;
        obs_paso(&O, u_prev, y);
        u_prev = u;
        if (++envio < ENVIAR_CADA) continue;
        envio = 0;

        /* D,t[ms],PWM[%],V,w_med,w_L,i_L,w_Kpri,i_Kpri,w_K,i_K,P11+,P22+,P11-,P22-,K1,K2
         * velocidades en rpm, corrientes en mA, P de corriente en mA^2, P de velocidad en rpm^2, K1 en mA/rpm */
        enviar("D,%lld,%.1f,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.2f,%.1f,%.4g,%.4g,%.4g,%.4g,%.4g,%.4f\n",
               (long long)((ahora - t0) / 1000), duty * 100.0f / PWM_MAX, u,
               y / RPM, O.xl[1] / RPM, O.xl[0] * 1000, O.xp[1] / RPM, O.xp[0] * 1000, O.xk[1] / RPM, O.xk[0] * 1000,
               O.Pk[0][0] * 1e6f, O.Pk[1][1] / (RPM * RPM), O.Pm[0][0] * 1e6f, O.Pm[1][1] / (RPM * RPM),
               O.K[0] * 1000 * RPM, O.K[1]);
    }
}

void app_main(void)
{
    hw_init();
    uart_driver_install(UART_NUM_0, 1024, 4096, 0, NULL, 0);
    uart_set_baudrate(UART_NUM_0, BAUD);
    enviar("\n#LAB4: motor 1, Luenberger + Kalman, Ts = 10 ms. Ordenes: 'p <-100..100>' y 'r'\n");
    xTaskCreatePinnedToCore(tarea_serial, "serial", 3072, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(tarea_control, "control", 4096, NULL, 10, &h_control, 1);
}
