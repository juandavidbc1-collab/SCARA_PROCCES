#include "hal.h"
#include <stdlib.h>
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

#define PWM_BITS LEDC_TIMER_10_BIT

static pcnt_unit_handle_t enc[NJ];
static int dir_actual[NJ + 1];          /* -1, 0, +1 por motor (el índice NJ es el motor 3, sin uso) */
static bool stby_estado = false;

void hal_stby(bool on)
{
    gpio_set_level(PIN_STBY, on ? 1 : 0);
    stby_estado = on;
}

bool hal_stby_get(void) { return stby_estado; }

static void ledc_duty(int ch, int d)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, d);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}

void hal_motor_free(int j)
{
    ledc_duty(j, 0);
    gpio_set_level(JC[j].in1, 0);
    gpio_set_level(JC[j].in2, 0);
    dir_actual[j] = 0;
}

void hal_motor_brake(int j)
{
    gpio_set_level(JC[j].in1, 1);
    gpio_set_level(JC[j].in2, 1);
    ledc_duty(j, PWM_MAX);
    dir_actual[j] = 0;
}

void hal_all_free(void)
{
    for (int j = 0; j < NJ; j++) hal_motor_free(j);
}

/* Al invertir el sentido: primero PWM = 0, luego IN1/IN2, luego el PWM nuevo. */
void hal_motor_drive(int j, int duty)
{
    duty *= JC[j].motor_sign;
    int d = abs(duty);
    if (d > PWM_MAX) d = PWM_MAX;
    if (d == 0) { hal_motor_free(j); return; }
    int dir = duty > 0 ? 1 : -1;
    if (dir != dir_actual[j]) {
        ledc_duty(j, 0);
        gpio_set_level(JC[j].in1, dir > 0 ? 1 : 0);
        gpio_set_level(JC[j].in2, dir > 0 ? 0 : 1);
        dir_actual[j] = dir;
    }
    ledc_duty(j, d);
}

int32_t hal_enc_raw(int j)
{
    int v = 0;
    pcnt_unit_get_count(enc[j], &v);
    return v;
}

int hal_enc_levels(int j)
{
    return gpio_get_level(JC[j].enc_a) | (gpio_get_level(JC[j].enc_b) << 1);
}

bool hal_fc_raw(int j, int side)
{
    return gpio_get_level(side ? JC[j].fc_max : JC[j].fc_min) == FC_PRESIONADO_NIVEL;
}

static pcnt_unit_handle_t init_encoder(gpio_num_t a, gpio_num_t b)
{
    pcnt_unit_handle_t u = NULL;
    pcnt_unit_config_t uc = { .low_limit = -30000, .high_limit = 30000, .flags.accum_count = 1 };
    ESP_ERROR_CHECK(pcnt_new_unit(&uc, &u));
    pcnt_glitch_filter_config_t f = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(u, &f));

    pcnt_channel_handle_t ca = NULL, cb = NULL;
    pcnt_chan_config_t cfa = { .edge_gpio_num = a, .level_gpio_num = b };
    pcnt_chan_config_t cfb = { .edge_gpio_num = b, .level_gpio_num = a };
    ESP_ERROR_CHECK(pcnt_new_channel(u, &cfa, &ca));
    ESP_ERROR_CHECK(pcnt_new_channel(u, &cfb, &cb));
    /* cuadratura x4 (misma configuración que ya se probó en la tarjeta) */
    pcnt_channel_set_edge_action(ca, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ca, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(cb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(cb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_add_watch_point(u, uc.high_limit);
    pcnt_unit_add_watch_point(u, uc.low_limit);
    ESP_ERROR_CHECK(pcnt_unit_enable(u));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(u));
    ESP_ERROR_CHECK(pcnt_unit_start(u));
    return u;
}

void hal_init(void)
{
    /* 1) STBY primero: drivers apagados antes de tocar cualquier otra cosa */
    gpio_config_t st = { .pin_bit_mask = 1ULL << PIN_STBY, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&st);
    hal_stby(false);

    /* 2) pines IN1/IN2 de los 3 canales (el motor 3 queda en 0) */
    uint64_t mask = (1ULL << PIN_M3_IN1) | (1ULL << PIN_M3_IN2);
    for (int j = 0; j < NJ; j++) mask |= (1ULL << JC[j].in1) | (1ULL << JC[j].in2);
    gpio_config_t io = { .pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    gpio_set_level(PIN_M3_IN1, 0);
    gpio_set_level(PIN_M3_IN2, 0);
    for (int j = 0; j < NJ; j++) { gpio_set_level(JC[j].in1, 0); gpio_set_level(JC[j].in2, 0); }

    /* 3) PWM 20 kHz, 10 bits. Canales 0 y 1 = motores 1 y 2; canal 2 = motor 3 (siempre 0) */
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = PWM_BITS, .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    gpio_num_t pwm_pins[3] = { JC[0].pwm, JC[1].pwm, PIN_M3_PWM };
    for (int c = 0; c < 3; c++) {
        ledc_channel_config_t cc = {
            .gpio_num = pwm_pins[c], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)c,
            .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&cc));
    }

    /* 4) finales de carrera: GPIO 34-39 no tienen pull interno; la placa trae R30-R33 */
    uint64_t fm = 0;
    for (int j = 0; j < NJ; j++) fm |= (1ULL << JC[j].fc_min) | (1ULL << JC[j].fc_max);
    gpio_config_t fc = { .pin_bit_mask = fm, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE };
    gpio_config(&fc);

    /* 5) encoders */
    for (int j = 0; j < NJ; j++) enc[j] = init_encoder(JC[j].enc_a, JC[j].enc_b);
}
