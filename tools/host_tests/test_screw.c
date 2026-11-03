/* Pruebas del modelo de M3 y de la máquina de atornillado, sin firmware ni FreeRTOS. */
#include <math.h>
#include <stdio.h>
#include "screw.h"

static int fallos = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("  [FALLO] " __VA_ARGS__); printf("\n"); fallos++; } else { printf("  [ok] " __VA_ARGS__); printf("\n"); } } while (0)

/* Pololu 37D 50:1 a 12 V: 200 rpm y 0,2 A en vacío; 5,5 A y 21 kg·cm calado */
static const screw_model_t M = { .r = 2.1818f, .ke = 0.5522f, .kt = 0.3886f, .i0 = 0.2f, .vbus = 12.0f, .pwm_max = 1023.0f,
                                 .paso_mm = 0.5f, .k_tuerca = 0.2f, .d_mm = 3.0f, .area_mm2 = 15.7f, .torque_rosca = 0.03f };
static const screw_cfg_t C = { .vueltas_meta = 12, .torque_meta = 0.4f, .vueltas_lento = 2, .vueltas_min = 9, .vueltas_max = 14,
                               .duty_rapido = 650, .duty_lento = 300, .torque_asiento = 0.15f, .i_max = 2.4f,
                               .t_arranque_s = 0.4f, .t_confirma_s = 0.03f, .t_max_s = 20, .w_quieto = 0.3f };

/* Planta de la prueba: motor + tornillo. seat = vuelta en la que asienta (<0: nunca); strip = torque en que se barre. */
typedef struct { double w, th; double duty; } pl_t;
static double par_tornillo(double vueltas, double seat, double k, double strip)
{
    if (seat < 0 || vueltas < seat) return 0.03;
    double t = 0.03 + k * (vueltas - seat) * 2 * M_PI;
    return (strip > 0 && t > strip) ? 0.0 : t;
}
static screw_result_t correr(double seat, double strip, bool enc_ok, screw_t *s, double *t_fin)
{
    pl_t p = { 0, 0, 0 };
    const double J = 0.0049, dt = 1e-3;
    double vueltas_ant = 0;
    screw_start(s, &M, &C);
    for (int k = 0; k < 40000 && screw_activo(s); k++) {
        double w_med = enc_ok ? p.w : 0.0;
        double v = enc_ok ? p.th / (2 * M_PI) : 0.0;
        float u = screw_step(s, &M, (float)dt, (float)p.duty, (float)w_med, (float)(v - vueltas_ant));
        vueltas_ant = v;
        double d = u - p.duty; if (d > 3) d = 3; if (d < -3) d = -3;
        p.duty += d;
        for (int n = 0; n < 10; n++) {
            double V = p.duty / 1023.0 * 12.0;
            double i = (V - M.ke * p.w) / M.r;
            double carga = par_tornillo(p.th / (2 * M_PI), seat, 0.32, strip) + M.kt * M.i0 * (p.w > 0.05 ? 1 : p.w / 0.05);
            p.w += (M.kt * i - carga) / J * dt / 10;
            if (p.w < 0) p.w = 0;
            p.th += p.w * dt / 10;
        }
        *t_fin = k * dt;
    }
    return s->res;
}

int main(void)
{
    printf("== Modelo ==\n");
    float w0 = 200.0f * 2 * (float)M_PI / 60.0f;
    CHECK(fabsf(screw_corriente(&M, 1023, w0) - 0.2f) < 0.01f, "en vacio a 12 V y 200 rpm: 0,2 A (%.3f)", screw_corriente(&M, 1023, w0));
    CHECK(fabsf(screw_corriente(&M, 1023, 0) - 5.5f) < 0.01f, "calado a 12 V: 5,5 A (%.3f)", screw_corriente(&M, 1023, 0));
    CHECK(fabsf(screw_estimar(&M, 1023, 0).torque - 2.06f) < 0.02f, "calado a 12 V: 21 kg cm = 2,06 N m (%.3f)", screw_estimar(&M, 1023, 0).torque);
    CHECK(screw_estimar(&M, 1023, w0).torque == 0.0f, "en vacio no hay torque en el tornillo");
    screw_est_t e = screw_estimar(&M, screw_duty_para_torque(&M, 0.4f), 0);
    CHECK(fabsf(e.torque - 0.4f) < 0.002f, "el PWM de apriete da el torque pedido al calar (%.3f)", e.torque);
    CHECK(fabsf(e.fuerza - (0.4f - 0.03f) / (0.2f * 0.003f)) < 2.0f, "fuerza = (T - T rosca)/(K d) (%.0f N)", e.fuerza);
    CHECK(fabsf(e.presion - e.fuerza / 15.7f) < 0.1f, "presion = F/A (%.1f MPa)", e.presion);
    CHECK(fabsf(screw_profundidad_mm(&M, 12) - 6.0f) < 1e-3f, "12 vueltas de paso 0,5 = 6 mm");

    printf("== Atornillado ==\n");
    screw_t s; double t = 0;
    screw_result_t r = correr(12, 0, true, &s, &t);
    CHECK(r == SCREW_R_OK, "tornillo normal: %s en %.1f s, %.2f vueltas, %.3f N m", screw_result_str(r), t, s.vueltas, s.torque_final);
    CHECK(fabsf(s.torque_final - C.torque_meta) < 0.06f, "torque final cerca de la meta (%.3f)", s.torque_final);
    CHECK(s.torque_max < 1.5f * C.torque_meta, "sin pico de torque por la inercia (%.3f)", s.torque_max);
    CHECK(fabsf(s.vueltas - 12) < 1.0f, "vueltas cerca de la meta (%.2f)", s.vueltas);
    CHECK(s.duty == 0.0f && !screw_activo(&s), "termina con PWM 0");

    r = correr(-1, 0, true, &s, &t);
    CHECK(r == SCREW_R_SIN_APRIETE, "sin tornillo: %s a las %.1f vueltas", screw_result_str(r), s.vueltas);
    CHECK(s.vueltas <= C.vueltas_max + 0.5f, "no sigue girando sin fin");

    r = correr(3, 0, true, &s, &t);
    CHECK(r == SCREW_R_TRABADO, "se traba a 3 vueltas: %s", screw_result_str(r));
    CHECK(s.vueltas < 4.0f, "se detiene enseguida (%.2f vueltas)", s.vueltas);

    r = correr(12, 0.25, true, &s, &t);
    CHECK(r == SCREW_R_SIN_APRIETE, "rosca que se barre: %s, nunca reporta ok", screw_result_str(r));

    r = correr(12, 0, false, &s, &t);
    CHECK(r == SCREW_R_TRABADO, "encoder sin senal: %s, nunca reporta ok", screw_result_str(r));
    CHECK(t < 1.0, "lo detecta rapido (%.2f s)", t);

    screw_start(&s, &M, &C);
    screw_step(&s, &M, 0.001f, 100, 1, 0.001f);
    screw_cancel(&s);
    CHECK(!screw_activo(&s) && s.res == SCREW_R_CANCELADO && s.duty == 0.0f, "cancelar deja PWM 0");
    CHECK(screw_step(&s, &M, 0.001f, 0, 0, 0) == 0.0f, "inactiva no pide PWM");

    screw_t lento;
    screw_cfg_t c2 = C; c2.t_max_s = 1.0f;
    screw_start(&lento, &M, &c2);
    for (int k = 0; k < 1200 && screw_activo(&lento); k++) screw_step(&lento, &M, 0.001f, 650, 13.0f, 0.0f);
    CHECK(lento.res == SCREW_R_TIEMPO, "tiempo agotado: %s", screw_result_str(lento.res));

    screw_t ov; screw_cfg_t c3 = C; c3.i_max = 0.5f; c3.torque_asiento = 5.0f;
    screw_start(&ov, &M, &c3);
    for (int k = 0; k < 1000 && screw_activo(&ov); k++) screw_step(&ov, &M, 0.001f, 300, 1.0f, 0.0f);
    CHECK(ov.res == SCREW_R_CORRIENTE, "sobrecorriente: %s", screw_result_str(ov.res));

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
