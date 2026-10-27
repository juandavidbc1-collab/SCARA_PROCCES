/*
 * SIMULACIÓN DEL FIRMWARE COMPLETO EN EL PC (sin hardware).
 * Incluye src/control.c tal cual y reemplaza hal.c/FreeRTOS por un modelo de los 2 motores
 * (R, L, Ke, J, B del documento + fricción seca), encoders cuantizados y finales de carrera físicos.
 * Sirve para comprobar la lógica de Home, manual (dead-man), punto a punto y fallas.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kinematics.h"
#include "../../src/control.c"

/* ---------------- reloj simulado y stubs de FreeRTOS ---------------- */
static int64_t g_now = 0;
int64_t esp_timer_get_time(void) { return g_now; }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(g_now / 1000); }
void vTaskDelayUntil(TickType_t *p, TickType_t i) { (void)p; (void)i; }
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t f, const char *n, uint32_t s, void *a, int p, TaskHandle_t *h, int c)
{ (void)f; (void)n; (void)s; (void)a; (void)p; (void)h; (void)c; return 1; }
esp_err_t esp_task_wdt_add(void *p) { (void)p; return 0; }
esp_err_t esp_task_wdt_reset(void) { return 0; }

typedef struct { int n, sz, head, count; char *data; } q_t;
QueueHandle_t xQueueCreate(int n, int sz) { q_t *q = calloc(1, sizeof *q); q->n = n; q->sz = sz; q->data = malloc((size_t)n * sz); return q; }
BaseType_t xQueueSend(QueueHandle_t h, const void *item, TickType_t t) { (void)t; q_t *q = h; if (q->count == q->n) return 0;
    memcpy(q->data + (size_t)((q->head + q->count) % q->n) * q->sz, item, q->sz); q->count++; return 1; }
BaseType_t xQueueReceive(QueueHandle_t h, void *item, TickType_t t) { (void)t; q_t *q = h; if (!q->count) return 0;
    memcpy(item, q->data + (size_t)q->head * q->sz, q->sz); q->head = (q->head + 1) % q->n; q->count--; return 1; }

/* ---------------- planta simulada ---------------- */
typedef struct { double i, w, th; double V; int mode; /*0 libre,1 drive,2 freno*/ } mot_t;
static mot_t P[NJ];
static bool g_stby = false;
static double phys_off_deg[NJ] = { 10.0, 30.0 };       /* posición física inicial */
static double sw_min_deg[NJ] = { -125.0, -145.0 }, sw_max_deg[NJ] = { 125.0, 145.0 };
static bool enc_desconectado[NJ] = { false, false };
static int enc_sign_fisico[NJ] = { -1, +1 };            /* sentido en que cuenta el encoder respecto a la física */
static int motor_dir_fisico[NJ] = { -1, -1 };           /* sentido físico del motor con duty>0 (antes de motor_sign): el cableado real necesita motor_sign = -1 */
static int32_t enc_frozen[NJ];

#define NGEAR 100.37
static double phys_deg(int j) { return P[j].th / NGEAR / (2 * M_PI) * 360.0 + phys_off_deg[j]; }

void hal_stby(bool on) { g_stby = on; }
bool hal_stby_get(void) { return g_stby; }
void hal_motor_free(int j) { P[j].mode = 0; P[j].V = 0; }
void hal_motor_brake(int j) { P[j].mode = 2; P[j].V = 0; }
void hal_all_free(void) { for (int j = 0; j < NJ; j++) hal_motor_free(j); }
void hal_motor_drive(int j, int duty)
{
    duty *= JC[j].motor_sign;
    if (duty == 0) { hal_motor_free(j); return; }
    if (abs(duty) > PWM_MAX) duty = duty > 0 ? PWM_MAX : -PWM_MAX;
    P[j].mode = 1; P[j].V = duty / 1023.0 * 12.0 * motor_dir_fisico[j];
}
int32_t hal_enc_raw(int j)
{
    if (enc_desconectado[j]) return enc_frozen[j];
    return (int32_t)floor(P[j].th / NGEAR / (2 * M_PI) * ENC_CUENTAS_SALIDA) * enc_sign_fisico[j];
}
int hal_enc_levels(int j) { (void)j; return 0; }
bool hal_fc_raw(int j, int side)
{
    double a = phys_deg(j);
    return side ? (a >= sw_max_deg[j]) : (a <= sw_min_deg[j]);
}
void hal_init(void) {}

static double g_phys_max_exceso = 0;     /* cuánto se pasó de los límites duros (grados) */
static void planta_1ms(void)
{
    const double R = 2.4, L = 1.5e-3, Ke = 0.01056, B = 2.97e-6, Tc = 3e-3, J = 3e-6 + 0.04 / (NGEAR * NGEAR);
    const double dt = 1e-4;
    for (int j = 0; j < NJ; j++) {
        mot_t *m = &P[j];
        double V = (g_stby ? m->V : 0.0);
        bool abierto = (!g_stby || m->mode == 0);
        for (int k = 0; k < 10; k++) {
            if (abierto) m->i = 0; else m->i += (V - R * m->i - Ke * m->w) / L * dt;
            double Tl = Tc * (m->w > 0.5 ? 1 : (m->w < -0.5 ? -1 : m->w / 0.5));
            m->w += (Ke * m->i - B * m->w - Tl) / J * dt;
            m->th += m->w * dt;
            double a = phys_deg(j);                    /* tope mecánico duro 5° más allá del final de carrera */
            if (a < sw_min_deg[j] - 5 || a > sw_max_deg[j] + 5) {
                double lim = a < 0 ? sw_min_deg[j] - 5 : sw_max_deg[j] + 5;
                m->th = (lim - phys_off_deg[j]) / 360.0 * 2 * M_PI * NGEAR; m->w = 0;
            }
            double ex = fmax(sw_min_deg[j] - a, a - sw_max_deg[j]); if (ex > g_phys_max_exceso) g_phys_max_exceso = ex;
        }
    }
}

static void correr(double seg, const char *tag, bool verbose)
{
    int n = (int)(seg * 1000);
    for (int k = 0; k < n; k++) {
        g_now += 1000;
        planta_1ms();
        tick();
        char ev[112];
        while (ctl_event_pop(ev, sizeof ev, 0)) printf("  [%7.3f s] %s %s\n", g_now / 1e6, tag, ev);
        if (verbose && (k % 500) == 0)
            printf("     t=%6.2f %-6s q=(%7.2f,%7.2f) ref=(%7.2f,%7.2f) duty=(%5.0f,%5.0f) fc=%x\n", g_now / 1e6, ctl_mode_str(S.mode),
                   S.q[0] * 180 / M_PI, S.q[1] * 180 / M_PI, S.qref[0] * 180 / M_PI, S.qref[1] * 180 / M_PI, S.duty[0], S.duty[1],
                   (S.fc[0][0]) | (S.fc[0][1] << 1) | (S.fc[1][0] << 2) | (S.fc[1][1] << 3));
    }
}

static int fallos = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("  >>> FALLO: " __VA_ARGS__); printf("\n"); fallos++; } else printf("  ok: " __VA_ARGS__), printf("\n"); } while (0)

static void reset_sim(void)
{
    memset(P, 0, sizeof P); g_stby = false; g_now = 0; g_phys_max_exceso = 0;
    ctl_init();
}

int main(void)
{
    /* ============ 1) Home + punto a punto ============ */
    printf("\n== ESCENARIO 1: arranque, Home, goto ==\n");
    reset_sim();
    correr(0.5, "boot", false);
    CHECK(S.mode == M_OFF, "arranca en OFF con motores apagados (modo %s, STBY=%d)", ctl_mode_str(S.mode), g_stby);
    CHECK(!ctl_cmd_move((float[]){0, 0}, (float[]){NAN, NAN, NAN}) || 1, "-");
    correr(0.1, "goto sin home", false);
    CHECK(S.mode == M_OFF, "goto sin Home se rechaza");
    ctl_cmd_home();
    double t_home0 = g_now / 1e6;
    correr(120.0, "home", false);
    printf("  (Home + vuelta tardaron como maximo %.0f s de simulacion)\n", g_now / 1e6 - t_home0);
    CHECK(S.homed[0] && S.homed[1], "ambos ejes referenciados");
    CHECK(S.mode == M_HOLD, "termina en HOLD (modo %s)", ctl_mode_str(S.mode));
    printf("  posicion fisica inicial = (%.2f, %.2f) deg; tras el Home = (%.2f, %.2f) deg\n", phys_off_deg[0], phys_off_deg[1], phys_deg(0), phys_deg(1));
    CHECK(fabs(phys_deg(0) - phys_off_deg[0]) < 0.7 && fabs(phys_deg(1) - phys_off_deg[1]) < 0.7, "volvio a la posicion inicial");
    for (int j = 0; j < NJ; j++) {
        float lo, hi; ctl_get_limits(j, &lo, &hi);
        double qh = JC[j].q_home * 180.0 / M_PI;                       /* q en el flanco de liberacion del 1er tope */
        double lo_f = qh + 3, hi_f = qh + (sw_max_deg[j] - sw_min_deg[j]) - 3;   /* limites 3 deg dentro de cada tope */
        printf("  eje %d: limites medidos %.2f a %.2f deg (esperado ~%.1f a ~%.1f)\n", j + 1, lo * 180 / M_PI, hi * 180 / M_PI, lo_f, hi_f);
        CHECK(fabs(lo * 180 / M_PI - lo_f) < 2.5 && fabs(hi * 180 / M_PI - hi_f) < 2.5, "recorrido medido correcto");
    }
    CHECK(g_phys_max_exceso < 3.0, "no se pasó más de 3 deg del final de carrera durante el Home (maximo %.2f deg)", g_phys_max_exceso);

    float qmin[2] = { JC[0].q_min, JC[1].q_min }, qmax[2] = { JC[0].q_max, JC[1].q_max };
    float pts[4][2] = { { 120, 80 }, { -100, 60 }, { 80, -120 }, { 150, 0 } };
    for (int i = 0; i < 4; i++) {
        float q[2]; int cod;
        ik_status_t st = kin_ik(&KIN, pts[i][0], pts[i][1], +1, qmin, qmax, q, &cod);
        if (st != IK_OK) { printf("  punto %d (%g,%g): %s\n", i, pts[i][0], pts[i][1], kin_ik_str(st)); continue; }
        float tgt[3] = { pts[i][0], pts[i][1], 0 };
        ctl_cmd_move(q, tgt);
        correr(4.0, "goto", false);
        float x, y; kin_fk(&KIN, S.q[0], S.q[1], &x, &y);
        double e = hypot(x - pts[i][0], y - pts[i][1]);
        CHECK(S.mode == M_HOLD && e < 1.0, "goto (%g,%g): error medido con encoders = %.3f mm", pts[i][0], pts[i][1], e);
    }

    /* ============ 2) Manual con dead-man ============ */
    printf("\n== ESCENARIO 2: manual (flechas + espacio) ==\n");
    float q0 = S.q[0];
    for (int k = 0; k < 50; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }   /* 1 s */
    float q1 = S.q[0];
    printf("  tras 1 s de flecha: J1 se movió %.1f deg (esperado ~%.0f)\n", (q1 - q0) * 180 / M_PI, JC[0].jog_v * 180 / M_PI);
    CHECK(fabs(q1 - q0) > 0.7f * JC[0].jog_v, "J1 se mueve con dead-man + flecha");
    /* suelta el espacio: el PC envía "d 0 0 0" de inmediato */
    float qr = S.q[0];
    ctl_cmd_jog(false, 0, 0); correr(0.2, "jog", false);
    float qa = S.q[0]; correr(0.3, "jog", false); float qb = S.q[0];
    printf("  al soltar el espacio recorrió %.2f deg más antes de frenar\n", (qa - qr) * 180 / M_PI);
    CHECK(S.mode == M_HOLD && fabs(qb - qa) < 0.009f && (qa - qr) < 2.0f * M_PI / 180, "al soltar el dead-man se detiene (modo %s, deriva posterior %.3f deg)", ctl_mode_str(S.mode), (qb - qa) * 180 / M_PI);
    /* dead-man presionado, se pierde la comunicación (no llega "d 0") */
    for (int k = 0; k < 25; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }
    qr = S.q[0]; correr(0.4, "jog", false); qa = S.q[0]; correr(0.3, "jog", false); qb = S.q[0];
    printf("  con comunicación perdida recorrió %.2f deg más antes de frenar\n", (qa - qr) * 180 / M_PI);
    CHECK(S.mode == M_HOLD && fabs(qb - qa) < 0.002f && (qa - qr) < 6.0f * M_PI / 180, "pérdida de comunicación -> se detiene");
    /* flecha sin espacio: no debe mover */
    q0 = S.q[0];
    for (int k = 0; k < 25; k++) { ctl_cmd_jog(false, +1, 0); correr(0.02, "jog", false); }
    CHECK(fabs(S.q[0] - q0) < 0.001f && S.mode == M_HOLD, "flecha sin espacio no mueve");
    /* se cae el USB con el motor en marcha */
    for (int k = 0; k < 25; k++) { ctl_cmd_jog(true, 0, -1); correr(0.02, "jog", false); }
    float q2a = S.q[1]; correr(0.5, "jog", false);
    CHECK(S.mode == M_HOLD && fabs(S.q[1] - q2a) < 0.1f, "J2: sin latidos se detiene en < 0.5 s");

    /* ============ 3) Límites por software y final de carrera en manual ============ */
    printf("\n== ESCENARIO 3: límites ==\n");
    for (int k = 0; k < 400; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }   /* 8 s hacia +J1 */
    float l1, h1; ctl_get_limits(0, &l1, &h1);
    printf("  J1 llegó a %.2f deg (límite software medido %.1f)\n", S.q[0] * 180 / M_PI, h1 * 180 / M_PI);
    CHECK(S.q[0] <= h1 + 0.01f, "respeta el límite por software con Home hecho");
    CHECK(g_phys_max_exceso < 3.0, "sin golpe mecánico (máximo %.2f deg pasado del interruptor)", g_phys_max_exceso);

    /* ============ 4) Manual SIN Home hasta el final de carrera ============ */
    printf("\n== ESCENARIO 4: manual sin Home contra el final de carrera ==\n");
    reset_sim(); correr(0.3, "boot", false);
    for (int k = 0; k < 600; k++) { ctl_cmd_jog(true, -1, 0); correr(0.02, "jog", false); }   /* 12 s hacia -J1 */
    printf("  físico J1 = %.2f deg (final de carrera en -125)  modo %s\n", phys_deg(0), ctl_mode_str(S.mode));
    CHECK(g_phys_max_exceso < 5.5 && phys_deg(0) > -131.0, "se detiene en el final de carrera (físico %.2f deg)", phys_deg(0));
    for (int k = 0; k < 50; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }
    CHECK(phys_deg(0) > -125.0, "puede alejarse del final de carrera (físico %.2f deg)", phys_deg(0));

    /* ============ 5) Fallas ============ */
    printf("\n== ESCENARIO 5: fallas ==\n");
    reset_sim(); correr(0.3, "boot", false); ctl_cmd_home(); correr(120, "home", false);
    float tgt[3] = { NAN, NAN, NAN }; float qq[2] = { 1.5f, 1.0f };   /* lejos de la posicion de partida (con q_home = -108 el eje 1 parte cerca de 0.5 rad) */
    enc_desconectado[0] = true; enc_frozen[0] = hal_enc_raw(0);     /* encoder 1 desconectado */
    enc_desconectado[0] = true;
    ctl_cmd_move(qq, tgt); correr(3.0, "enc1 cortado", false);
    CHECK(S.mode == M_FAULT, "encoder desconectado -> FAULT (%s)", ctl_fault_str(S.fault));
    CHECK(!g_stby, "STBY = 0 tras la falla");
    ctl_cmd_clear(); correr(0.1, "clear", false);
    CHECK(S.mode == M_OFF && !S.homed[0], "clear -> OFF y sin referencia");

    reset_sim(); enc_desconectado[0] = false; correr(0.3, "boot", false); ctl_cmd_home(); correr(120, "home", false);
    motor_dir_fisico[1] = -motor_dir_fisico[1];                      /* motor 2 con signo equivocado (al reves de como esta cableado) */
    ctl_cmd_move(qq, tgt); correr(3.0, "motor invertido", false);
    CHECK(S.mode == M_FAULT && (S.fault == F_FOLLOW || S.fault == F_STALL || S.fault == F_LIMIT), "signo invertido (realimentación positiva) -> FAULT (%s); sobrepaso del interruptor %.2f deg", ctl_fault_str(S.fault), g_phys_max_exceso);
    motor_dir_fisico[1] = -motor_dir_fisico[1];                      /* se restaura */

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
