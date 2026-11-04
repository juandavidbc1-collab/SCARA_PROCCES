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

#include "../sim/sim_core.h"

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
    memset(P, 0, sizeof P); g_stby = false; g_now = 0; g_phys_max_exceso = 0; g_power = true; g_tool_duty = g_tool_max = 0; g_lock[0] = g_lock[1] = false;
    g_screw_mode = SIM_TORNILLO_NORMAL; g_m3_enc_ok = true; g_servo_deg = -1.0f; g_m3_w = g_m3_th = 0; g_m3_th_tornillo = NAN; g_m3_barrido = false;
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
    CHECK(fabs(S.q[0]) < 0.02f && fabs(S.q[1]) < 0.02f, "termina en 0 0");
    for (int j = 0; j < NJ; j++) {
        float lo, hi; ctl_get_limits(j, &lo, &hi);
        double qh = JC[j].home_dir * JC[j].cuentas_a_cero / JC[j].k * 180.0 / M_PI;   /* q en el flanco de liberacion del 1er tope */
        double lo_f = qh + 3, hi_f = qh + (sw_max_deg[j] - sw_min_deg[j]) - 3;   /* limites 3 deg dentro de cada tope */
        printf("  eje %d: limites medidos %.2f a %.2f deg (esperado ~%.1f a ~%.1f)\n", j + 1, lo * 180 / M_PI, hi * 180 / M_PI, lo_f, hi_f);
        CHECK(fabs(lo * 180 / M_PI - lo_f) < 2.5 && fabs(hi * 180 / M_PI - hi_f) < 2.5, "recorrido medido correcto");
    }
    CHECK(g_phys_max_exceso < 5.0, "no llega al tope duro (5 deg tras el final de carrera) durante el Home (maximo %.2f deg)", g_phys_max_exceso);

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
    CHECK(g_phys_max_exceso < 5.0, "sin golpe mecánico (máximo %.2f deg pasado del interruptor)", g_phys_max_exceso);

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

    /* ============ 6) Las tres formas del RST y el PID de respaldo ============ */
    printf("\n== ESCENARIO 6: RST posicional, incremental, PI-D y enmascarado; PID de respaldo ==\n");
    for (int f = 0; f <= RST_FORM_MASK + 1; f++) {
        reset_sim(); enc_desconectado[0] = enc_desconectado[1] = false; correr(0.3, "boot", false);
        const char *nom = f == RST_FORM_MASK + 1 ? "PID" : RST_FORM_NAME[f];
        if (f == RST_FORM_MASK + 1) ctl_cmd_ctrl(CTRL_PID); else ctl_cmd_rstform((rst_form_t)f);
        correr(0.01, "cfg", false);
        ctl_cmd_home(); correr(120, "home", false);
        CHECK(S.mode == M_HOLD && S.homed[0] && S.homed[1], "%s: Home completo", nom);
        float q1[2] = { 0.6f, 1.2f }, x0, y0, x1, y1;
        ctl_cmd_move(q1, (float[]){NAN, NAN, NAN}); correr(6, "mov", false);
        kin_fk(&KIN, q1[0], q1[1], &x0, &y0); kin_fk(&KIN, S.q[0], S.q[1], &x1, &y1);
        double e = hypot(x1 - x0, y1 - y0);
        CHECK(e < 0.8 && S.mode == M_HOLD, "%s: goto con error %.3f mm", nom, e);
    }
    ctl_cmd_ctrl(CTRL_RST);

    /* ============ 7) Paro de emergencia ============ */
    printf("\n== ESCENARIO 7: paro de emergencia ==\n");
    reset_sim(); correr(0.3, "boot", false);
    ctl_cmd_home(); correr(1.0, "home", false);
    CHECK(S.mode == M_HOMING, "(en pleno Home)");
    ctl_cmd_estop(); correr(0.002, "paro", false);
    CHECK(S.mode == M_FAULT && S.fault == F_ESTOP, "paro durante Home gana en 2 ms");
    correr(0.3, "paro", false);
    CHECK(!g_stby && P[0].mode == 0 && P[1].mode == 0, "tras el freno: STBY = 0 y motores libres");
    CHECK(!S.homed[0] && !S.homed[1], "un Home interrumpido no deja referencia");
    ctl_cmd_clear(); correr(0.01, "clear", false);
    CHECK(S.mode == M_OFF && S.fault == F_NONE, "clear rearma");

    reset_sim(); correr(0.3, "boot", false); ctl_cmd_home(); correr(120, "home", false);
    float qh[2] = { S.q[0], S.q[1] };
    ctl_cmd_estop(); correr(0.5, "paro", false);
    CHECK(S.homed[0] && S.homed[1], "paro en HOLD conserva el Home");
    ctl_cmd_move(qh, (float[]){NAN, NAN, NAN}); correr(0.1, "paro", false);
    CHECK(S.mode == M_FAULT, "enclavado: no acepta movimientos hasta clear");
    ctl_cmd_clear(); correr(0.01, "clear", false);
    float q2t[2] = { 0.5f, 1.0f };
    ctl_cmd_move(q2t, (float[]){NAN, NAN, NAN}); correr(5, "tras rearme", false);
    CHECK(S.mode == M_HOLD && fabsf(S.q[0] - 0.5f) < 0.02f, "tras rearmar se mueve sin repetir Home");

    reset_sim(); correr(0.3, "boot", false);
    set_fault(F_STALL); correr(0.01, "f", false);
    ctl_cmd_estop(); correr(0.002, "paro", false);
    CHECK(S.fault == F_ESTOP, "paro manda sobre una falla previa");

    reset_sim(); correr(0.3, "boot", false);
    for (int k = 0; k < 20; k++) ctl_cmd_stop();                     /* cola de órdenes llena */
    ctl_cmd_estop(); correr(0.002, "paro", false);
    CHECK(S.mode == M_FAULT && S.fault == F_ESTOP, "paro funciona con la cola de ordenes llena");

    reset_sim(); correr(0.3, "boot", false); ctl_cmd_home(); correr(120, "home", false);
    for (int k = 0; k < 20; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }
    ctl_cmd_estop();
    for (int k = 0; k < 15; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }   /* sigue pulsado */
    float qp = S.q[0];
    ctl_cmd_clear();
    for (int k = 0; k < 25; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }
    CHECK(fabsf(S.q[0] - qp) < 0.002f && S.mode == M_OFF, "tras clear con el dead-man aun pulsado no se mueve");
    ctl_cmd_jog(false, 0, 0); correr(0.4, "soltar", false);
    for (int k = 0; k < 25; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }
    CHECK(S.q[0] > qp + 0.02f, "soltar y volver a pulsar el dead-man si mueve");
    ctl_cmd_jog(false, 0, 0); correr(0.4, "soltar", false);

    reset_sim(); correr(0.3, "boot", false); ctl_cmd_home(); correr(120, "home", false);
    float qc0 = S.q[1], ucut = 0;
    g_power = false; correr(1.0, "12V cortados", false); g_power = true;
    for (int k = 0; k < 500; k++) { correr(0.001, "12V vuelven", false); ucut = fmaxf(ucut, fabsf(S.q[1] - qc0)); }
    CHECK(ucut < 0.03f, "cortar y devolver 12 V en HOLD no da salto (%.2f deg)", ucut * 180 / M_PI);
    float qc2[2] = { S.q[0] + 1.0f, S.q[1] };
    ctl_cmd_move(qc2, (float[]){NAN, NAN, NAN}); correr(0.3, "mov", false);
    g_power = false; correr(2.0, "12V cortados en movimiento", false);
    CHECK(S.mode == M_FAULT, "cortar 12 V en movimiento lleva a FAULT (%s)", ctl_fault_str(S.fault));
    float qc3 = phys_deg(0); g_power = true; correr(1.0, "12V vuelven", false);
    CHECK(fabs(phys_deg(0) - qc3) < 0.5, "al volver 12 V tras la falla no se mueve");

    reset_sim(); correr(0.3, "boot", false); ctl_cmd_home(); correr(120, "home", false);
    for (int k = 0; k < 25; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }
    ctl_cmd_stop();
    for (int k = 0; k < 10; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }   /* frena con la tecla aún pulsada */
    float qst = S.q[0];
    for (int k = 0; k < 25; k++) { ctl_cmd_jog(true, +1, 0); correr(0.02, "jog", false); }
    printf("  tras detener con tecla pulsada: q cambia %.4f rad\n", S.q[0] - qst);
    /* si el manual siguiera activo, en estos 0.5 s avanzaría 0.26 rad; lo que queda es el asentamiento en la posición de paro */
    CHECK(S.mode == M_HOLD && S.q[0] - qst < 0.05f, "detener con las teclas pulsadas: no sigue avanzando hasta soltar (modo %s)", ctl_mode_str(S.mode));
    ctl_cmd_jog(false, 0, 0); correr(0.4, "soltar", false);

    /* ============ 8) Activar y desactivar motores ============ */
    printf("\n== ESCENARIO 8: motores activados o desactivados ==\n");
    reset_sim(); correr(0.3, "boot", false);
    CHECK(S.en[0] && S.en[1] && !S.tool_en, "al arrancar J1 y J2 activos, M3 desactivado");
    ctl_cmd_motor(1, false); correr(0.01, "m", false);
    double f20 = phys_deg(1);
    ctl_cmd_home(); correr(120, "home", false);
    CHECK(S.mode == M_HOLD && S.homed[0] && !S.homed[1], "Home solo del eje activo");
    CHECK(fabs(phys_deg(1) - f20) < 0.1, "el eje desactivado no se mueve en el Home");
    float qd[2] = { 0.3f, 0.5f }, q2d = S.q[1];
    ctl_cmd_move(qd, (float[]){NAN, NAN, NAN}); correr(4, "ang", false);
    CHECK(S.mode == M_HOLD && fabsf(S.q[0] - 0.3f) < 0.02f, "ang mueve el eje activo");
    CHECK(fabsf(S.q[1] - q2d) < 0.001f, "ang no mueve el eje desactivado");
    P[1].th += 0.5 * NGEAR; correr(1.0, "a mano", false);
    CHECK(S.mode == M_HOLD, "mover a mano el eje desactivado no da falla");
    CHECK(P[1].mode == 0, "driver del eje desactivado libre");
    for (int k = 0; k < 25; k++) { ctl_cmd_jog(true, 0, +1); correr(0.02, "jog", false); }
    CHECK(P[1].mode == 0, "manual no mueve el eje desactivado");
    ctl_cmd_jog(false, 0, 0); correr(0.4, "soltar", false);
    float q2r = S.q[1], saltor = 0;
    ctl_cmd_motor(1, true);
    for (int k = 0; k < 500; k++) { correr(0.001, "reactiva", false); saltor = fmaxf(saltor, fabsf(S.q[1] - q2r)); }
    CHECK(S.mode == M_HOLD && saltor < 0.005f, "al reactivar mantiene la posicion sin salto (%.3f deg)", saltor * 180 / M_PI);

    reset_sim(); correr(0.3, "boot", false); ctl_cmd_home(); correr(120, "home", false);
    float qm[2] = { S.q[0] + 1.0f, S.q[1] + 1.0f };
    ctl_cmd_move(qm, (float[]){NAN, NAN, NAN}); correr(0.3, "mov", false);
    ctl_cmd_motor(1, false); correr(4, "mov", false);
    CHECK(S.mode == M_HOLD && S.fault == F_NONE, "desactivar en plena trayectoria no da falla");
    CHECK(fabsf(S.q[0] - qm[0]) < 0.02f, "el otro eje termina su movimiento");
    float qm2[2] = { S.q[0] - 1.0f, S.q[1] };
    ctl_cmd_move(qm2, (float[]){NAN, NAN, NAN}); correr(0.3, "mov", false);
    ctl_cmd_motor(1, true); correr(0.5, "reactiva en mov", false);
    CHECK(S.mode == M_HOLD && S.fault == F_NONE, "reactivar en plena trayectoria detiene y mantiene");

    reset_sim(); correr(0.3, "boot", false); ctl_cmd_home(); correr(1, "home", false);
    ctl_cmd_motor(1, false); correr(0.01, "m", false);
    CHECK(S.en[1] && S.mode == M_HOMING, "no se puede cambiar un motor durante el Home");
    ctl_cmd_estop(); correr(0.3, "p", false); ctl_cmd_clear(); correr(0.01, "c", false);

    reset_sim(); correr(0.3, "boot", false);
    ctl_cmd_motor(0, false); ctl_cmd_motor(1, false); correr(0.01, "m", false);
    ctl_cmd_home(); correr(0.1, "home", false);
    CHECK(S.mode == M_OFF, "Home sin ejes activos se rechaza");

    /* ============ 9) Motor 3 (atornillador) ============ */
    printf("\n== ESCENARIO 9: motor 3 ==\n");
    reset_sim(); correr(0.3, "boot", false);
    ctl_cmd_tool(50); correr(0.5, "m3", false);
    CHECK(g_tool_max == 0, "M3 desactivado no gira");
    ctl_cmd_motor(2, true); correr(0.01, "m3", false);
    ctl_cmd_tool(50); correr(0.5, "m3", false);
    CHECK(g_tool_duty == TOOL_DUTY_MAX / 2 && g_stby, "M3 activo gira, con el brazo apagado (duty %d, STBY %d)", g_tool_duty, g_stby);
    CHECK(S.mode == M_OFF && P[0].mode == 0, "el brazo sigue apagado");
    correr(TOOL_MAX_ON_MS / 1000.0 + 0.5, "m3", false);
    CHECK(g_tool_duty == 0 && !g_stby, "M3 se apaga solo si no se renueva la orden (comunicacion perdida)");
    ctl_cmd_tool(50); correr(0.5, "m3", false); ctl_cmd_estop(); correr(0.002, "paro", false);
    CHECK(g_tool_duty == 0, "paro apaga M3 de inmediato");
    ctl_cmd_tool(50); correr(0.5, "paro", false);
    CHECK(g_tool_duty == 0, "M3 no arranca durante el paro");
    ctl_cmd_clear(); correr(0.01, "c", false);
    CHECK(S.tool_cmd == 0, "tras rearmar M3 sigue parado");
    ctl_cmd_tool(-40); correr(0.5, "m3", false);
    CHECK(g_tool_duty < 0, "M3 gira en ambos sentidos");
    ctl_cmd_off(); correr(0.01, "off", false);
    CHECK(g_tool_duty == 0, "off apaga M3");

    /* ============ 10) Atornillado con servo y modelo de corriente ============ */
    printf("\n== ESCENARIO 10: servo y atornillado ==\n");
    #define ESPERA_TORNILLO(max_s) do { for (int t = 0; t < (int)((max_s) * 1000) && screw_activo(&S.screw); t++) correr(0.001, "m3", false); correr(0.3, "m3", false); } while (0)
    reset_sim(); correr(0.1, "boot", false);
    S.m3_ref = true;                      /* como tras "m3 home" */
    CHECK(g_servo_deg == SERVO_ABAJO_DEG, "el servo arranca abajo, sin punto medio");
    ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.01, "s", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(0.05, "m3", false);
    CHECK(!screw_activo(&S.screw), "sin bajar el destornillador no atornilla");
    ctl_cmd_servo(SERVO_ABAJO_DEG); correr(0.01, "s", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(0.05, "m3", false);
    CHECK(!screw_activo(&S.screw) && g_tool_max == 0, "M3 desactivado: no atornilla");
    ctl_cmd_motor(2, true); correr(0.01, "m3", false);
    ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.01, "s", false);
    CHECK(g_servo_deg == SERVO_ARRIBA_DEG, "el servo sube");
    ctl_cmd_servo(SERVO_ABAJO_DEG); correr(0.01, "s", false);
    CHECK(!ctl_cmd_servo(200.0f) && !ctl_cmd_servo(-200.0f), "angulo de servo fuera de rango se rechaza");

    g_m3_th_tornillo = NAN;               /* el tornillo de la prueba de arriba ya no está */
    ctl_cmd_screw(12.0f, 0.4f); correr(0.01, "m3", false);
    CHECK(screw_activo(&S.screw), "atornilla con M3 activo, aun con el brazo apagado");
    ESPERA_TORNILLO(25);
    CHECK(S.screw.res == SCREW_R_SIN_APRIETE, "sin soltar tornillo con el servo: %s", screw_result_str(S.screw.res));
    CHECK(g_tool_duty == 0, "M3 queda apagado");

    ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.1, "s", false); ctl_cmd_servo(SERVO_ABAJO_DEG); correr(0.1, "s", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(0.15, "m3", false);
    {
        ctl_status_t st; ctl_get_status(&st);
        CHECK(st.tool_i > 0.6f && st.tool_fuerza == 0.0f && st.tool_presion == 0.0f && st.tool_torque == 0.0f,
              "al arrancar la corriente (%.2f A) acelera el motor: no se reporta como fuerza (%.0f N) ni torque (%.3f N m)", st.tool_i, st.tool_fuerza, st.tool_torque);
    }
    ctl_cmd_pulse(0); correr(0.02, "m3", false);
    CHECK(S.mode != M_PULSE, "mientras atornilla rechaza el pulso de identificacion");
    correr(1.85, "m3", false);
    {
        ctl_status_t st; ctl_get_status(&st);
        CHECK(st.screw_fase == SCREW_RAPIDO && st.tool_i > 0.1f && st.tool_i < 1.0f, "girando: estima la corriente (%.2f A) y publica la fase", st.tool_i);
        CHECK(fabs(st.tool_i - g_m3_i_real) < 0.05, "la corriente del modelo coincide con la de la planta (%.3f y %.3f A)", st.tool_i, g_m3_i_real);
        CHECK(st.tool_vueltas > 0.5f, "cuenta vueltas (%.2f)", st.tool_vueltas);
    }
    ctl_cmd_tool(50); ctl_cmd_home(); ctl_cmd_screw(12.0f, 0.4f); correr(0.02, "m3", false);
    CHECK(S.tool_cmd == S.screw.duty && S.mode != M_HOMING, "mientras atornilla rechaza tool, home y otro tornillo");
    ESPERA_TORNILLO(25);
    CHECK(S.screw.res == SCREW_R_OK, "tornillo bien: %s, %.2f vueltas, %.3f N m", screw_result_str(S.screw.res), S.screw.vueltas, S.screw.torque_final);
    CHECK(fabsf(S.screw.torque_final - SCREW_TORQUE_META) < 0.08f, "torque final cerca de la meta (%.3f N m)", S.screw.torque_final);
    CHECK(fabs(S.screw.torque_final - g_m3_torque_real) < 0.08, "el torque del modelo coincide con el de la planta (%.3f y %.3f N m)", S.screw.torque_final, g_m3_torque_real);
    CHECK(fabsf(S.screw.fuerza_final - (S.screw.torque_final - M3_MODEL.torque_rosca) / (M3_MODEL.k_tuerca * M3_MODEL.d_mm * 1e-3f)) < 5.0f, "fuerza coherente con el torque (%.0f N)", S.screw.fuerza_final);
    {
        ctl_status_t st; ctl_get_status(&st);
        CHECK(fabsf(st.tool_prof - st.tool_vueltas * M3_MODEL.paso_mm) < 0.01f, "publica la profundidad (%.2f mm)", st.tool_prof);
        CHECK(st.tool_fuerza > 100.0f && st.tool_presion > 5.0f, "el resultado sigue visible al terminar (%.0f N, %.1f MPa)", st.tool_fuerza, st.tool_presion);
        CHECK(st.screw_fase == SCREW_FIN && st.tool_i == 0.0f, "motor quieto: corriente 0");
    }

    g_screw_mode = SIM_TORNILLO_ATASCO; ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.1, "s", false); ctl_cmd_servo(SERVO_ABAJO_DEG); correr(0.1, "s", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(0.01, "m3", false); ESPERA_TORNILLO(25);
    CHECK(S.screw.res == SCREW_R_TRABADO, "atasco a pocas vueltas: %s", screw_result_str(S.screw.res));
    g_screw_mode = SIM_TORNILLO_BARRIDO; ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.1, "s", false); ctl_cmd_servo(SERVO_ABAJO_DEG); correr(0.1, "s", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(0.01, "m3", false); ESPERA_TORNILLO(25);
    CHECK(S.screw.res == SCREW_R_SIN_APRIETE, "rosca barrida: %s", screw_result_str(S.screw.res));
    g_screw_mode = SIM_TORNILLO_NORMAL; g_m3_enc_ok = false; ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.1, "s", false); ctl_cmd_servo(SERVO_ABAJO_DEG); correr(0.1, "s", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(0.01, "m3", false); ESPERA_TORNILLO(25);
    CHECK(S.screw.res != SCREW_R_OK && S.screw.res != SCREW_R_NONE, "encoder de M3 sin senal: nunca dice ok (%s)", screw_result_str(S.screw.res));
    g_m3_enc_ok = true;

    /* paro y otras interrupciones a mitad del tornillo */
    ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.1, "s", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(0.01, "m3", false);
    CHECK(!screw_activo(&S.screw), "con el destornillador arriba no atornilla");
    ctl_cmd_servo(SERVO_ABAJO_DEG); correr(0.1, "s", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(1.0, "m3", false);
    ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.01, "m3", false);
    CHECK(g_servo_deg == SERVO_ABAJO_DEG && screw_activo(&S.screw), "mientras atornilla, el destornillador no sube");
    ctl_cmd_estop(); correr(0.002, "paro", false);
    CHECK(g_tool_duty == 0 && S.screw.res == SCREW_R_CANCELADO, "paro: M3 parado y tornillo cancelado");
    ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.02, "paro", false);
    CHECK(g_servo_deg == SERVO_ABAJO_DEG, "durante el paro el destornillador no sube");
    ctl_cmd_screw(12.0f, 0.4f); correr(0.05, "paro", false);
    CHECK(!screw_activo(&S.screw), "durante el paro no atornilla");
    ctl_cmd_clear(); correr(0.01, "c", false);
    ctl_cmd_screw(12.0f, 0.4f); correr(1.0, "m3", false); ctl_cmd_stop(); correr(0.01, "m3", false);
    CHECK(S.screw.res == SCREW_R_CANCELADO && S.tool_cmd == 0, "detener cancela el tornillo");
    correr(0.4, "m3", false); CHECK(g_tool_duty == 0, "y M3 se apaga");
    ctl_cmd_screw(12.0f, 0.4f); correr(1.0, "m3", false); ctl_cmd_motor(2, false); correr(0.01, "m3", false);
    CHECK(S.screw.res == SCREW_R_CANCELADO && g_tool_duty == 0, "desactivar M3 cancela el tornillo");
    ctl_cmd_motor(2, true); ctl_cmd_screw(12.0f, 0.4f); correr(0.5, "m3", false);
    ctl_cmd_screw(12.0f, 0.4f);
    { float qm[2] = { 0.1f, 0.2f }; ctl_cmd_move(qm, (float[]){NAN, NAN, NAN}); }
    correr(0.05, "m3", false);
    CHECK(S.mode != M_MOVE, "no mueve el brazo mientras atornilla");
    ctl_cmd_screw(5.0f, 0.2f); ctl_cmd_stop(); correr(0.01, "m3", false);
    ctl_cmd_screw(5.0f, 0.2f); correr(0.01, "m3", false);
    CHECK(fabsf(S.screw.cfg.vueltas_meta - 5.0f) < 1e-3f && fabsf(S.screw.cfg.torque_meta - 0.2f) < 1e-3f, "acepta vueltas y torque propios");
    ctl_cmd_stop(); correr(0.01, "m3", false);
    CHECK(!ctl_cmd_screw(-1.0f, 0.2f) && !ctl_cmd_screw(5.0f, 3.0f), "valores fuera de rango se rechazan");

    /* ============ 11) El RST por defecto produce el mismo PWM que el PID original ============ */
    printf("\n== ESCENARIO 11: PWM del RST frente al PID original ==\n");
    {
        /* Mismas entradas a los dos controladores: el PID manda en el lazo y el RST corre en paralelo sin actuar. */
        reset_sim();
        ctl_cmd_ctrl(CTRL_PID); correr(0.05, "c", false);
        ctl_cmd_home(); for (int t = 0; t < 45000 && !(S.mode == M_HOLD && S.homed[0] && S.homed[1]); t++) correr(0.001, "home", false);
        rst_ctl_t par[NJ];
        for (int j = 0; j < NJ; j++) { rst_cargar(j, RST_FORM_MASK); par[j] = S.rst[j]; rst_reset(&par[j], S.q[j]); pid_reset(&S.pid[j]); S.qdref[j] = 0; S.qd[j] = 0; }
        float q[2] = { 0.6f, 1.2f };
        ctl_cmd_move(q, (float[]){NAN, NAN, NAN});
        double dmax = 0, sum = 0; int nn = 0;
        for (int k = 0; k < 9000; k++) {
            correr(0.001, "goto", false);
            if (k < 2 || S.mode != M_MOVE) continue;          /* pid_step ya usó la q de este tick */
            for (int j = 0; j < NJ; j++) {
                float e = S.qref[j] - S.q[j];
                float base = S.pid[j].kff * S.qdref[j] + S.pid[j].kp * e + S.pid[j].kd * (S.qdref[j] - S.qd[j]) + S.pid[j].i;
                float u_rst = rst_step(&par[j], S.qref[j], S.q[j], 1e9f);
                double d = fabs(base - u_rst);
                if (k > 50) { sum += d * d; nn++; if (d > dmax) dmax = d; }
            }
        }
        double rms = sqrt(sum / nn);
        printf("  misma entrada: diferencia de salida RST-PID rms %.3f, maxima %.3f PWM\n", rms, dmax);
        CHECK(rms < 2.0 && dmax < 3.0, "con las mismas entradas el RST calcula el PWM del PID original (rms %.3f, max %.3f)", rms, dmax);
    }
    {
        /* En lazo cerrado el ruido del encoder se desfasa entre corridas; se compara lo que se ve: forma y pico del PWM. */
        enum { NPWM = 12000 };
        static float pwm[2][NPWM][NJ];
        for (int caso = 0; caso < 2; caso++) {
            reset_sim();
            if (caso == 0) ctl_cmd_ctrl(CTRL_PID);
            correr(0.05, "c", false);
            ctl_cmd_home();
            for (int t = 0; t < 45000 && !(S.mode == M_HOLD && S.homed[0] && S.homed[1]); t++) correr(0.001, "home", false);
            float q[2] = { 0.6f, 1.2f };
            ctl_cmd_move(q, (float[]){NAN, NAN, NAN});
            float f[NJ] = { 0, 0 };
            for (int k = 0; k < NPWM; k++) {
                correr(0.001, "goto", false);
                for (int j = 0; j < NJ; j++) { f[j] += 0.02f * (S.duty[j] - f[j]); pwm[caso][k][j] = f[j]; }   /* filtro de 50 ms */
            }
            CHECK(fabsf(S.q[0] - q[0]) < 0.01f && fabsf(S.q[1] - q[1]) < 0.01f, "%s llega", caso ? "RST" : "PID");
        }
        double dmax = 0, sum = 0, umax[2] = { 0, 0 };
        for (int k = 0; k < NPWM; k++) for (int j = 0; j < NJ; j++) {
            double d = fabs(pwm[0][k][j] - pwm[1][k][j]);
            if (d > dmax) dmax = d;
            sum += d * d;
            for (int c = 0; c < 2; c++) if (fabs(pwm[c][k][j]) > umax[c]) umax[c] = fabs(pwm[c][k][j]);
        }
        double rms = sqrt(sum / (NPWM * NJ));
        printf("  PWM filtrado 50 ms: pico PID %.0f, pico RST %.0f; diferencia rms %.2f, maxima %.1f\n", umax[0], umax[1], rms, dmax);
        CHECK(rms < 10.0 && fabs(umax[0] - umax[1]) < 0.1 * umax[0], "en lazo cerrado el PWM del RST tiene la forma y el pico del PID original");
    }

    /* ============ Numeros que no son numeros, y falla con el servo abierto ============ */
    printf("\n== ESCENARIO: NaN en las ordenes y servo en una falla ==\n");
    reset_sim(); correr(0.1, "nan", false);
    CHECK(!ctl_cmd_tool(NAN), "tool NaN se rechaza");
    {   /* la cola es de 8 ordenes: se vacia antes de cada intento, o un rechazo por cola llena pasaria por un rechazo por NaN */
        correr(0.002, "nan", false);
        CHECK(ctl_cmd_gain(0, 100, 100, 1, 10, 10), "gain valida se acepta");
        const float g[5] = { 100, 100, 1, 10, 10 };
        for (int i = 0; i < 5; i++) {
            float v[5]; memcpy(v, g, sizeof v); v[i] = NAN;
            correr(0.002, "nan", false);
            CHECK(!ctl_cmd_gain(0, v[0], v[1], v[2], v[3], v[4]), "gain con NaN en el campo %d se rechaza", i);
        }
        const float h[7] = { 100, 100, 1, 0, 1, 0, 10 };
        correr(0.002, "nan", false);
        CHECK(ctl_cmd_rst(0, h), "rst valido se acepta");
        for (int i = 0; i < 7; i++) {
            float v[7]; memcpy(v, h, sizeof v); v[i] = NAN;
            correr(0.002, "nan", false);
            CHECK(!ctl_cmd_rst(0, v), "rst con NaN en el campo %d se rechaza", i);
        }
    }
    reset_sim(); correr(0.1, "servo", false);
    ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.01, "servo", false);
    ctl_cmd_estop(); correr(0.002, "paro", false);
    CHECK(g_servo_deg == SERVO_ABAJO_DEG, "un paro baja el destornillador");
    reset_sim(); correr(0.1, "servo", false);
    ctl_cmd_servo(SERVO_ARRIBA_DEG); correr(0.01, "servo", false);
    CHECK(g_servo_deg == SERVO_ARRIBA_DEG, "destornillador arriba antes de la falla");
    set_fault(F_STALL); correr(0.01, "servo", false);
    CHECK(g_servo_deg == SERVO_ABAJO_DEG, "una falla baja el destornillador");

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
