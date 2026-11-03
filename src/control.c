#include "control.h"
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "hal.h"
#include "pid.h"
#include "rst.h"
#include "rst_defaults.h"
#include "screw.h"
#include "traj.h"

#define DT (1.0f / CONTROL_HZ)

typedef enum { C_HOME, C_MOVE, C_STOP, C_OFF, C_CLEAR, C_PULSE, C_GAIN, C_MOTOR, C_TOOL, C_SERVO, C_SCREW, C_CTRL, C_RSTFORM, C_RST } cmd_type_t;
typedef struct {
    cmd_type_t t;
    float q[NJ];
    float tgt[3];
    int   j;
    float g[7];
} cmd_t;

typedef enum { H_SEEK = 0, H_BRAKE, H_BACKOFF, H_SETTLE } home_phase_t;

static QueueHandle_t cmd_q, evt_q;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

/* latido del modo manual (escrito por la tarea serie, leído por el lazo) */
static struct { bool dm; int8_t dir[NJ]; int64_t t_us; } hb;

/* estado visible (copia protegida con lock) */
static ctl_status_t pub;
static float pub_gain[NJ][5];
static float pub_rst[NJ][7];
static volatile bool estop_req = false;   /* el paro no pasa por la cola: no se puede perder ni esperar */

static const float RST_WEIGHTS[4][2] = { { 1.0f, 1.0f }, { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 0.0f } };   /* (b, c) */

/* ---------------------------------------------------------------- estado interno del lazo */
static struct {
    ctl_mode_t mode;
    fault_t fault;
    bool homed[NJ];
    int32_t off[NJ];                 /* cuentas que corresponden a q = 0 */
    int32_t counts[NJ];
    float q[NJ], qd[NJ];             /* medidos */
    float qref[NJ], qdref[NJ];
    float duty[NJ];                  /* PWM actual (con rampa) */
    float u[NJ];                     /* PWM deseado de este ciclo */
    bool brake[NJ];                  /* true: freno activo en este ciclo */
    pid_ctl_t pid[NJ];
    ctrl_t ctrl;
    rst_form_t form;
    rst_ctl_t rst[NJ];
    float u_rst[NJ];                 /* salida del RST: se mantiene entre muestras */
    bool ctrl_vivo[NJ];              /* el controlador corrió en este tick; si no, su historia se reinicia */
    bool en[NJ];                     /* motor activado */
    bool tool_en;
    float tool_cmd, tool_duty;
    int64_t tool_t_us;
    int32_t tool_counts;
    float tool_w;                    /* velocidad de M3, eje de salida [rad/s], positiva al apretar */
    bool tool_init;
    screw_t screw;
    screw_est_t tool_est;            /* con el tornillo terminado, se queda el resultado para mostrarlo */
    float servo_deg;
    bool jog_lock;                   /* tras una falla o paro: exige soltar el dead-man antes de volver a mover */
    bool fc[NJ][2];                  /* con antirrebote */
    uint8_t fc_cnt[NJ][2];
    traj_t traj;
    int64_t t_traj_us;
    float tgt[3];
    bool await_reach;
    int64_t t_reach_us, t_inband_us;
    /* homing */
    int h_step; int h_stage; home_phase_t h_phase; int64_t h_t_phase_us, h_t_start_us;
    int32_t c_start[NJ];             /* cuentas al iniciar el Home: a ahi se vuelve al terminar */
    float qlim_min[NJ], qlim_max[NJ]; /* limites por software vigentes (se miden en el Home) */
    float edge0[NJ];                 /* q del borde de liberacion del primer tope */
    int32_t h_c_prev; int64_t h_t_prev_us;
    /* pulse */
    int p_j; int64_t p_t0_us; int32_t p_c0; float p_q0;
    /* supervisión */
    int follow_ms[NJ];
    int32_t stall_c[NJ]; int64_t stall_t_us[NJ];
    int64_t fault_t_us;
    uint32_t ticks;
    bool qd_init;
} S;

static const int HOME_ORDER[NJ] = HOME_ORDER_INIT;

/* ---------------------------------------------------------------- utilidades */
static int64_t now_us(void) { return esp_timer_get_time(); }

static void emit(const char *fmt, ...)
{
    char b[160];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    xQueueSend(evt_q, b, 0);
}

const char *ctl_mode_str(ctl_mode_t m)
{
    static const char *n[] = { "OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT" };
    return (m <= M_FAULT) ? n[m] : "?";
}

const char *ctl_fault_str(fault_t f)
{
    switch (f) {
    case F_NONE:         return "ninguna";
    case F_FOLLOW:       return "error de seguimiento (trabado, encoder o motor con signo invertido)";
    case F_STALL:        return "motor trabado (PWM alto y el encoder no cuenta)";
    case F_LIMIT:        return "final de carrera inesperado (se perdio la referencia)";
    case F_HOME_TIMEOUT: return "tiempo de Home agotado";
    case F_DIAG_FC:      return "MIN y MAX del mismo eje presionados a la vez (cableado o final de carrera)";
    case F_ESTOP:        return "paro de emergencia";
    }
    return "?";
}

static bool all_homed(void)
{
    for (int j = 0; j < NJ; j++) if (S.en[j] && !S.homed[j]) return false;
    return true;
}

static bool algun_eje_activo(void)
{
    for (int j = 0; j < NJ; j++) if (S.en[j]) return true;
    return false;
}

static void screw_informar(void)
{
    const screw_t *s = &S.screw;
    emit("#TORNILLO: %s vueltas=%.2f prof=%.2fmm avance=%.0f%% torque=%.3fNm fuerza=%.0fN presion=%.1fMPa corriente=%.2fA t=%.1fs",
         screw_result_str(s->res), s->vueltas, screw_profundidad_mm(&M3_MODEL, s->vueltas),
         100.0f * s->vueltas / s->cfg.vueltas_meta, s->torque_final, s->fuerza_final, s->presion_final, s->i_final, s->t);
}

static void tool_detener(void)
{
    S.tool_cmd = 0.0f;
    if (screw_activo(&S.screw)) { screw_cancel(&S.screw); screw_informar(); }
}

static void tool_apagar(void)
{
    tool_detener();
    S.tool_duty = 0.0f;
    hal_tool_free();
}

static void servo_mover(float deg)
{
    S.servo_deg = deg;
    hal_servo_set_deg(deg);
}

static void ctrl_reset(int j)
{
    pid_reset(&S.pid[j]);
    rst_reset(&S.rst[j], S.q[j]);
    S.u_rst[j] = 0.0f;
}

/* Una articulación con el controlador vigente. El RST recalcula cada RST_PERIOD_TICKS y mantiene entre muestras. */
static void control_eje(int j)
{
    if (!S.en[j]) {
        S.qref[j] = S.q[j]; S.qdref[j] = 0.0f; S.u[j] = 0.0f;
        return;
    }
    S.ctrl_vivo[j] = true;
    if (S.ctrl == CTRL_PID) {
        S.u[j] = pid_step(&S.pid[j], S.qref[j] - S.q[j], S.qdref[j], S.qd[j], DT, DUTY_MAX);
        return;
    }
    if ((S.ticks % RST_PERIOD_TICKS) == 0) {
        /* La compensación de fricción va por fuera del RST, con la misma regla de pid.c: es parte del PWM original. */
        float u = rst_step(&S.rst[j], S.qref[j], S.q[j], DUTY_MAX);
        if (fabsf(S.qref[j] - S.q[j]) > S.pid[j].e_zone && u != 0.0f) u += copysignf(S.pid[j].fric, u);
        if (u >  DUTY_MAX) u =  DUTY_MAX;
        if (u < -DUTY_MAX) u = -DUTY_MAX;
        S.u_rst[j] = u;
    }
    S.u[j] = S.u_rst[j];
}

/* Cada forma tiene sus propios polos: ganancias de rst_defaults.h y pesos (b, c). */
static void rst_cargar(int j, rst_form_t f)
{
    const float *g = RST_GAINS[f][j];
    rst_set_pid(&S.rst[j], g[0], g[1], g[2], g[3], RST_WEIGHTS[f][0], RST_WEIGHTS[f][1], g[4], RST_TS_S);
}

static uint8_t ctrl_code(void)
{
    return S.ctrl == CTRL_PID ? 0 : (uint8_t)(1 + S.form);
}

static void set_fault(fault_t f)
{
    if (S.mode == M_FAULT) return;
    for (int j = 0; j < NJ; j++) { hal_motor_brake(j); S.homed[j] = false; S.duty[j] = 0; S.u[j] = 0; }
    S.mode = M_FAULT;
    S.fault = f;
    S.fault_t_us = now_us();
    S.await_reach = false;
    S.jog_lock = true;
    tool_apagar();
    servo_mover(SERVO_ABAJO_DEG);
    emit("#FALLA: %s. Motores apagados. Escribe 0 (o clear) para borrarla.", ctl_fault_str(f));
}

/* Paro de emergencia: manda sobre cualquier otro estado, incluida una falla ya activa. */
static void entrar_paro(void)
{
    if (S.mode == M_FAULT && S.fault == F_ESTOP) return;
    bool en_home = (S.mode == M_HOMING);
    for (int j = 0; j < NJ; j++) {
        hal_motor_brake(j); S.duty[j] = 0; S.u[j] = 0;
        if (en_home) S.homed[j] = false;              /* un Home a medias no deja referencia */
    }
    tool_apagar();
    servo_mover(SERVO_ABAJO_DEG);
    S.mode = M_FAULT;
    S.fault = F_ESTOP;
    S.fault_t_us = now_us();
    S.await_reach = false;
    S.jog_lock = true;
    emit("#FALLA: PARO DE EMERGENCIA. Motores apagados. Usa clear para rearmar.");
}

static void enter_hold(void)
{
    /* la referencia actual se conserva; solo se anula la velocidad */
    for (int j = 0; j < NJ; j++) S.qdref[j] = 0.0f;
    S.mode = M_HOLD;
}

static void start_pid_from_here(void)
{
    for (int j = 0; j < NJ; j++) {
        ctrl_reset(j);
        S.qref[j] = S.q[j];
        S.qdref[j] = 0.0f;
        S.duty[j] = 0.0f;
        S.follow_ms[j] = 0;
        S.stall_c[j] = S.counts[j];
        S.stall_t_us[j] = now_us();
    }
}

/* ---------------------------------------------------------------- sensores */
static void leer_sensores(void)
{
    const float a = DT / (VEL_FILTRO_TAU_S + DT);
    for (int j = 0; j < NJ; j++) {
        int32_t c = hal_enc_raw(j);
        S.counts[j] = c;
        float qn = JC[j].enc_sign * (float)(c - S.off[j]) / JC[j].k;
        if (!S.qd_init) { S.qd[j] = 0.0f; }
        else { S.qd[j] += a * ((qn - S.q[j]) / DT - S.qd[j]); }
        S.q[j] = qn;
        for (int s = 0; s < 2; s++) {
            bool raw = hal_fc_raw(j, s);
            if (raw == S.fc[j][s]) { S.fc_cnt[j][s] = 0; }
            else if (++S.fc_cnt[j][s] >= FC_ANTIREBOTE_MS * CONTROL_HZ / 1000) {
                S.fc[j][s] = raw; S.fc_cnt[j][s] = 0;
            }
        }
    }
    S.qd_init = true;

    int32_t c3 = TOOL_ENC_SIGN * hal_tool_enc_raw();
    float dv = S.tool_init ? (float)(c3 - S.tool_counts) / TOOL_ENC_CUENTAS : 0.0f;
    S.tool_counts = c3;
    S.tool_init = true;
    S.tool_w += DT / (TOOL_VEL_TAU_S + DT) * (dv * 2.0f * (float)M_PI / DT - S.tool_w);
    /* El atornillado también se decide aquí: se calcula con el PWM de verdad aplicado en el ciclo anterior. */
    if (screw_activo(&S.screw)) {
        S.tool_cmd = screw_step(&S.screw, &M3_MODEL, DT, S.tool_duty, S.tool_w, dv);
        if (!screw_activo(&S.screw)) { S.tool_cmd = 0.0f; screw_informar(); }
    }
    if (S.tool_duty != 0.0f) {
        const float sgn = S.tool_duty > 0.0f ? 1.0f : -1.0f;
        S.tool_est = screw_estimar(&M3_MODEL, fabsf(S.tool_duty), sgn * S.tool_w);
        if (screw_activo(&S.screw) && S.screw.t < S.screw.cfg.t_arranque_s) {
            /* El motor está acelerando: esa corriente no va al tornillo. */
            S.tool_est.torque = S.tool_est.fuerza = S.tool_est.presion = 0.0f;
        }
    } else {
        memset(&S.tool_est, 0, sizeof S.tool_est);
    }
    if (S.screw.fase == SCREW_FIN && S.tool_cmd == 0.0f) {   /* el PWM baja con la rampa: el resultado es lo medido antes */
        S.tool_est.torque = S.screw.torque_final; S.tool_est.fuerza = S.screw.fuerza_final; S.tool_est.presion = S.screw.presion_final;
    }
}

/* ---------------------------------------------------------------- PID de todas las articulaciones */
static void pid_todos(void)
{
    for (int j = 0; j < NJ; j++) control_eje(j);
}

/* ---------------------------------------------------------------- órdenes */
static void do_cmd(const cmd_t *c)
{
    switch (c->t) {
    case C_HOME:
        if (screw_activo(&S.screw)) { emit("#ERROR: M3 esta atornillando: espera o detenlo."); break; }
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: no se puede iniciar Home, el robot esta en modo %s (usa 8 para detener).", ctl_mode_str(S.mode)); break; }
        if (!algun_eje_activo()) { emit("#ERROR: no hay ejes activos para hacer Home."); break; }
        for (int j = 0; j < NJ; j++) { if (S.en[j]) S.homed[j] = false; S.u[j] = 0; }
        for (int j = 0; j < NJ; j++) S.c_start[j] = S.counts[j];
        S.h_step = 0; S.h_stage = 0; S.h_phase = H_SEEK;
        while (!S.en[HOME_ORDER[S.h_step]]) S.h_step++;
        S.h_t_phase_us = S.h_t_start_us = now_us();
        S.h_c_prev = S.counts[HOME_ORDER[S.h_step]]; S.h_t_prev_us = now_us();
        for (int j = 0; j < NJ; j++) S.duty[j] = 0;
        S.mode = M_HOMING;
        hal_stby(true);
        emit("#HOME: iniciando. Eje %d: primero un tope, luego el otro, y vuelve a donde estaba. (cualquier tecla = detener)", HOME_ORDER[S.h_step] + 1);
        break;

    case C_MOVE:
        if (screw_activo(&S.screw)) { emit("#ERROR: M3 esta atornillando: el brazo no se mueve hasta que termine."); break; }
        if (!all_homed()) { emit("#ERROR: primero ejecuta HOME (opcion 5)."); break; }
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: el robot esta ocupado (modo %s). Detenlo primero (opcion 8).", ctl_mode_str(S.mode)); break; }
        if (S.mode == M_OFF) start_pid_from_here();
        {
            float vmax[NJ], amax[NJ];
            for (int j = 0; j < NJ; j++) { vmax[j] = JC[j].vmax; amax[j] = JC[j].amax; }
            float q0[NJ], q1[NJ];
            for (int j = 0; j < NJ; j++) {
                q0[j] = S.en[j] ? S.qref[j] : S.q[j];
                q1[j] = S.en[j] ? c->q[j] : S.q[j];           /* un eje desactivado no participa */
            }
            bool mueve = traj_plan(&S.traj, q0, q1, vmax, amax);
            memcpy(S.tgt, c->tgt, sizeof S.tgt);
            S.t_traj_us = now_us();
            S.mode = M_MOVE;
            S.await_reach = true; S.t_inband_us = 0;
            S.t_reach_us = now_us() + (int64_t)(S.traj.T * 1e6f) + REACH_TIMEOUT_MS * 1000LL;
            hal_stby(true);
            if (!mueve) { for (int j = 0; j < NJ; j++) S.qref[j] = q1[j]; }
        }
        break;

    case C_STOP:
        tool_detener();
        S.jog_lock = true;                             /* que no reanude solo si las teclas siguen pulsadas */
        if (S.mode == M_HOMING || S.mode == M_PULSE) {
            for (int j = 0; j < NJ; j++) { hal_motor_free(j); S.duty[j] = 0; S.u[j] = 0; }
            S.mode = M_OFF; hal_stby(false);
            emit("#INFO: proceso detenido, motores apagados.");
        } else if (S.mode == M_MOVE || S.mode == M_JOG) {
            for (int j = 0; j < NJ; j++) S.qref[j] = S.q[j];
            S.await_reach = false;
            enter_hold();
            emit("#INFO: movimiento detenido, manteniendo posicion.");
        }
        break;

    case C_OFF:
        if (S.mode != M_FAULT) {
            for (int j = 0; j < NJ; j++) { hal_motor_free(j); S.duty[j] = 0; S.u[j] = 0; }
            tool_apagar();
            S.jog_lock = true;
            S.mode = M_OFF; S.await_reach = false; hal_stby(false);
            emit("#INFO: motores apagados (STBY = 0).");
        }
        break;

    case C_CLEAR:
        if (S.mode == M_FAULT) {
            bool paro = (S.fault == F_ESTOP);
            S.mode = M_OFF; S.fault = F_NONE;
            if (paro) {
                bool ref = true;
                for (int j = 0; j < NJ; j++) if (S.en[j] && !S.homed[j]) ref = false;
                emit(ref ? "#INFO: paro rearmado. La referencia se conserva; los motores siguen apagados."
                         : "#INFO: paro rearmado. Faltan ejes por referenciar: ejecuta HOME.");
            } else {
                emit("#INFO: falla borrada. Los ejes quedaron SIN referencia: ejecuta HOME de nuevo.");
            }
        }
        break;

    case C_PULSE:
        if (screw_activo(&S.screw)) { emit("#ERROR: M3 esta atornillando: espera o detenlo."); break; }
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: no se puede hacer el pulso, el robot esta ocupado."); break; }
        if (!S.en[c->j]) { emit("#ERROR: el eje %d esta desactivado.", c->j + 1); break; }
        S.p_j = c->j; S.p_t0_us = now_us(); S.p_c0 = S.counts[c->j]; S.p_q0 = S.q[c->j];
        for (int j = 0; j < NJ; j++) { S.duty[j] = 0; S.u[j] = 0; }
        S.mode = M_PULSE; hal_stby(true);
        break;

    case C_GAIN:
        S.pid[c->j].kp = c->g[0]; S.pid[c->j].ki = c->g[1]; S.pid[c->j].kd = c->g[2];
        S.pid[c->j].kff = c->g[3]; S.pid[c->j].fric = c->g[4];
        pid_reset(&S.pid[c->j]);
        break;

    case C_MOTOR: {
        if (S.mode == M_HOMING || S.mode == M_PULSE) { emit("#ERROR: no se puede cambiar un motor durante %s.", ctl_mode_str(S.mode)); break; }
        bool on = c->g[0] != 0.0f;
        if (c->j == NJ) {                              /* motor 3 */
            S.tool_en = on;
            if (!on) tool_apagar();
            emit("#INFO: M3 %s.", on ? "activado" : "desactivado");
            break;
        }
        int j = c->j;
        if (S.en[j] == on) break;
        if (on) {
            S.en[j] = true;
            S.qref[j] = S.q[j]; S.qdref[j] = 0.0f; S.duty[j] = 0.0f;
            ctrl_reset(j);
            S.follow_ms[j] = 0; S.stall_c[j] = S.counts[j]; S.stall_t_us[j] = now_us();
            if (S.mode == M_MOVE || S.mode == M_JOG) {    /* la trayectoria en curso no contaba con este eje */
                for (int i = 0; i < NJ; i++) S.qref[i] = S.q[i];
                S.await_reach = false;
                enter_hold();
                emit("#INFO: movimiento detenido al reactivar J%d.", j + 1);
            }
        } else {
            S.en[j] = false;
            hal_motor_free(j); S.duty[j] = 0.0f; S.u[j] = 0.0f;
            S.tgt[0] = S.tgt[1] = S.tgt[2] = NAN;         /* ya no se llegará a la (x, y) pedida */
            if (!algun_eje_activo() && (S.mode == M_HOLD || S.mode == M_MOVE || S.mode == M_JOG)) {
                S.mode = M_OFF; S.await_reach = false;
            }
        }
        emit("#INFO: J%d %s.", j + 1, on ? "activado" : "desactivado");
        break;
    }

    case C_TOOL:
        if (S.mode == M_FAULT) { emit("#ERROR: M3 no arranca con una falla o paro activo."); break; }
        if (!S.tool_en) { emit("#ERROR: M3 esta desactivado (usa motor 3 on)."); break; }
        if (screw_activo(&S.screw)) { emit("#ERROR: M3 esta atornillando: espera o detenlo."); break; }
        S.tool_cmd = c->g[0] * (float)TOOL_DUTY_MAX / 100.0f;
        S.tool_t_us = now_us();
        break;

    case C_SERVO:
        if (c->g[0] != SERVO_ABAJO_DEG) {
            if (S.mode == M_FAULT) { emit("#ERROR: con una falla o paro activo el destornillador no sube."); break; }
            if (screw_activo(&S.screw)) { emit("#ERROR: el destornillador no sube mientras atornilla."); break; }
        }
        servo_mover(c->g[0]);
        break;

    case C_SCREW:
        if (S.mode == M_FAULT) { emit("#ERROR: no se atornilla con una falla o paro activo."); break; }
        if (S.servo_deg != SERVO_ABAJO_DEG) { emit("#ERROR: baja el destornillador antes de atornillar."); break; }
        if (!S.tool_en) { emit("#ERROR: M3 esta desactivado (usa motor 3 on)."); break; }
        if (screw_activo(&S.screw)) { emit("#ERROR: M3 ya esta atornillando."); break; }
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: el brazo debe estar quieto para atornillar (modo %s).", ctl_mode_str(S.mode)); break; }
        {
            screw_cfg_t cfg = SCREW_CFG;
            if (c->g[0] > 0.0f) {
                cfg.vueltas_meta = c->g[0];
                cfg.vueltas_min = 0.75f * c->g[0];
                cfg.vueltas_max = c->g[0] + 2.0f;
            }
            if (c->g[1] > 0.0f) cfg.torque_meta = c->g[1];
            screw_start(&S.screw, &M3_MODEL, &cfg);
            S.tool_cmd = S.screw.duty;
        }
        break;

    case C_CTRL:
    case C_RSTFORM:
    case C_RST:
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: cambia el controlador con el robot en OFF o HOLD (ahora %s).", ctl_mode_str(S.mode)); break; }
        if (c->t == C_CTRL) {
            S.ctrl = (ctrl_t)c->j;
        } else if (c->t == C_RSTFORM) {
            S.form = (rst_form_t)c->j;
            for (int j = 0; j < NJ; j++) {
                rst_cargar(j, S.form);
            }
        } else {
            rst_set_pid(&S.rst[c->j], c->g[0], c->g[1], c->g[2], c->g[3], c->g[4], c->g[5], c->g[6], RST_TS_S);
            S.form = RST_FORM_CUSTOM;
        }
        for (int j = 0; j < NJ; j++) ctrl_reset(j);
        emit("#INFO: controlador: %s.", S.ctrl == CTRL_PID ? "PID" : RST_FORM_NAME[S.form]);
        break;
    }
}

/* ---------------------------------------------------------------- modos */
static void h_fase(home_phase_t f, int j)
{
    S.h_phase = f;
    S.h_t_phase_us = S.h_t_prev_us = now_us();
    S.h_c_prev = S.counts[j];
}

/*
 * HOME en tres tiempos, eje por eje (orden HOME_ORDER):
 *   etapa 0: va al tope de home_dir, frena, retrocede hasta liberarlo -> ahi q = q_home (referencia).
 *   etapa 1: va al tope contrario, frena, retrocede hasta liberarlo -> mide el recorrido real.
 * Al terminar los dos ejes, ambos vuelven a la posicion en que estaban al pulsar Home.
 */
static void modo_homing(void)
{
    int64_t t = now_us();
    int j = HOME_ORDER[S.h_step];
    int hd = JC[j].home_dir;                       /* -1 = MIN, +1 = MAX */
    int side0 = (hd < 0) ? 0 : 1;
    int side = S.h_stage ? 1 - side0 : side0;      /* tope que se busca ahora */
    int dir_seek = S.h_stage ? -hd : hd;           /* sentido de busqueda */

    /* las articulaciones ya referenciadas se mantienen con PID; las demas, libres */
    for (int i = 0; i < NJ; i++) {
        if (i == j) continue;
        S.qdref[i] = 0.0f;
        if (S.homed[i] && S.en[i]) control_eje(i);
        else S.u[i] = 0;
    }

    if ((t - S.h_t_start_us) > HOME_TIMEOUT_MS * 1000LL) { set_fault(F_HOME_TIMEOUT); return; }
    if (S.fc[j][1 - side]) { set_fault(F_LIMIT); return; }   /* tope contrario inesperado */

    switch (S.h_phase) {
    case H_SEEK:
        S.u[j] = dir_seek * (float)HOME_DUTY;
        if (S.fc[j][side]) { h_fase(H_BRAKE, j); S.u[j] = 0; }
        break;
    case H_BRAKE:
        S.u[j] = 0; S.brake[j] = true;
        if (t - S.h_t_phase_us > 250000) h_fase(H_BACKOFF, j);
        break;
    case H_BACKOFF:
        S.u[j] = -dir_seek * (float)HOME_DUTY_SLOW;
        if (!S.fc[j][side]) {
            /* borde de liberacion del tope */
            if (S.h_stage == 0) {
                S.off[j] = S.counts[j] - (int32_t)lroundf(JC[j].enc_sign * JC[j].q_home * JC[j].k);
                S.q[j] = JC[j].q_home;
                S.edge0[j] = JC[j].q_home;
                S.homed[j] = true;
                emit("#HOME: eje %d referenciado en el tope %s (q = %.2f deg). Ahora el tope contrario...",
                     j + 1, side ? "MAX" : "MIN", JC[j].q_home * 180.0f / (float)M_PI);
            } else {
                float e1 = S.q[j], e0 = S.edge0[j];
                float lo = fminf(e0, e1) + HOME_MARGEN_RAD, hi = fmaxf(e0, e1) - HOME_MARGEN_RAD;
                if (hi > lo) { portENTER_CRITICAL(&lock); S.qlim_min[j] = lo; S.qlim_max[j] = hi; portEXIT_CRITICAL(&lock); }
                emit("#HOME: eje %d recorrido medido: de %.2f a %.2f deg (total %.2f deg). Limites por software: %.2f a %.2f deg.",
                     j + 1, fminf(e0, e1) * 180.0f / (float)M_PI, fmaxf(e0, e1) * 180.0f / (float)M_PI,
                     fabsf(e1 - e0) * 180.0f / (float)M_PI, S.qlim_min[j] * 180.0f / (float)M_PI, S.qlim_max[j] * 180.0f / (float)M_PI);
            }
            S.qref[j] = S.q[j]; S.qdref[j] = 0; ctrl_reset(j);
            h_fase(H_SETTLE, j); S.u[j] = 0;
        }
        break;
    case H_SETTLE:
        S.u[j] = 0;
        if (t - S.h_t_phase_us > 250000) {
            S.qref[j] = S.q[j];
            if (S.h_stage == 0) {                      /* sigue con el tope contrario del mismo eje */
                S.h_stage = 1;
                h_fase(H_SEEK, j); S.h_t_start_us = t;
            } else {
                S.h_stage = 0;
                do { S.h_step++; } while (S.h_step < NJ && !S.en[HOME_ORDER[S.h_step]]);
                if (S.h_step >= NJ) {
                    /* terminaron los dos ejes: volver a donde estaban */
                    float vmax[NJ], amax[NJ], q0[NJ], q1[NJ];
                    for (int i = 0; i < NJ; i++) {
                        q1[i] = JC[i].enc_sign * (float)(S.c_start[i] - S.off[i]) / JC[i].k;
                        if (q1[i] < S.qlim_min[i]) q1[i] = S.qlim_min[i];
                        if (q1[i] > S.qlim_max[i]) q1[i] = S.qlim_max[i];
                        if (!S.en[i]) q1[i] = S.q[i];
                        vmax[i] = JC[i].vmax * 0.5f; amax[i] = JC[i].amax * 0.5f;
                        q0[i] = S.q[i]; S.qref[i] = S.q[i]; ctrl_reset(i);
                        S.stall_c[i] = S.counts[i]; S.stall_t_us[i] = t; S.follow_ms[i] = 0;
                    }
                    traj_plan(&S.traj, q0, q1, vmax, amax);
                    S.t_traj_us = t;
                    S.tgt[0] = S.tgt[1] = S.tgt[2] = NAN;
                    S.await_reach = true; S.t_inband_us = 0;
                    S.t_reach_us = t + (int64_t)(S.traj.T * 1e6f) + REACH_TIMEOUT_MS * 1000LL;
                    S.mode = M_MOVE;
                    emit("#HOME: terminado. Volviendo a la posicion inicial (q1 = %.2f, q2 = %.2f deg)...",
                         q1[0] * 180.0f / (float)M_PI, q1[1] * 180.0f / (float)M_PI);
                } else {
                    int jn = HOME_ORDER[S.h_step];
                    h_fase(H_SEEK, jn); S.h_t_start_us = t;
                    emit("#HOME: ahora el eje %d, buscando su primer tope...", jn + 1);
                }
            }
        }
        break;
    }

    /* trabado durante la busqueda: el eje no se mueve y no hay final de carrera */
    if (S.h_phase == H_SEEK || S.h_phase == H_BACKOFF) {
        if (t - S.h_t_phase_us > 350000 && t - S.h_t_prev_us > HOME_STALL_MS * 1000LL) {
            if (abs(S.counts[j] - S.h_c_prev) < 2) { set_fault(F_STALL); return; }
            S.h_c_prev = S.counts[j]; S.h_t_prev_us = t;
        }
    }
}

static void modo_jog(bool fresh_dm, const int8_t dir[NJ])
{
    bool moving = false;
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) continue;
        float vt = fresh_dm ? dir[j] * JC[j].jog_v : 0.0f;
        float dv = ((fabsf(vt) < fabsf(S.qdref[j])) ? JOG_STOP_ACCEL : JOG_ACCEL) * DT;
        if (S.qdref[j] < vt - dv) S.qdref[j] += dv;
        else if (S.qdref[j] > vt + dv) S.qdref[j] -= dv;
        else S.qdref[j] = vt;

        /* no avanzar hacia un final de carrera presionado */
        if ((S.fc[j][0] && S.qdref[j] < 0) || (S.fc[j][1] && S.qdref[j] > 0)) S.qdref[j] = 0;

        S.qref[j] += S.qdref[j] * DT;
        if (S.homed[j]) {
            if (S.qref[j] > S.qlim_max[j]) { S.qref[j] = S.qlim_max[j]; if (S.qdref[j] > 0) S.qdref[j] = 0; }
            if (S.qref[j] < S.qlim_min[j]) { S.qref[j] = S.qlim_min[j]; if (S.qdref[j] < 0) S.qdref[j] = 0; }
        }
        /* la referencia no puede alejarse más de JOG_LEAD_RAD de la medición (evita acumular error si algo frena el eje) */
        if (S.qref[j] > S.q[j] + JOG_LEAD_RAD) S.qref[j] = S.q[j] + JOG_LEAD_RAD;
        if (S.qref[j] < S.q[j] - JOG_LEAD_RAD) S.qref[j] = S.q[j] - JOG_LEAD_RAD;
        if (fabsf(S.qdref[j]) > 1e-4f) moving = true;
    }
    pid_todos();
    if (!moving && !fresh_dm) enter_hold();
    else if (!moving) { /* dead-man presionado sin flecha: se queda en JOG manteniendo posición */ }
}

static void modo_move(void)
{
    float t = (float)(now_us() - S.t_traj_us) * 1e-6f;
    traj_eval(&S.traj, t, S.qref, S.qdref);
    pid_todos();
    if (t >= S.traj.T) {
        for (int j = 0; j < NJ; j++) S.qdref[j] = 0;
        S.mode = M_HOLD;
    }
}

static void modo_hold(void)
{
    pid_todos();
    if (!S.await_reach) return;
    int64_t t = now_us();
    bool dentro = true;
    for (int j = 0; j < NJ; j++) if (S.en[j] && fabsf(S.qref[j] - S.q[j]) > HOLD_TOL_RAD) dentro = false;
    if (dentro) {
        if (S.t_inband_us == 0) S.t_inband_us = t;
        if (t - S.t_inband_us > REACH_SETTLE_MS * 1000LL) {
            float x, y; kin_fk(&KIN, S.q[0], S.q[1], &x, &y);
            S.await_reach = false;
            if (isnan(S.tgt[0]))
                emit("#LLEGO: q1=%.3f q2=%.3f deg | medido x=%.2f y=%.2f mm", S.q[0] * 180.0f / (float)M_PI, S.q[1] * 180.0f / (float)M_PI, x, y);
            else
                emit("#LLEGO: objetivo x=%.2f y=%.2f z=%.2f | medido x=%.2f y=%.2f | error ex=%.3f ey=%.3f mm | q1=%.3f q2=%.3f deg",
                     S.tgt[0], S.tgt[1], S.tgt[2], x, y, x - S.tgt[0], y - S.tgt[1],
                     S.q[0] * 180.0f / (float)M_PI, S.q[1] * 180.0f / (float)M_PI);
        }
    } else {
        S.t_inband_us = 0;
        if (t > S.t_reach_us) { S.await_reach = false; emit("#ERROR: no llego a la consigna a tiempo (revisa las ganancias con gain)."); }
    }
}

static void modo_pulse(void)
{
    int64_t t = now_us();
    int j = S.p_j;
    float el = (float)(t - S.p_t0_us) * 1e-3f;
    for (int i = 0; i < NJ; i++) S.u[i] = 0;
    float rampa = fminf(1.0f, el / 120.0f);
    S.u[j] = PULSE_DUTY * rampa;
    if (el >= PULSE_MS) {
        float dq = (S.q[j] - S.p_q0) * 180.0f / (float)M_PI;
        int32_t dc = S.counts[j] - S.p_c0;
        S.u[j] = 0; S.duty[j] = 0; hal_motor_free(j);
        S.mode = M_OFF; hal_stby(false);
        emit("#PULSO eje %d: cuentas=%ld, dq=%+.2f deg -> %s", j + 1, (long)dc, dq,
             (fabsf(dq) < 0.3f) ? "NO SE MOVIO: revisa 12 V en J15, motor y encoder"
             : (dq > 0 ? "OK: u>0 aumenta q (motor_sign correcto)" : "INVERTIDO: cambia motor_sign de este eje en config.c"));
    }
}

/* ---------------------------------------------------------------- supervisión y salida */
static void supervisar(void)
{
    if (S.mode != M_HOLD && S.mode != M_MOVE && S.mode != M_JOG) {
        for (int j = 0; j < NJ; j++) { S.follow_ms[j] = 0; S.stall_c[j] = S.counts[j]; S.stall_t_us[j] = now_us(); }
        return;
    }
    int64_t t = now_us();
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) continue;
        /* error de seguimiento */
        if (fabsf(S.qref[j] - S.q[j]) > FOLLOW_ERR_RAD) {
            if (++S.follow_ms[j] * 1000 / CONTROL_HZ > FOLLOW_ERR_MS) { set_fault(F_FOLLOW); return; }
        } else S.follow_ms[j] = 0;

        /* bloqueo: PWM alto y el encoder no cuenta */
        if (fabsf(S.duty[j]) < STALL_DUTY) { S.stall_c[j] = S.counts[j]; S.stall_t_us[j] = t; }
        else if (t - S.stall_t_us[j] > STALL_WINDOW_MS * 1000LL) {
            if (abs(S.counts[j] - S.stall_c[j]) < STALL_MIN_COUNTS) { set_fault(F_STALL); return; }
            S.stall_c[j] = S.counts[j]; S.stall_t_us[j] = t;
        }

        /* en MOVE, tocar un final de carrera significa que se perdió la referencia */
        if (S.mode == M_MOVE && (S.fc[j][0] || S.fc[j][1])) { set_fault(F_LIMIT); return; }
    }
}

static void salida_m3(void)
{
    if (!screw_activo(&S.screw) && S.tool_cmd != 0.0f && now_us() - S.tool_t_us > TOOL_MAX_ON_MS * 1000LL) {
        S.tool_cmd = 0.0f;
        emit("#INFO: M3 apagado: no se renovo la orden en %d ms.", TOOL_MAX_ON_MS);
    }
    float d = S.tool_cmd - S.tool_duty;
    if (d >  DUTY_SLEW_PER_MS) d =  DUTY_SLEW_PER_MS;
    if (d < -DUTY_SLEW_PER_MS) d = -DUTY_SLEW_PER_MS;
    S.tool_duty += d;
    hal_tool_drive((int)lroundf(S.tool_duty));
}

static void aplicar_salidas(void)
{
    if (S.mode == M_OFF) {
        hal_all_free();
        for (int j = 0; j < NJ; j++) S.duty[j] = 0;
        salida_m3();
        hal_stby(S.tool_duty != 0.0f);               /* M3 funciona con el brazo apagado */
        return;
    }
    if (S.mode == M_FAULT) {
        if (now_us() - S.fault_t_us < FALLA_FRENO_MS * 1000LL) return;   /* sigue frenando */
        hal_all_free(); hal_stby(false);
        return;
    }
    hal_stby(true);
    salida_m3();
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) { hal_motor_free(j); S.duty[j] = 0; continue; }
        if (S.brake[j]) { hal_motor_brake(j); S.duty[j] = 0; continue; }
        float u = S.u[j];
        /* prohibido empujar contra un final de carrera presionado: solo se puede alejarse */
        if (S.fc[j][0] && u < 0) u = 0;
        if (S.fc[j][1] && u > 0) u = 0;
        /* rampa de PWM */
        float d = u - S.duty[j];
        if (d >  DUTY_SLEW_PER_MS) d =  DUTY_SLEW_PER_MS;
        if (d < -DUTY_SLEW_PER_MS) d = -DUTY_SLEW_PER_MS;
        S.duty[j] += d;
        hal_motor_drive(j, (int)lroundf(S.duty[j]));
    }
}

static void publicar(void)
{
    portENTER_CRITICAL(&lock);
    pub.mode = S.mode; pub.fault = S.fault;
    pub.fc = 0;
    pub.en = (S.en[0] ? 1 : 0) | (S.en[1] ? 2 : 0) | (S.tool_en ? 4 : 0);
    pub.ctrl = ctrl_code();
    pub.tool_duty = S.tool_duty;
    pub.tool_i = S.tool_est.i; pub.tool_torque = S.tool_est.torque;
    pub.tool_fuerza = S.tool_est.fuerza; pub.tool_presion = S.tool_est.presion;
    pub.tool_vueltas = S.screw.vueltas; pub.tool_prof = screw_profundidad_mm(&M3_MODEL, S.screw.vueltas);
    pub.servo_deg = S.servo_deg; pub.screw_fase = (uint8_t)S.screw.fase;
    for (int j = 0; j < NJ; j++) {
        rst_get_pid(&S.rst[j], pub_rst[j]);
        pub.homed[j] = S.homed[j]; pub.q[j] = S.q[j]; pub.qref[j] = S.qref[j];
        pub.duty[j] = S.duty[j]; pub.counts[j] = S.counts[j];
        pub.fc |= (S.fc[j][0] ? 1 : 0) << (2 * j);
        pub.fc |= (S.fc[j][1] ? 1 : 0) << (2 * j + 1);
        pub_gain[j][0] = S.pid[j].kp; pub_gain[j][1] = S.pid[j].ki; pub_gain[j][2] = S.pid[j].kd;
        pub_gain[j][3] = S.pid[j].kff; pub_gain[j][4] = S.pid[j].fric;
    }
    portEXIT_CRITICAL(&lock);
}

/* ---------------------------------------------------------------- tick de 1 ms */
static void tick(void)
{
    leer_sensores();

    /* el paro va antes que cualquier orden o modo */
    if (estop_req) { estop_req = false; entrar_paro(); }

    cmd_t c;
    while (xQueueReceive(cmd_q, &c, 0) == pdTRUE) do_cmd(&c);

    /* autodiagnóstico: MIN y MAX del mismo eje presionados a la vez es imposible */
    if (S.ticks == 200) {
        for (int j = 0; j < NJ; j++) if (S.fc[j][0] && S.fc[j][1]) set_fault(F_DIAG_FC);
    }
    S.ticks++;

    /* latido del modo manual */
    bool fresh_dm; int8_t dir[NJ];
    portENTER_CRITICAL(&lock);
    fresh_dm = hb.dm && (now_us() - hb.t_us) < DEADMAN_TIMEOUT_MS * 1000LL;
    for (int j = 0; j < NJ; j++) dir[j] = hb.dir[j];
    portEXIT_CRITICAL(&lock);
    if (!fresh_dm) S.jog_lock = false;
    bool quiere_jog = false;
    for (int j = 0; j < NJ; j++) if (fresh_dm && !S.jog_lock && S.en[j] && dir[j] != 0) quiere_jog = true;
    if (screw_activo(&S.screw)) quiere_jog = false;   /* la punta está dentro del tornillo */

    if ((S.mode == M_OFF || S.mode == M_HOLD) && quiere_jog) {
        if (S.mode == M_OFF) start_pid_from_here();
        S.await_reach = false;
        S.mode = M_JOG;
    }

    for (int j = 0; j < NJ; j++) { S.u[j] = 0; S.brake[j] = false; }
    switch (S.mode) {
    case M_HOMING: modo_homing(); break;
    case M_MOVE:   modo_move(); break;
    case M_HOLD:   modo_hold(); break;
    case M_JOG:    modo_jog(fresh_dm, dir); break;
    case M_PULSE:  modo_pulse(); break;
    default: break;
    }

    /* Un eje que no estuvo bajo control (lazo abierto, libre, desactivado) no puede guardar historia vieja:
     * al retomarlo, el término derivativo del RST vería un salto. */
    for (int j = 0; j < NJ; j++) {
        if (!S.ctrl_vivo[j]) ctrl_reset(j);
        S.ctrl_vivo[j] = false;
    }

    supervisar();
    aplicar_salidas();
    if ((S.ticks % 10) == 0) publicar();
}

static void control_task(void *arg)
{
    (void)arg;
    esp_task_wdt_add(NULL);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / CONTROL_HZ));
        tick();
        esp_task_wdt_reset();
    }
}

/* ---------------------------------------------------------------- API pública */
void ctl_init(void)
{
    cmd_q = xQueueCreate(8, sizeof(cmd_t));
    evt_q = xQueueCreate(16, 160);
    memset(&S, 0, sizeof S);
    memset(&hb, 0, sizeof hb);
    S.mode = M_OFF;
    estop_req = false;
    S.ctrl = CTRL_DEFAULT;
    S.form = RST_FORM_DEFAULT;
    S.tool_en = TOOL_ACTIVO_INIT;
    servo_mover(SERVO_ABAJO_DEG);
    {
        const bool en0[NJ] = MOTORES_INIT;
        for (int j = 0; j < NJ; j++) S.en[j] = en0[j];
    }
    for (int j = 0; j < NJ; j++) {
        rst_cargar(j, S.form);
        rst_reset(&S.rst[j], 0.0f);
        S.pid[j] = (pid_ctl_t){ .kp = JC[j].kp, .ki = JC[j].ki, .kd = JC[j].kd, .kff = JC[j].kff,
                                .i_max = 250.0f, .fric = JC[j].fric, .e_zone = 0.0026f };
        S.off[j] = hal_enc_raw(j);                      /* q = 0 en el arranque (sin referencia) */
        S.qlim_min[j] = JC[j].q_min; S.qlim_max[j] = JC[j].q_max;   /* hasta que el Home los mida */
    }
    publicar();
}

void ctl_start(void)
{
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}

void ctl_cmd_jog(bool dm, int d1, int d2)
{
    portENTER_CRITICAL(&lock);
    hb.dm = dm;
    hb.dir[0] = (int8_t)(d1 > 0) - (int8_t)(d1 < 0);
    hb.dir[1] = (int8_t)(d2 > 0) - (int8_t)(d2 < 0);
    hb.t_us = now_us();
    portEXIT_CRITICAL(&lock);
}

static bool enviar(const cmd_t *c) { return xQueueSend(cmd_q, c, 0) == pdTRUE; }

bool ctl_cmd_home(void)  { cmd_t c = { .t = C_HOME }; return enviar(&c); }
void ctl_cmd_stop(void)  { cmd_t c = { .t = C_STOP }; enviar(&c); }
void ctl_cmd_off(void)   { cmd_t c = { .t = C_OFF }; enviar(&c); }
void ctl_cmd_clear(void) { cmd_t c = { .t = C_CLEAR }; enviar(&c); }

bool ctl_cmd_move(const float q[NJ], const float tgt_xyz[3])
{
    cmd_t c = { .t = C_MOVE };
    memcpy(c.q, q, sizeof c.q);
    memcpy(c.tgt, tgt_xyz, sizeof c.tgt);
    return enviar(&c);
}

bool ctl_cmd_pulse(int j)
{
    if (j < 0 || j >= NJ) return false;
    cmd_t c = { .t = C_PULSE, .j = j };
    return enviar(&c);
}

bool ctl_cmd_gain(int j, float kp, float ki, float kd, float kff, float fric)
{
    if (j < 0 || j >= NJ) return false;
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || !isfinite(kff) || !isfinite(fric)) return false;
    if (kp < 0 || ki < 0 || kd < 0 || kff < 0 || fric < 0 || kp > 50000 || ki > 50000 || kd > 5000 || kff > 500 || fric > 150) return false;
    cmd_t c = { .t = C_GAIN, .j = j, .g = { kp, ki, kd, kff, fric } };
    return enviar(&c);
}

void ctl_get_gain(int j, float out[5])
{
    portENTER_CRITICAL(&lock);
    for (int i = 0; i < 5; i++) out[i] = pub_gain[j][i];
    portEXIT_CRITICAL(&lock);
}

void ctl_cmd_estop(void) { estop_req = true; }

bool ctl_cmd_motor(int m, bool on)
{
    if (m < 0 || m > NJ) return false;
    cmd_t c = { .t = C_MOTOR, .j = m, .g = { on ? 1.0f : 0.0f } };
    return enviar(&c);
}

bool ctl_motor_enabled(int m)
{
    if (m < 0 || m > NJ) return false;
    ctl_status_t s;
    portENTER_CRITICAL(&lock);
    s.en = pub.en;
    portEXIT_CRITICAL(&lock);
    return (s.en >> m) & 1;
}

bool ctl_cmd_tool(float pct)
{
    if (!(pct >= -100.0f && pct <= 100.0f)) return false;
    cmd_t c = { .t = C_TOOL, .g = { pct } };
    return enviar(&c);
}

bool ctl_cmd_servo(float deg)
{
    if (!(deg >= SERVO_GRADOS_MIN && deg <= SERVO_GRADOS_MAX)) return false;
    cmd_t c = { .t = C_SERVO, .g = { deg } };
    return enviar(&c);
}

bool ctl_cmd_screw(float vueltas, float torque)
{
    if (!(vueltas >= 0.0f && vueltas <= 40.0f) || !(torque >= 0.0f && torque <= 1.5f)) return false;
    cmd_t c = { .t = C_SCREW, .g = { vueltas, torque } };
    return enviar(&c);
}

bool ctl_cmd_ctrl(ctrl_t k)
{
    cmd_t c = { .t = C_CTRL, .j = (int)k };
    return enviar(&c);
}

bool ctl_cmd_rstform(rst_form_t f)
{
    if (f > RST_FORM_ORIG) return false;
    cmd_t c = { .t = C_RSTFORM, .j = (int)f };
    return enviar(&c);
}

bool ctl_cmd_rst(int j, const float g[7])
{
    if (j < 0 || j >= NJ) return false;
    for (int i = 0; i < 7; i++) if (!isfinite(g[i])) return false;
    if (g[0] < 0 || g[1] < 0 || g[2] < 0 || g[0] > 100000 || g[1] > 500000 || g[2] > 5000) return false;
    if (fabsf(g[3]) >= 1.0f || g[4] < 0 || g[4] > 1 || g[5] < 0 || g[5] > 1 || g[6] < 0 || g[6] > 500) return false;
    cmd_t c = { .t = C_RST, .j = j };
    memcpy(c.g, g, sizeof c.g);
    return enviar(&c);
}

void ctl_get_rst(int j, float out[7])
{
    portENTER_CRITICAL(&lock);
    for (int i = 0; i < 7; i++) out[i] = pub_rst[j][i];
    portEXIT_CRITICAL(&lock);
}

ctrl_t ctl_get_ctrl(void)
{
    ctl_status_t s;
    portENTER_CRITICAL(&lock);
    s.ctrl = pub.ctrl;
    portEXIT_CRITICAL(&lock);
    return s.ctrl == 0 ? CTRL_PID : CTRL_RST;
}

void ctl_get_limits(int j, float *qmin, float *qmax)
{
    portENTER_CRITICAL(&lock);
    *qmin = S.qlim_min[j]; *qmax = S.qlim_max[j];
    portEXIT_CRITICAL(&lock);
}

void ctl_get_status(ctl_status_t *s)
{
    portENTER_CRITICAL(&lock);
    *s = pub;
    portEXIT_CRITICAL(&lock);
    for (int j = 0; j < NJ; j++) s->enc_levels[j] = (uint8_t)hal_enc_levels(j);
}

bool ctl_event_pop(char *buf, size_t n, uint32_t wait_ms)
{
    char tmp[160];
    if (xQueueReceive(evt_q, tmp, pdMS_TO_TICKS(wait_ms)) != pdTRUE) return false;
    strncpy(buf, tmp, n - 1); buf[n - 1] = 0;
    return true;
}
