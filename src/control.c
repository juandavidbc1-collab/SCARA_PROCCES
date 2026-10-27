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
#include "traj.h"

#define DT (1.0f / CONTROL_HZ)

typedef enum { C_HOME, C_MOVE, C_STOP, C_OFF, C_CLEAR, C_PULSE, C_GAIN } cmd_type_t;
typedef struct {
    cmd_type_t t;
    float q[NJ];
    float tgt[3];
    int   j;
    float g[5];
} cmd_t;

typedef enum { H_SEEK = 0, H_BRAKE, H_BACKOFF, H_SETTLE } home_phase_t;

static QueueHandle_t cmd_q, evt_q;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

/* latido del modo manual (escrito por la tarea serie, leído por el lazo) */
static struct { bool dm; int8_t dir[NJ]; int64_t t_us; } hb;

/* estado visible (copia protegida con lock) */
static ctl_status_t pub;
static float pub_gain[NJ][5];

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
    }
    return "?";
}

static bool all_homed(void)
{
    for (int j = 0; j < NJ; j++) if (!S.homed[j]) return false;
    return true;
}

static void set_fault(fault_t f)
{
    if (S.mode == M_FAULT) return;
    for (int j = 0; j < NJ; j++) { hal_motor_brake(j); S.homed[j] = false; S.duty[j] = 0; S.u[j] = 0; }
    S.mode = M_FAULT;
    S.fault = f;
    S.fault_t_us = now_us();
    S.await_reach = false;
    emit("#FALLA: %s. Motores apagados. Escribe 0 (o clear) para borrarla.", ctl_fault_str(f));
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
        pid_reset(&S.pid[j]);
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
}

/* ---------------------------------------------------------------- PID de todas las articulaciones */
static void pid_todos(void)
{
    for (int j = 0; j < NJ; j++) {
        S.u[j] = pid_step(&S.pid[j], S.qref[j] - S.q[j], S.qdref[j], S.qd[j], DT, DUTY_MAX);
    }
}

/* ---------------------------------------------------------------- órdenes */
static void do_cmd(const cmd_t *c)
{
    switch (c->t) {
    case C_HOME:
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: no se puede iniciar Home, el robot esta en modo %s (usa 8 para detener).", ctl_mode_str(S.mode)); break; }
        for (int j = 0; j < NJ; j++) { S.homed[j] = false; S.u[j] = 0; }
        for (int j = 0; j < NJ; j++) S.c_start[j] = S.counts[j];
        S.h_step = 0; S.h_stage = 0; S.h_phase = H_SEEK;
        S.h_t_phase_us = S.h_t_start_us = now_us();
        S.h_c_prev = S.counts[HOME_ORDER[0]]; S.h_t_prev_us = now_us();
        for (int j = 0; j < NJ; j++) S.duty[j] = 0;
        S.mode = M_HOMING;
        hal_stby(true);
        emit("#HOME: iniciando. Eje %d: primero un tope, luego el otro, y vuelve a donde estaba. (cualquier tecla = detener)", HOME_ORDER[0] + 1);
        break;

    case C_MOVE:
        if (!all_homed()) { emit("#ERROR: primero ejecuta HOME (opcion 5)."); break; }
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: el robot esta ocupado (modo %s). Detenlo primero (opcion 8).", ctl_mode_str(S.mode)); break; }
        if (S.mode == M_OFF) start_pid_from_here();
        {
            float vmax[NJ], amax[NJ];
            for (int j = 0; j < NJ; j++) { vmax[j] = JC[j].vmax; amax[j] = JC[j].amax; }
            float q0[NJ]; for (int j = 0; j < NJ; j++) q0[j] = S.qref[j];
            bool mueve = traj_plan(&S.traj, q0, c->q, vmax, amax);
            memcpy(S.tgt, c->tgt, sizeof S.tgt);
            S.t_traj_us = now_us();
            S.mode = M_MOVE;
            S.await_reach = true; S.t_inband_us = 0;
            S.t_reach_us = now_us() + (int64_t)(S.traj.T * 1e6f) + REACH_TIMEOUT_MS * 1000LL;
            hal_stby(true);
            if (!mueve) { for (int j = 0; j < NJ; j++) S.qref[j] = c->q[j]; }
        }
        break;

    case C_STOP:
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
            S.mode = M_OFF; S.await_reach = false; hal_stby(false);
            emit("#INFO: motores apagados (STBY = 0).");
        }
        break;

    case C_CLEAR:
        if (S.mode == M_FAULT) {
            S.mode = M_OFF; S.fault = F_NONE;
            emit("#INFO: falla borrada. Los ejes quedaron SIN referencia: ejecuta HOME de nuevo.");
        }
        break;

    case C_PULSE:
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: no se puede hacer el pulso, el robot esta ocupado."); break; }
        S.p_j = c->j; S.p_t0_us = now_us(); S.p_c0 = S.counts[c->j]; S.p_q0 = S.q[c->j];
        for (int j = 0; j < NJ; j++) { S.duty[j] = 0; S.u[j] = 0; }
        S.mode = M_PULSE; hal_stby(true);
        break;

    case C_GAIN:
        S.pid[c->j].kp = c->g[0]; S.pid[c->j].ki = c->g[1]; S.pid[c->j].kd = c->g[2];
        S.pid[c->j].kff = c->g[3]; S.pid[c->j].fric = c->g[4];
        pid_reset(&S.pid[c->j]);
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
        if (S.homed[i]) S.u[i] = pid_step(&S.pid[i], S.qref[i] - S.q[i], 0, S.qd[i], DT, DUTY_MAX);
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
            S.qref[j] = S.q[j]; S.qdref[j] = 0; pid_reset(&S.pid[j]);
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
                if (++S.h_step >= NJ) {
                    /* terminaron los dos ejes: volver a donde estaban */
                    float vmax[NJ], amax[NJ], q0[NJ], q1[NJ];
                    for (int i = 0; i < NJ; i++) {
                        q1[i] = JC[i].enc_sign * (float)(S.c_start[i] - S.off[i]) / JC[i].k;
                        if (q1[i] < S.qlim_min[i]) q1[i] = S.qlim_min[i];
                        if (q1[i] > S.qlim_max[i]) q1[i] = S.qlim_max[i];
                        vmax[i] = JC[i].vmax * 0.5f; amax[i] = JC[i].amax * 0.5f;
                        q0[i] = S.q[i]; S.qref[i] = S.q[i]; pid_reset(&S.pid[i]);
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
    for (int j = 0; j < NJ; j++) if (fabsf(S.qref[j] - S.q[j]) > HOLD_TOL_RAD) dentro = false;
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

static void aplicar_salidas(void)
{
    if (S.mode == M_OFF) {
        hal_all_free(); hal_stby(false);
        for (int j = 0; j < NJ; j++) S.duty[j] = 0;
        return;
    }
    if (S.mode == M_FAULT) {
        if (now_us() - S.fault_t_us < FALLA_FRENO_MS * 1000LL) return;   /* sigue frenando */
        hal_all_free(); hal_stby(false);
        return;
    }
    hal_stby(true);
    for (int j = 0; j < NJ; j++) {
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
    for (int j = 0; j < NJ; j++) {
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
    bool quiere_jog = fresh_dm && (dir[0] != 0 || dir[1] != 0);

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
    for (int j = 0; j < NJ; j++) {
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
