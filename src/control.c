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
#define RPM2RADS ((float)M_PI / 30.0f)

typedef enum { C_HOME, C_MOVE, C_STOP, C_OFF, C_CLEAR, C_PULSE, C_GAIN,
               C_ENABLE, C_EST_RESET, C_LUEN, C_KAL, C_VMAP } cmd_type_t;
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

/* latidos del modo manual y de la potencia manual (escritos por la tarea de comunicación, leídos por el lazo) */
static struct { bool dm; int8_t dir[NJ]; int64_t t_us; } hb;
static struct { bool dm; float pct[NM]; int64_t t_us; } hbp;

/* estado visible (copia protegida con lock) */
static ctl_status_t pub;
static float pub_gain[NJ][5];
static obs_snap_t pub_obs[NM];
static uint32_t pub_obs_seq;
static est_params_t pub_est_par[NM];
static float pub_est_L[NM][2];
static bool pub_est_ok[NM];
static float pub_vgain = OBS_VGAIN, pub_v0 = OBS_V0;

/* ---------------------------------------------------------------- estado interno del lazo */
static struct {
    ctl_mode_t mode;
    fault_t fault;
    bool homed[NJ];
    bool en[NM];                     /* motores habilitados */
    int32_t off[NM];                 /* cuentas que corresponden a q = 0 */
    int32_t counts[NM];
    float q[NM], qd[NM];             /* medidos (q[M_TOOL]: ángulo del eje del atornillador) */
    float qref[NJ], qdref[NJ];
    float duty[NM];                  /* PWM actual (con rampa) */
    float u[NM];                     /* PWM deseado de este ciclo */
    bool brake[NM];                  /* true: freno activo en este ciclo */
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
    /* potencia manual */
    float pw[NM];                    /* PWM pedido por potencia manual en este ciclo (0 = sin potencia) */
    bool power_fresh;
    bool pw_latch;                   /* tras stop/off/falla: potencia bloqueada hasta soltar el dead-man */
    bool pstop[NJ];                  /* eje que tuvo potencia y está frenando */
    /* supervisión */
    int follow_ms[NJ];
    int32_t stall_c[NJ]; int64_t stall_t_us[NJ];
    int64_t fault_t_us;
    uint32_t ticks;
    bool qd_init;
    /* observadores */
    est_t est[NM];
    float obs_duty_sum[NM];
    int32_t obs_c_prev[NM];
    int obs_n;
    float vgain, v0;
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
    static const char *n[] = { "OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT", "POWER" };
    return (m <= M_POWER) ? n[m] : "?";
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

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* Home hecho en todas las articulaciones HABILITADAS (y al menos una habilitada). */
static bool enabled_homed(void)
{
    bool alguna = false;
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) continue;
        alguna = true;
        if (!S.homed[j]) return false;
    }
    return alguna;
}

static void bloquear_potencia(void)
{
    if (S.power_fresh && !S.pw_latch) emit("#INFO: potencia manual bloqueada: suelta el dead-man para volver a usarla.");
    S.pw_latch = true;
    for (int m = 0; m < NM; m++) S.pw[m] = 0;
}

static void set_fault(fault_t f)
{
    if (S.mode == M_FAULT) return;
    bloquear_potencia();
    for (int m = 0; m < NM; m++) { hal_motor_brake(m); S.duty[m] = 0; S.u[m] = 0; }
    for (int j = 0; j < NJ; j++) S.homed[j] = false;
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

static void hold_here(int j)
{
    pid_reset(&S.pid[j]);
    S.qref[j] = S.q[j];
    S.qdref[j] = 0.0f;
    S.follow_ms[j] = 0;
    S.stall_c[j] = S.counts[j];
    S.stall_t_us[j] = now_us();
}

static void start_pid_from_here(void)
{
    for (int j = 0; j < NJ; j++) { hold_here(j); S.duty[j] = 0.0f; }
}

/* ---------------------------------------------------------------- sensores */
static float cuentas_a_rad(int m, int32_t c)
{
    if (m < NJ) return JC[m].enc_sign * (float)c / JC[m].k;
    return TOOL.enc_sign * (float)c / TOOL.k;
}

static void leer_sensores(void)
{
    const float a = DT / (VEL_FILTRO_TAU_S + DT);
    for (int m = 0; m < NM; m++) {
        int32_t c = hal_enc_raw(m);
        S.counts[m] = c;
        float qn = cuentas_a_rad(m, c - S.off[m]);
        if (!S.qd_init) { S.qd[m] = 0.0f; }
        else { S.qd[m] += a * ((qn - S.q[m]) / DT - S.qd[m]); }
        S.q[m] = qn;
    }
    for (int j = 0; j < NJ; j++) {
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

/* ---------------------------------------------------------------- PID de las articulaciones habilitadas */
static void pid_todos(void)
{
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) { S.u[j] = 0; continue; }
        S.u[j] = pid_step(&S.pid[j], S.qref[j] - S.q[j], S.qdref[j], S.qd[j], DT, DUTY_MAX);
    }
}

/* ---------------------------------------------------------------- observadores */
static est_params_t est_params_default(void)
{
    est_params_t p = {
        .polo_re = OBS_POLO_RE, .polo_im = OBS_POLO_IM,
        .sig_i = OBS_SIG_I, .sig_w = OBS_SIG_W_RPM * RPM2RADS, .sig_y = OBS_SIG_Y_RPM * RPM2RADS,
        .p0_i = OBS_P0_I, .p0_w = OBS_P0_W_RPM * RPM2RADS,
    };
    return p;
}

static const motor_model_t MODELO = { MOT_RA, MOT_LA, MOT_KV, MOT_B, MOT_J };

static void est_publicar_params(int m)
{
    portENTER_CRITICAL(&lock);
    pub_est_par[m] = S.est[m].par;
    pub_est_L[m][0] = S.est[m].L[0]; pub_est_L[m][1] = S.est[m].L[1];
    pub_est_ok[m] = S.est[m].ok;
    pub_vgain = S.vgain; pub_v0 = S.v0;
    portEXIT_CRITICAL(&lock);
}

static void est_rediseno(int m, const est_params_t *p)
{
    est_design(&S.est[m], &MODELO, p, OBS_TS);
    S.obs_c_prev[m] = S.counts[m];
    S.obs_duty_sum[m] = 0;
    est_publicar_params(m);
}

/* Cada OBS_DIV ticks: velocidad por diferencia de cuentas y voltaje medio del intervalo. */
static void observadores(void)
{
    for (int m = 0; m < NM; m++) S.obs_duty_sum[m] += S.duty[m];
    if (++S.obs_n < OBS_DIV) return;
    S.obs_n = 0;

    obs_snap_t o[NM];
    for (int m = 0; m < NM; m++) {
        est_t *e = &S.est[m];
        bool med = hal_tiene_encoder(m);
        float y = cuentas_a_rad(m, S.counts[m] - S.obs_c_prev[m]) / OBS_TS;
        S.obs_c_prev[m] = S.counts[m];
        float duty_med = S.obs_duty_sum[m] / (OBS_DIV * (float)PWM_MAX);
        S.obs_duty_sum[m] = 0;
        float u = est_voltaje(duty_med, VBUS, S.vgain, S.v0);
        est_step(e, u, y, med);
        o[m] = (obs_snap_t){
            .w_meas = med ? y : NAN, .u = u,
            .wL = e->xl[1], .iL = e->xl[0], .wKp = e->xp[1], .iKp = e->xp[0], .wK = e->xk[1], .iK = e->xk[0],
            .sig_i = sqrtf(fmaxf(e->Pk[0][0], 0)), .sig_w = sqrtf(fmaxf(e->Pk[1][1], 0)),
            .Ki = e->K[0], .Kw = e->K[1], .ok = e->ok, .med = med,
        };
    }
    portENTER_CRITICAL(&lock);
    memcpy(pub_obs, o, sizeof o);
    pub_obs_seq++;
    portEXIT_CRITICAL(&lock);
}

/* ---------------------------------------------------------------- orden de Home (salta los ejes deshabilitados) */
static int home_next(int from)
{
    for (int s = from; s < NJ; s++) if (S.en[HOME_ORDER[s]]) return s;
    return NJ;
}

/* ---------------------------------------------------------------- órdenes */
static void do_cmd(const cmd_t *c)
{
    switch (c->t) {
    case C_HOME:
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: no se puede iniciar Home, el robot esta en modo %s (usa 8 para detener).", ctl_mode_str(S.mode)); break; }
        if (home_next(0) >= NJ) { emit("#ERROR: no hay ejes del brazo habilitados (motor 1 y 2 deshabilitados)."); break; }
        for (int j = 0; j < NJ; j++) { if (S.en[j]) S.homed[j] = false; S.u[j] = 0; }
        for (int j = 0; j < NJ; j++) S.c_start[j] = S.counts[j];
        S.h_step = home_next(0); S.h_stage = 0; S.h_phase = H_SEEK;
        S.h_t_phase_us = S.h_t_start_us = now_us();
        S.h_c_prev = S.counts[HOME_ORDER[S.h_step]]; S.h_t_prev_us = now_us();
        for (int j = 0; j < NJ; j++) S.duty[j] = 0;
        S.mode = M_HOMING;
        hal_stby(true);
        emit("#HOME: iniciando. Eje %d: primero un tope, luego el otro, y vuelve a donde estaba. (cualquier tecla = detener)", HOME_ORDER[S.h_step] + 1);
        break;

    case C_MOVE:
        for (int j = 0; j < NJ; j++)
            if (S.en[j] && !S.homed[j]) { emit("#ERROR: primero ejecuta HOME (opcion 5)."); return; }
        if (!enabled_homed()) { emit("#ERROR: no hay ejes del brazo habilitados."); break; }
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: el robot esta ocupado (modo %s). Detenlo primero (opcion 8).", ctl_mode_str(S.mode)); break; }
        if (S.mode == M_OFF) start_pid_from_here();
        {
            float vmax[NJ], amax[NJ], q0[NJ], q1[NJ];
            for (int j = 0; j < NJ; j++) {
                vmax[j] = JC[j].vmax; amax[j] = JC[j].amax; q0[j] = S.qref[j];
                q1[j] = S.en[j] ? c->q[j] : S.qref[j];          /* un eje deshabilitado no se mueve */
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
        if (S.mode == M_HOMING || S.mode == M_PULSE) {
            for (int j = 0; j < NJ; j++) { hal_motor_free(j); S.duty[j] = 0; S.u[j] = 0; }
            S.mode = M_OFF;
            emit("#INFO: proceso detenido, motores apagados.");
        } else if (S.mode == M_POWER) {
            /* bloquear_potencia() (abajo) deja la potencia en 0: modo_power frena cada eje y pasa a HOLD */
            S.await_reach = false;
            emit("#INFO: movimiento detenido, manteniendo posicion.");
        } else if (S.mode == M_JOG) {
            /* frena con la rampa de JOG (sin flechas) y luego HOLD */
            portENTER_CRITICAL(&lock); hb.dm = false; portEXIT_CRITICAL(&lock);
            for (int j = 0; j < NJ; j++) {
                S.qref[j] = S.q[j];
                S.qdref[j] = S.en[j] ? clampf(S.qd[j], -2.0f * JC[j].jog_v, 2.0f * JC[j].jog_v) : 0.0f;
                pid_reset(&S.pid[j]);
            }
            S.await_reach = false;
            S.mode = M_JOG;
            emit("#INFO: movimiento detenido, manteniendo posicion.");
        } else if (S.mode == M_MOVE) {
            for (int j = 0; j < NJ; j++) S.qref[j] = S.q[j];
            S.await_reach = false;
            enter_hold();
            emit("#INFO: movimiento detenido, manteniendo posicion.");
        }
        bloquear_potencia();
        break;

    case C_OFF:
        bloquear_potencia();
        if (S.mode != M_FAULT) {
            for (int m = 0; m < NM; m++) { hal_motor_free(m); S.duty[m] = 0; S.u[m] = 0; }
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
        if (!S.en[c->j]) { emit("#ERROR: el motor %d esta deshabilitado (motor %d on).", c->j + 1, c->j + 1); break; }
        S.p_j = c->j; S.p_t0_us = now_us(); S.p_c0 = S.counts[c->j]; S.p_q0 = S.q[c->j];
        for (int j = 0; j < NJ; j++) { S.duty[j] = 0; S.u[j] = 0; }
        S.mode = M_PULSE; hal_stby(true);
        break;

    case C_GAIN:
        S.pid[c->j].kp = c->g[0]; S.pid[c->j].ki = c->g[1]; S.pid[c->j].kd = c->g[2];
        S.pid[c->j].kff = c->g[3]; S.pid[c->j].fric = c->g[4];
        pid_reset(&S.pid[c->j]);
        break;

    case C_ENABLE: {
        int m = c->j;
        bool on = c->g[0] > 0.5f;
        if (S.en[m] == on) break;
        S.en[m] = on;
        S.u[m] = 0; S.duty[m] = 0; hal_motor_free(m);
        if (m < NJ) hold_here(m);                 /* al habilitar: mantiene donde está */
        if (!on && S.mode == M_PULSE && S.p_j == m) S.mode = M_OFF;
        emit("#MOTOR: M%d %s.", m + 1, on ? "HABILITADO" : "deshabilitado (libre)");
        break; }

    case C_EST_RESET:
        for (int m = 0; m < NM; m++) est_reset(&S.est[m]);
        emit("#INFO: observadores reiniciados (x = 0, P = P0).");
        break;

    case C_LUEN: {
        est_params_t p = S.est[c->j].par;
        p.polo_re = c->g[0]; p.polo_im = c->g[1];
        est_rediseno(c->j, &p);
        emit("#EST: M%d Luenberger polos %.3f +/- %.3fj -> L = [%.5g, %.5g]%s", c->j + 1, p.polo_re, p.polo_im,
             S.est[c->j].L[0], S.est[c->j].L[1], S.est[c->j].ok ? "" : " (DISENO INVALIDO)");
        break; }

    case C_KAL: {
        est_params_t p = S.est[c->j].par;
        p.sig_i = c->g[0]; p.sig_w = c->g[1]; p.sig_y = c->g[2];
        est_rediseno(c->j, &p);
        emit("#EST: M%d Kalman sig_i=%.4g A, sig_w=%.3g rpm, sig_y=%.3g rpm (R = %.3g rpm^2)", c->j + 1, p.sig_i,
             p.sig_w / RPM2RADS, p.sig_y / RPM2RADS, (p.sig_y / RPM2RADS) * (p.sig_y / RPM2RADS));
        break; }

    case C_VMAP:
        S.vgain = c->g[0]; S.v0 = c->g[1];
        for (int m = 0; m < NM; m++) est_publicar_params(m);
        emit("#EST: voltaje del modelo V = %.4f * (%.0f V * duty) - %.3f V", S.vgain, VBUS, S.v0);
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
 * HOME en tres tiempos, eje por eje (orden HOME_ORDER, solo los ejes habilitados):
 *   etapa 0: va al tope de home_dir, frena, retrocede hasta liberarlo -> ahi q = q_home (referencia).
 *   etapa 1: va al tope contrario, frena, retrocede hasta liberarlo -> mide el recorrido real.
 * Al terminar, los ejes vuelven a la posicion en que estaban al pulsar Home.
 */
static void modo_homing(void)
{
    int64_t t = now_us();
    int j = HOME_ORDER[S.h_step];
    if (!S.en[j]) { S.mode = M_OFF; emit("#ERROR: Home cancelado: el motor %d se deshabilito.", j + 1); return; }
    int hd = JC[j].home_dir;                       /* -1 = MIN, +1 = MAX */
    int side0 = (hd < 0) ? 0 : 1;
    int side = S.h_stage ? 1 - side0 : side0;      /* tope que se busca ahora */
    int dir_seek = S.h_stage ? -hd : hd;           /* sentido de busqueda */

    /* las articulaciones ya referenciadas se mantienen con PID; las demas, libres */
    for (int i = 0; i < NJ; i++) {
        if (i == j) continue;
        if (S.en[i] && S.homed[i]) S.u[i] = pid_step(&S.pid[i], S.qref[i] - S.q[i], 0, S.qd[i], DT, DUTY_MAX);
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
                S.h_step = home_next(S.h_step + 1);
                if (S.h_step >= NJ) {
                    /* terminaron los ejes habilitados: volver a donde estaban */
                    float vmax[NJ], amax[NJ], q0[NJ], q1[NJ];
                    for (int i = 0; i < NJ; i++) {
                        if (S.en[i] && S.homed[i]) {
                            q1[i] = JC[i].enc_sign * (float)(S.c_start[i] - S.off[i]) / JC[i].k;
                            if (q1[i] < S.qlim_min[i]) q1[i] = S.qlim_min[i];
                            if (q1[i] > S.qlim_max[i]) q1[i] = S.qlim_max[i];
                        } else {
                            q1[i] = S.q[i];
                        }
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
        float vt = (fresh_dm && S.en[j]) ? dir[j] * JC[j].jog_v : 0.0f;
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

/* Potencia manual: los ejes con potencia van en lazo abierto; los que tienen 0 % mantienen con su PID.
 * Al soltar un eje (0 %, dead-man suelto o stop) se FRENA con amortiguamiento (u = -kd * velocidad) hasta que
 * casi no se mueve y AHÍ se queda (no vuelve a donde se soltó). Sin ejes con potencia ni frenando -> HOLD. */
static void modo_power(void)
{
    bool alguno = false, frenando = false;
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) { S.u[j] = 0; S.pstop[j] = false; continue; }
        if (fabsf(S.pw[j]) > 0.5f) {
            float u = S.pw[j];
            if (S.homed[j]) {                          /* con Home: no salir de los límites por software */
                if (S.q[j] >= S.qlim_max[j] && u > 0) u = 0;
                if (S.q[j] <= S.qlim_min[j] && u < 0) u = 0;
            }
            S.u[j] = u;
            S.qref[j] = S.q[j]; S.qdref[j] = 0; pid_reset(&S.pid[j]); S.follow_ms[j] = 0;
            S.pstop[j] = true;
            alguno = true;
        } else if (S.pstop[j]) {
            S.qref[j] = S.q[j]; S.qdref[j] = 0; pid_reset(&S.pid[j]); S.follow_ms[j] = 0;
            S.u[j] = clampf(-S.pid[j].kd * S.qd[j], -(float)DUTY_MAX, (float)DUTY_MAX);
            if (fabsf(S.qd[j]) < 0.03f) S.pstop[j] = false;   /* < 1,7 deg/s: se queda aquí */
            else frenando = true;
        } else {
            S.u[j] = pid_step(&S.pid[j], S.qref[j] - S.q[j], 0, S.qd[j], DT, DUTY_MAX);
        }
    }
    if (!alguno && !frenando) enter_hold();
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
        S.mode = M_OFF;
        emit("#PULSO eje %d: cuentas=%ld, dq=%+.2f deg -> %s", j + 1, (long)dc, dq,
             (fabsf(dq) < 0.3f) ? "NO SE MOVIO: revisa 12 V en J15, motor y encoder"
             : (dq > 0 ? "OK: u>0 aumenta q (motor_sign correcto)" : "INVERTIDO: cambia motor_sign de este eje en config.c"));
    }
}

/* ---------------------------------------------------------------- supervisión y salida */
static void supervisar(void)
{
    if (S.mode != M_HOLD && S.mode != M_MOVE && S.mode != M_JOG && S.mode != M_POWER) {
        for (int j = 0; j < NJ; j++) { S.follow_ms[j] = 0; S.stall_c[j] = S.counts[j]; S.stall_t_us[j] = now_us(); }
        return;
    }
    int64_t t = now_us();
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) { S.follow_ms[j] = 0; S.stall_c[j] = S.counts[j]; S.stall_t_us[j] = t; continue; }
        bool potencia = (S.mode == M_POWER && fabsf(S.pw[j]) > 0.5f);

        /* error de seguimiento (en potencia manual la referencia sigue al eje: no aplica) */
        if (!potencia && fabsf(S.qref[j] - S.q[j]) > FOLLOW_ERR_RAD) {
            if (++S.follow_ms[j] * 1000 / CONTROL_HZ > FOLLOW_ERR_MS) { set_fault(F_FOLLOW); return; }
        } else S.follow_ms[j] = 0;

        /* bloqueo: PWM alto y el encoder no cuenta */
        if ((potencia && !POWER_STALL_FALLA) || fabsf(S.duty[j]) < STALL_DUTY) { S.stall_c[j] = S.counts[j]; S.stall_t_us[j] = t; }
        else if (t - S.stall_t_us[j] > STALL_WINDOW_MS * 1000LL) {
            if (abs(S.counts[j] - S.stall_c[j]) < STALL_MIN_COUNTS) { set_fault(F_STALL); return; }
            S.stall_c[j] = S.counts[j]; S.stall_t_us[j] = t;
        }

        /* en MOVE, tocar un final de carrera significa que se perdió la referencia */
        if (S.mode == M_MOVE && (S.fc[j][0] || S.fc[j][1])) { set_fault(F_LIMIT); return; }
    }
}

static void rampa_y_salida(int m, float u)
{
    float d = u - S.duty[m];
    if (d >  DUTY_SLEW_PER_MS) d =  DUTY_SLEW_PER_MS;
    if (d < -DUTY_SLEW_PER_MS) d = -DUTY_SLEW_PER_MS;
    S.duty[m] += d;
    hal_motor_drive(m, (int)lroundf(S.duty[m]));
}

static void aplicar_salidas(void)
{
    if (S.mode == M_FAULT) {
        if (now_us() - S.fault_t_us < FALLA_FRENO_MS * 1000LL) return;   /* sigue frenando */
        hal_all_free(); hal_stby(false);
        for (int m = 0; m < NM; m++) S.duty[m] = 0;
        return;
    }

    /* motor 3 (atornillador): independiente del modo del brazo */
    float ut = S.en[M_TOOL] ? S.u[M_TOOL] : 0.0f;
    bool herramienta = S.en[M_TOOL] && (fabsf(ut) > 0.5f || fabsf(S.duty[M_TOOL]) > 0.5f);

    if (S.mode == M_OFF) {
        for (int j = 0; j < NJ; j++) { hal_motor_free(j); S.duty[j] = 0; }
        if (herramienta) { hal_stby(true); rampa_y_salida(M_TOOL, ut); }
        else { hal_motor_free(M_TOOL); S.duty[M_TOOL] = 0; hal_stby(false); }
        return;
    }
    hal_stby(true);
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) { hal_motor_free(j); S.duty[j] = 0; continue; }
        if (S.brake[j]) { hal_motor_brake(j); S.duty[j] = 0; continue; }
        float u = S.u[j];
        /* prohibido empujar contra un final de carrera presionado: solo se puede alejarse */
        if (S.fc[j][0] && u < 0) u = 0;
        if (S.fc[j][1] && u > 0) u = 0;
        rampa_y_salida(j, u);
    }
    if (herramienta) rampa_y_salida(M_TOOL, ut);
    else { hal_motor_free(M_TOOL); S.duty[M_TOOL] = 0; }
}

static void publicar(void)
{
    portENTER_CRITICAL(&lock);
    pub.mode = S.mode; pub.fault = S.fault;
    pub.fc = 0; pub.enabled = 0;
    for (int j = 0; j < NJ; j++) {
        pub.homed[j] = S.homed[j]; pub.q[j] = S.q[j]; pub.qref[j] = S.qref[j];
        pub.fc |= (S.fc[j][0] ? 1 : 0) << (2 * j);
        pub.fc |= (S.fc[j][1] ? 1 : 0) << (2 * j + 1);
        pub_gain[j][0] = S.pid[j].kp; pub_gain[j][1] = S.pid[j].ki; pub_gain[j][2] = S.pid[j].kd;
        pub_gain[j][3] = S.pid[j].kff; pub_gain[j][4] = S.pid[j].fric;
    }
    for (int m = 0; m < NM; m++) {
        pub.duty[m] = S.duty[m]; pub.counts[m] = S.counts[m]; pub.qd[m] = S.qd[m];
        if (S.en[m]) pub.enabled |= (uint8_t)(1u << m);
    }
    pub.power_on = S.power_fresh;
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

    /* latidos del modo manual (flechas) y de la potencia manual */
    bool fresh_dm, fresh_pw; int8_t dir[NJ]; float pct[NM];
    portENTER_CRITICAL(&lock);
    int64_t tn = now_us();
    fresh_dm = hb.dm && (tn - hb.t_us) < DEADMAN_TIMEOUT_MS * 1000LL;
    fresh_pw = hbp.dm && (tn - hbp.t_us) < DEADMAN_TIMEOUT_MS * 1000LL;
    for (int j = 0; j < NJ; j++) dir[j] = hb.dir[j];
    for (int m = 0; m < NM; m++) pct[m] = hbp.pct[m];
    portEXIT_CRITICAL(&lock);
    bool quiere_jog = fresh_dm && ((S.en[0] && dir[0] != 0) || (S.en[1] && dir[1] != 0));

    S.power_fresh = fresh_pw;
    if (!fresh_pw) S.pw_latch = false;
    bool quiere_power = false;
    for (int m = 0; m < NM; m++) {
        float d = (fresh_pw && !S.pw_latch && S.en[m]) ? clampf(pct[m], -100.0f, 100.0f) * (PWM_MAX / 100.0f) : 0.0f;
        S.pw[m] = clampf(d, -(float)DUTY_MAX, (float)DUTY_MAX);
        if (m < NJ && fabsf(S.pw[m]) > 0.5f) quiere_power = true;
    }

    if ((S.mode == M_OFF || S.mode == M_HOLD) && quiere_jog) {
        if (S.mode == M_OFF) start_pid_from_here();
        S.await_reach = false;
        S.mode = M_JOG;
    }
    if ((S.mode == M_OFF || S.mode == M_HOLD || S.mode == M_JOG) && quiere_power) {
        if (S.mode == M_OFF) start_pid_from_here();
        if (S.mode == M_JOG) for (int j = 0; j < NJ; j++) { S.qref[j] = S.q[j]; S.qdref[j] = 0; }
        for (int j = 0; j < NJ; j++) S.pstop[j] = false;
        S.await_reach = false;
        S.mode = M_POWER;
    }

    for (int m = 0; m < NM; m++) { S.u[m] = 0; S.brake[m] = false; }
    switch (S.mode) {
    case M_HOMING: modo_homing(); break;
    case M_MOVE:   modo_move(); break;
    case M_HOLD:   modo_hold(); break;
    case M_JOG:    modo_jog(fresh_dm, dir); break;
    case M_PULSE:  modo_pulse(); break;
    case M_POWER:  modo_power(); break;
    default: break;
    }
    /* el atornillador solo se mueve con potencia manual, en cualquier modo salvo FAULT/HOMING/PULSE */
    if (S.mode != M_FAULT && S.mode != M_HOMING && S.mode != M_PULSE) S.u[M_TOOL] = S.pw[M_TOOL];

    supervisar();
    aplicar_salidas();
    observadores();
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
    memset(&hbp, 0, sizeof hbp);
    S.mode = M_OFF;
    S.vgain = OBS_VGAIN; S.v0 = OBS_V0;
    for (int j = 0; j < NJ; j++) {
        S.pid[j] = (pid_ctl_t){ .kp = JC[j].kp, .ki = JC[j].ki, .kd = JC[j].kd, .kff = JC[j].kff,
                                .i_max = 250.0f, .fric = JC[j].fric, .e_zone = 0.0026f };
        S.qlim_min[j] = JC[j].q_min; S.qlim_max[j] = JC[j].q_max;   /* hasta que el Home los mida */
    }
    est_params_t p = est_params_default();
    for (int m = 0; m < NM; m++) {
        S.off[m] = hal_enc_raw(m);                      /* q = 0 en el arranque (sin referencia) */
        S.counts[m] = S.off[m];
        S.en[m] = (MOTORES_HABILITADOS_INIT >> m) & 1;
        est_rediseno(m, &p);
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

void ctl_cmd_power(bool dm, const float pct[NM])
{
    portENTER_CRITICAL(&lock);
    hbp.dm = dm;
    for (int m = 0; m < NM; m++) hbp.pct[m] = (pct && isfinite(pct[m])) ? pct[m] : 0.0f;
    hbp.t_us = now_us();
    portEXIT_CRITICAL(&lock);
}

static bool enviar(const cmd_t *c) { return xQueueSend(cmd_q, c, 0) == pdTRUE; }

bool ctl_cmd_home(void)  { cmd_t c = { .t = C_HOME }; return enviar(&c); }
void ctl_cmd_stop(void)  { cmd_t c = { .t = C_STOP }; enviar(&c); }
void ctl_cmd_off(void)   { cmd_t c = { .t = C_OFF }; enviar(&c); }
void ctl_cmd_clear(void) { cmd_t c = { .t = C_CLEAR }; enviar(&c); }
void ctl_cmd_est_reset(void) { cmd_t c = { .t = C_EST_RESET }; enviar(&c); }

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

bool ctl_cmd_enable(int m, bool on)
{
    if (m < 0 || m >= NM) return false;
    cmd_t c = { .t = C_ENABLE, .j = m, .g = { on ? 1.0f : 0.0f } };
    return enviar(&c);
}

uint8_t ctl_get_enabled(void)
{
    portENTER_CRITICAL(&lock);
    uint8_t e = pub.enabled;
    portEXIT_CRITICAL(&lock);
    return e;
}

bool ctl_cmd_luen(int m, float re, float im)
{
    if (m < 0 || m >= NM || !isfinite(re) || !isfinite(im)) return false;
    if (sqrtf(re * re + im * im) >= 0.999f) return false;          /* el error no convergería */
    cmd_t c = { .t = C_LUEN, .j = m, .g = { re, fabsf(im) } };
    return enviar(&c);
}

bool ctl_cmd_kal(int m, float sig_i, float sig_w, float sig_y)
{
    if (m < 0 || m >= NM) return false;
    if (!(sig_i > 0 && sig_w > 0 && sig_y > 0) || sig_i > 10 || sig_w > 100 || sig_y > 100) return false;
    cmd_t c = { .t = C_KAL, .j = m, .g = { sig_i, sig_w, sig_y } };
    return enviar(&c);
}

bool ctl_cmd_vmap(float vgain, float v0)
{
    if (!(vgain > 0.3f && vgain < 1.5f) || !(v0 >= 0.0f && v0 < 6.0f)) return false;
    cmd_t c = { .t = C_VMAP, .g = { vgain, v0 } };
    return enviar(&c);
}

void ctl_get_est(int m, est_params_t *p, float L[2], bool *ok)
{
    portENTER_CRITICAL(&lock);
    if (p) *p = pub_est_par[m];
    if (L) { L[0] = pub_est_L[m][0]; L[1] = pub_est_L[m][1]; }
    if (ok) *ok = pub_est_ok[m];
    portEXIT_CRITICAL(&lock);
}

void ctl_get_vmap(float *vgain, float *v0)
{
    portENTER_CRITICAL(&lock);
    *vgain = pub_vgain; *v0 = pub_v0;
    portEXIT_CRITICAL(&lock);
}

void ctl_get_obs(obs_snap_t out[NM], uint32_t *seq)
{
    portENTER_CRITICAL(&lock);
    memcpy(out, pub_obs, sizeof pub_obs);
    if (seq) *seq = pub_obs_seq;
    portEXIT_CRITICAL(&lock);
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
    for (int m = 0; m < NM; m++) s->enc_levels[m] = (uint8_t)hal_enc_levels(m);
}

bool ctl_event_pop(char *buf, size_t n, uint32_t wait_ms)
{
    char tmp[160];
    if (xQueueReceive(evt_q, tmp, pdMS_TO_TICKS(wait_ms)) != pdTRUE) return false;
    strncpy(buf, tmp, n - 1); buf[n - 1] = 0;
    return true;
}
