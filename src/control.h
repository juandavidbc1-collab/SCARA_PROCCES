/*
 * NÚCLEO DE CONTROL: lazo de 1 kHz (tarea FreeRTOS de alta prioridad en el núcleo 1).
 *
 * Máquina de estados del brazo (J1, J2):
 *
 *   OFF ──home──▶ HOMING ──(ok)──▶ MOVE(vuelve a donde estaba) ──▶ HOLD
 *    ▲  ▲                                      ▲                │
 *    │  └─ off/stop ───────────────────────────┼── goto/ang ─▶ MOVE ──(llegó)──▶ HOLD
 *    │                                         │
 *    └──── clear ──── FAULT ◀── cualquier falla    OFF/HOLD ◀──▶ JOG   (flechas + dead-man)
 *                                                  OFF/HOLD ◀──▶ POWER (potencia manual + dead-man)
 *
 *  - Toda falla: freno 150 ms, motores libres, STBY = 0 y modo FAULT (hasta "clear").
 *  - JOG y POWER solo se mantienen mientras llegan latidos con dead-man = 1 (DEADMAN_TIMEOUT_MS).
 *  - MOVE/goto exigen que las articulaciones habilitadas estén referenciadas (Home hecho).
 *  - Motores habilitados/deshabilitados en vivo (máscara de 3 bits). Un motor deshabilitado queda libre.
 *  - El motor 3 (atornillador) solo se mueve con potencia manual; funciona en paralelo al brazo.
 *  - Observadores (Luenberger + Kalman) de los 3 motores a 100 Hz: estiman velocidad y corriente.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "config.h"
#include "estimator.h"

typedef enum { M_OFF = 0, M_HOMING, M_HOLD, M_MOVE, M_JOG, M_PULSE, M_FAULT, M_POWER } ctl_mode_t;

typedef enum {
    F_NONE = 0, F_FOLLOW, F_STALL, F_LIMIT, F_HOME_TIMEOUT, F_DIAG_FC,
} fault_t;

typedef struct {
    ctl_mode_t mode;
    fault_t    fault;
    bool       homed[NJ];
    float      q[NJ];         /* [rad] medido */
    float      qref[NJ];      /* [rad] referencia */
    float      duty[NM];      /* PWM aplicado (con signo, convención de la articulación) */
    float      qd[NM];        /* [rad/s] velocidad medida del eje de salida (filtrada, 1 kHz) */
    int32_t    counts[NM];
    uint8_t    fc;            /* bit (2*j + side): final de carrera presionado (con antirrebote) */
    uint8_t    enc_levels[NM];
    uint8_t    enabled;       /* bit m: motor m+1 habilitado */
    bool       power_on;      /* llegan latidos de potencia manual con dead-man */
} ctl_status_t;

/* Lo que estiman los observadores de un motor (unidades SI, eje de salida). */
typedef struct {
    float w_meas;             /* velocidad medida: diferencia de cuentas en 10 ms [rad/s] */
    float u;                  /* voltaje que recibe el motor según el modelo [V] */
    float wL, iL;             /* Luenberger: velocidad [rad/s], corriente [A] */
    float wKp, iKp;           /* Kalman, predicción a priori */
    float wK, iK;             /* Kalman, corrección a posteriori */
    float sig_i, sig_w;       /* sqrt de la diagonal de P+  [A], [rad/s] */
    float Ki, Kw;             /* ganancia de Kalman (Kw: adimensional, 0..1) */
    bool  ok, med;            /* diseño válido / tiene medida (encoder) */
} obs_snap_t;

void ctl_init(void);          /* crea colas; no mueve nada */
void ctl_start(void);         /* lanza la tarea del lazo de control */

/* Latido de modo manual: dm = dead-man (espacio presionado); dir = -1/0/+1 por articulación. */
void ctl_cmd_jog(bool dm, int d1, int d2);
/* Latido de potencia manual: dm = dead-man; pct = % de PWM con signo por motor (se limita a DUTY_MAX). */
void ctl_cmd_power(bool dm, const float pct[NM]);

/* Órdenes: devuelven false si se rechazan (el motivo sale como evento "#ERROR ..."). */
bool ctl_cmd_home(void);
bool ctl_cmd_move(const float q[NJ], const float tgt_xyz[3]);   /* tgt_xyz = NAN si no aplica */
void ctl_cmd_stop(void);      /* detiene el movimiento y mantiene posición (HOLD) */
void ctl_cmd_off(void);       /* apaga motores (STBY = 0) */
void ctl_cmd_clear(void);     /* borra una falla */
bool ctl_cmd_pulse(int j);
bool ctl_cmd_gain(int j, float kp, float ki, float kd, float kff, float fric);
void ctl_get_gain(int j, float out[5]);
/* Limites por software vigentes [rad]: los de config.c hasta que el Home mide el recorrido real. */
void ctl_get_limits(int j, float *qmin, float *qmax);

/* Motores: m = 0..NM-1 */
bool    ctl_cmd_enable(int m, bool on);
uint8_t ctl_get_enabled(void);

/* Observadores */
void ctl_cmd_est_reset(void);
bool ctl_cmd_luen(int m, float re, float im);                        /* polos re +/- j im, |polo| < 1 */
bool ctl_cmd_kal(int m, float sig_i, float sig_w, float sig_y);      /* [A], [rad/s], [rad/s], > 0 */
bool ctl_cmd_vmap(float vgain, float v0);
void ctl_get_est(int m, est_params_t *p, float L[2], bool *ok);
void ctl_get_vmap(float *vgain, float *v0);
void ctl_get_obs(obs_snap_t out[NM], uint32_t *seq);

void ctl_get_status(ctl_status_t *s);
const char *ctl_mode_str(ctl_mode_t m);
const char *ctl_fault_str(fault_t f);

/* Cola de mensajes del lazo de control hacia el puerto serie. true si hay mensaje. */
bool ctl_event_pop(char *buf, size_t n, uint32_t wait_ms);
