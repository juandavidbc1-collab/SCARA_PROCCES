/*
 * NÚCLEO DE CONTROL: lazo de 1 kHz (tarea FreeRTOS de alta prioridad en el núcleo 1).
 *
 * Máquina de estados:
 *
 *   OFF ──home──▶ HOMING ──(ok)──▶ MOVE(vuelve a donde estaba) ──▶ HOLD
 *    ▲  ▲                                      ▲                │
 *    │  └─ off/stop ───────────────────────────┼── goto/ang ─▶ MOVE ──(llegó)──▶ HOLD
 *    │                                         │
 *    └──── clear ──── FAULT ◀── cualquier falla    OFF/HOLD ◀──▶ JOG (flechas + espacio)
 *
 *  - Toda falla: freno 150 ms, motores libres, STBY = 0 y modo FAULT (hasta "clear").
 *  - JOG solo se mantiene mientras llegan latidos con dead-man = 1 (ver DEADMAN_TIMEOUT_MS).
 *  - MOVE/goto exigen que las dos articulaciones estén referenciadas (Home hecho).
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "config.h"

typedef enum { M_OFF = 0, M_HOMING, M_HOLD, M_MOVE, M_JOG, M_PULSE, M_FAULT } ctl_mode_t;

typedef enum {
    F_NONE = 0, F_FOLLOW, F_STALL, F_LIMIT, F_HOME_TIMEOUT, F_DIAG_FC,
} fault_t;

typedef struct {
    ctl_mode_t mode;
    fault_t    fault;
    bool       homed[NJ];
    float      q[NJ];         /* [rad] medido */
    float      qref[NJ];      /* [rad] referencia */
    float      duty[NJ];      /* PWM aplicado */
    int32_t    counts[NJ];
    uint8_t    fc;            /* bit (2*j + side): final de carrera presionado (con antirrebote) */
    uint8_t    enc_levels[NJ];
} ctl_status_t;

void ctl_init(void);          /* crea colas; no mueve nada */
void ctl_start(void);         /* lanza la tarea del lazo de control */

/* Latido de modo manual: dm = dead-man (espacio presionado); dir = -1/0/+1 por articulación. */
void ctl_cmd_jog(bool dm, int d1, int d2);

/* Órdenes: devuelven false si se rechazan (el motivo sale como evento "#err ..."). */
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

void ctl_get_status(ctl_status_t *s);
const char *ctl_mode_str(ctl_mode_t m);
const char *ctl_fault_str(fault_t f);

/* Cola de mensajes del lazo de control hacia el puerto serie. true si hay mensaje. */
bool ctl_event_pop(char *buf, size_t n, uint32_t wait_ms);
