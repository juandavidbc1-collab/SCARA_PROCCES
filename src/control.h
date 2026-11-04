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
 *  - MOVE/goto exigen que las articulaciones activas estén referenciadas (Home hecho).
 *  - Paro de emergencia: no pasa por la cola de órdenes. Freno, motores libres, STBY = 0, enclavado en
 *    FAULT (F_ESTOP) hasta "clear". Conserva el Home salvo que interrumpa un Home en curso.
 *  - Motor desactivado: driver libre, encoder contando, sin supervisión. Home solo con los activos.
 *  - Atornillado (M3): el brazo se queda quieto. Paro, falla, detener, apagar o desactivar M3 lo cancelan. Paro y falla bajan el destornillador (servo a SERVO_ABAJO_DEG).
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "config.h"

typedef enum { M_OFF = 0, M_HOMING, M_HOLD, M_MOVE, M_JOG, M_PULSE, M_FAULT } ctl_mode_t;

typedef enum {
    F_NONE = 0, F_FOLLOW, F_STALL, F_LIMIT, F_HOME_TIMEOUT, F_DIAG_FC, F_ESTOP,
} fault_t;

typedef enum { CTRL_PID = 0, CTRL_RST } ctrl_t;
/* Cómo entra la consigna en el RST: pesos (b, c) de P y D. Ver src/rst.h. */
typedef enum { RST_FORM_POS = 0, RST_FORM_INC, RST_FORM_MASK, RST_FORM_ORIG, RST_FORM_CUSTOM } rst_form_t;
extern const char *const RST_FORM_NAME[];

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
    uint8_t    en;            /* bit 0 = J1, bit 1 = J2, bit 2 = M3 activos */
    uint8_t    ctrl;          /* 0 = PID; 1 + rst_form_t si es RST */
    float      tool_duty;     /* PWM de M3 */
    /* Atornillado. La corriente y lo que sale de ella vienen del modelo: no hay sensor de corriente. */
    float      tool_i;        /* [A] */
    float      tool_torque;   /* [N·m] */
    float      tool_fuerza;   /* [N] */
    float      tool_presion;  /* [MPa] */
    float      tool_vueltas;  /* del tornillo actual */
    float      tool_prof;     /* [mm] */
    float      servo_deg;
    uint8_t    screw_fase;    /* screw_phase_t */
    bool       m3_ref;        /* M3 ya encontró el tope de arriba */
    bool       m3_homing;
    bool       m3_moviendo;   /* buscando el tope o yendo a una altura */
    float      m3_vueltas;    /* desde el tope de arriba, crece al bajar; 0 si no hay referencia */
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

void ctl_cmd_estop(void);     /* paro de emergencia: siempre se atiende en el siguiente tick */
bool ctl_cmd_motor(int m, bool on);   /* m: 0 = J1, 1 = J2, 2 = M3 */
bool ctl_motor_enabled(int m);
bool ctl_cmd_tool(float pct);         /* M3 en lazo abierto: -100..100 %. Hay que renovarla antes de TOOL_MAX_ON_MS */
bool ctl_cmd_servo(float deg);        /* SERVO_GRADOS_MIN a SERVO_GRADOS_MAX */
bool ctl_cmd_m3_ir(float vueltas);    /* a esas vueltas desde el tope de arriba. Al llegar emite "#M3: ..." */
bool ctl_cmd_m3_home(void);           /* sube M3 hasta el tope y pone ahí el cero */
bool ctl_cmd_screw(float vueltas, float torque);   /* 0 = valor por defecto. Al terminar emite "#TORNILLO: ..." */
bool ctl_cmd_ctrl(ctrl_t c);          /* solo con el robot en OFF o HOLD */
bool ctl_cmd_rstform(rst_form_t f);   /* solo con el robot en OFF o HOLD; ambos ejes */
bool ctl_cmd_rst(int j, const float g[7]);   /* kp ki kd s1 b c kff */
void ctl_get_rst(int j, float out[7]);
ctrl_t ctl_get_ctrl(void);
/* Limites por software vigentes [rad]: los de config.c hasta que el Home mide el recorrido real. */
void ctl_get_limits(int j, float *qmin, float *qmax);

void ctl_get_status(ctl_status_t *s);
const char *ctl_mode_str(ctl_mode_t m);
const char *ctl_fault_str(fault_t f);

/* Cola de mensajes del lazo de control hacia el puerto serie. true si hay mensaje. */
bool ctl_event_pop(char *buf, size_t n, uint32_t wait_ms);
