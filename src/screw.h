/*
 * MODELO DEL MOTOR 3 Y MÁQUINA DE ATORNILLADO. C puro, sin hardware: se prueba en el PC.
 *
 * Ya no hay sensor de corriente (el ACS712 cedió J13 al servo). La corriente sale del modelo y de la velocidad que mide
 * el encoder de M3, todo referido al eje de salida del reductor:
 *
 *     i      = (V - Ke w) / R                 V = duty * Vbus / PWM_MAX
 *     torque = Kt (i - I0)                    torque que llega al tornillo
 *     fuerza = (torque - torque_rosca) / (K d)     relación torque-tensión de un tornillo
 *     presión= fuerza / area bajo la cabeza
 *     profundidad = vueltas * paso
 *
 * Fases de un tornillo: rápido -> lento (últimas vueltas) -> apriete (PWM cuyo torque de calado es el pedido) -> fin.
 */
#pragma once
#include <stdbool.h>

typedef struct {
    float r, ke, kt, i0, vbus, pwm_max;   /* motor: ohm, V/(rad/s), N·m/A, A (eje de salida) */
    float paso_mm, k_tuerca, d_mm, area_mm2, torque_rosca;
} screw_model_t;

typedef struct {
    float i, torque, fuerza, presion;     /* A, N·m, N, MPa */
} screw_est_t;

typedef enum { SCREW_IDLE = 0, SCREW_RAPIDO, SCREW_LENTO, SCREW_APRIETE, SCREW_FIN } screw_phase_t;

typedef enum {
    SCREW_R_NONE = 0,
    SCREW_R_OK,           /* llegó al torque con las vueltas esperadas */
    SCREW_R_SIN_APRIETE,  /* pasó de las vueltas sin torque: no hay tornillo, o está barrido */
    SCREW_R_TRABADO,      /* torque antes de tiempo: rosca cruzada, atasco o encoder sin señal */
    SCREW_R_FLOJO,        /* se detuvo antes de llegar al torque */
    SCREW_R_TIEMPO,       /* se agotó el tiempo */
    SCREW_R_CORRIENTE,    /* sobrecorriente estimada */
    SCREW_R_CANCELADO,    /* paro, falla, motor desactivado u orden de detener */
} screw_result_t;

typedef struct {
    float vueltas_meta, torque_meta;      /* vueltas hasta asentar [vueltas], torque final [N·m] */
    float vueltas_lento;                  /* cuántas vueltas antes de la meta pasa a lento */
    float vueltas_min, vueltas_max;       /* fuera de este rango, el apriete no cuenta como bueno */
    float duty_rapido, duty_lento;        /* PWM */
    float torque_asiento;                 /* torque que indica que la cabeza ya asentó [N·m] */
    float i_max;                          /* A */
    float t_arranque_s, t_confirma_s, t_max_s;
    float w_quieto;                       /* rad/s: por debajo, el motor está calado */
} screw_cfg_t;

typedef struct {
    screw_cfg_t cfg;
    screw_phase_t fase;
    screw_result_t res;
    float t, t_fase, t_torque, t_quieto, t_corriente;
    float vueltas;
    float torque_max, i_final, torque_final, fuerza_final, presion_final;
    float duty;
    float duty_apriete;
} screw_t;

/* Modelo */
float       screw_corriente(const screw_model_t *m, float duty, float w);
screw_est_t screw_estimar(const screw_model_t *m, float duty, float w);
float       screw_profundidad_mm(const screw_model_t *m, float vueltas);
float       screw_duty_para_torque(const screw_model_t *m, float torque);   /* PWM cuyo torque de calado es ese */

/* Máquina de estados: screw_step se llama con cada muestra; devuelve el PWM a aplicar, con signo positivo = apretar. */
void  screw_start(screw_t *s, const screw_model_t *m, const screw_cfg_t *cfg);
float screw_step(screw_t *s, const screw_model_t *m, float dt, float duty_aplicado, float w, float dvueltas);
void  screw_cancel(screw_t *s);
bool  screw_activo(const screw_t *s);
const char *screw_result_str(screw_result_t r);
