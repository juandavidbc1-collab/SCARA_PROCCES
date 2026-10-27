/*
 * CONTROLADOR DE POSICIÓN: PID + prealimentación de velocidad + compensación de fricción seca.
 *
 *   u = Kff*qd_ref  +  Kp*e  +  I  +  Kd*(qd_ref - qd_meas)  +  sign(u0)*fric   [cuentas de PWM]
 *
 *   e = q_ref - q_meas [rad].  Unidades de ganancia: Kp [pwm/rad], Ki [pwm/(rad·s)], Kd [pwm/(rad/s)].
 *   - La derivada se calcula sobre la velocidad medida (filtrada fuera de este módulo): sin
 *     "patada" derivativa ante un cambio de consigna.
 *   - Anti-windup por condicionamiento: no integra si la salida está saturada y el error
 *     empuja más hacia la saturación; además el integrador está acotado (i_max).
 *   - fric: ayuda a vencer la zona muerta (fricción seca del reductor) solo cuando el error
 *     supera e_zone, para no provocar oscilación de reposo.
 */
#pragma once

typedef struct {
    float kp, ki, kd, kff;
    float i_max;         /* límite del integrador [pwm] */
    float fric;          /* compensación de fricción seca [pwm] */
    float e_zone;        /* error mínimo para aplicar fric [rad] */
    float i;             /* estado del integrador */
} pid_ctl_t;

void  pid_reset(pid_ctl_t *p);
/* e: error [rad], qd_ref y qd_meas [rad/s], dt [s], out_max: saturación [pwm]. Devuelve u en [-out_max, out_max]. */
float pid_step(pid_ctl_t *p, float e, float qd_ref, float qd_meas, float dt, float out_max);
