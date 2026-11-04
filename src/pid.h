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
 *   - En movimiento, con |e| <= e_zone el integrador se mantiene sin acumular.
 *   - Con la consigna quieta, I es un empuje contra la fricción estática: crece a ritmo fijo y se retira al moverse el eje.
 *   - fric: en movimiento se aplica siempre, según el sentido de la consigna. Con la consigna quieta se usa
 *     fric_quieto, solo si el error supera e_zone, para no provocar oscilación de reposo.
 */
#pragma once

typedef struct {
    float kp, ki, kd, kff;
    float i_max;         /* límite del integrador [pwm] */
    float fric;          /* compensación de fricción seca en movimiento [pwm] */
    float fric_quieto;   /* la misma, con la consigna quieta [pwm] */
    float e_zone;        /* error mínimo para aplicar fric [rad] */
    float e_empuje;      /* con la consigna quieta, por debajo de este error no se empuja [rad] */
    float ritmo;         /* cómo crece el empuje [pwm/s] */
    float empuje_max;    /* [pwm] */
    float se_mueve;      /* velocidad a partir de la cual el eje ya se soltó [rad/s] */
    float i;             /* estado del integrador */
} pid_ctl_t;

void  pid_reset(pid_ctl_t *p);
/* e: error [rad], qd_ref y qd_meas [rad/s], dt [s], out_max: saturación [pwm]. Devuelve u en [-out_max, out_max]. */
float pid_step(pid_ctl_t *p, float e, float qd_ref, float qd_meas, float dt, float out_max);
