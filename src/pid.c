#include "pid.h"
#include <math.h>
#include <stdbool.h>

void pid_reset(pid_ctl_t *p) { p->i = 0.0f; }

float pid_step(pid_ctl_t *p, float e, float qd_ref, float qd_meas, float dt, float out_max)
{
    float base = p->kff * qd_ref + p->kp * e + p->kd * (qd_ref - qd_meas);
    float u = base + p->i;

    /* anti-windup: integra solo si no se empuja más hacia la saturación */
    bool sat = (fabsf(u) >= out_max) && ((u > 0.0f) == (e > 0.0f));
    if (!sat) {
        p->i += p->ki * e * dt;
        if (p->i >  p->i_max) p->i =  p->i_max;
        if (p->i < -p->i_max) p->i = -p->i_max;
        u = base + p->i;
    }

    if (fabsf(e) > p->e_zone && u != 0.0f)
        u += copysignf(p->fric, u);

    if (u >  out_max) u =  out_max;
    if (u < -out_max) u = -out_max;
    return u;
}
