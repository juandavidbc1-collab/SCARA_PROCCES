#include "pid.h"
#include <math.h>
#include <stdbool.h>

void pid_reset(pid_ctl_t *p) { p->i = 0.0f; }

float pid_step(pid_ctl_t *p, float e, float qd_ref, float qd_meas, float dt, float out_max)
{
    float base = p->kff * qd_ref + p->kp * e + p->kd * (qd_ref - qd_meas);
    float u = base + p->i;

    if (qd_ref == 0.0f) {
        /* Consigna quieta. Con el error pequeño, el empuje necesario depende de la fricción estática, no del error: crece a ritmo
         * fijo hasta soltar el eje, y al moverse se retira de golpe para que no se pase del punto. */
        if (fabsf(qd_meas) > p->se_mueve || fabsf(e) <= p->e_empuje) {
            p->i = 0.0f;
        } else {
            p->i += copysignf(p->ritmo * dt, e);
            if (p->i * e < 0.0f) p->i = 0.0f;
            if (p->i >  p->empuje_max) p->i =  p->empuje_max;
            if (p->i < -p->empuje_max) p->i = -p->empuje_max;
        }
        u = base + p->i;
        if (fabsf(e) > p->e_zone && u != 0.0f) u += copysignf(p->fric_quieto, u);
        if (u >  out_max) u =  out_max;
        if (u < -out_max) u = -out_max;
        return u;
    }

    /* anti-windup: integra solo si no se empuja más hacia la saturación */
    bool sat = (fabsf(u) >= out_max) && ((u > 0.0f) == (e > 0.0f));
    if (!sat) {
        /* Con el eje ya en su sitio no se acumula: la fricción lo sostiene, y si no el PWM subiría solo hasta parecer un motor trabado. */
        if (fabsf(e) > p->e_zone) p->i += p->ki * e * dt;
        if (p->i >  p->i_max) p->i =  p->i_max;
        if (p->i < -p->i_max) p->i = -p->i_max;
        u = base + p->i;
    }

    /* En movimiento, la fricción se compensa según el sentido de la consigna y sin pausas: si dependiera del error,
     * se encendería y apagaría con cada pequeña desviación, y el eje avanzaría a tirones. */
    /* Se reparte en los primeros 0,02 rad/s para que un eje casi quieto no salte de un lado a otro al cambiar de sentido. */
    float f = qd_ref / 0.02f;
    if (f >  1.0f) f =  1.0f;
    if (f < -1.0f) f = -1.0f;
    u += p->fric * f;

    if (u >  out_max) u =  out_max;
    if (u < -out_max) u = -out_max;
    return u;
}
