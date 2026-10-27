#include "traj.h"
#include <math.h>

bool traj_plan(traj_t *t, const float q0[TRAJ_NJ], const float q1[TRAJ_NJ],
               const float vmax[TRAJ_NJ], const float amax[TRAJ_NJ])
{
    float vs = 1e9f, as = 1e9f;
    bool any = false;
    for (int j = 0; j < TRAJ_NJ; j++) {
        float dq = q1[j] - q0[j];
        t->q0[j] = q0[j];
        t->dq[j] = dq;
        float a = fabsf(dq);
        if (a > 1e-6f) {
            any = true;
            vs = fminf(vs, vmax[j] / a);
            as = fminf(as, amax[j] / a);
        }
    }
    if (!any) {
        t->vs = t->as = t->ta = t->T = 0.0f;
        return false;
    }
    float ta = vs / as;
    if (vs * ta > 1.0f) {            /* no alcanza la velocidad máxima: perfil triangular */
        ta = sqrtf(1.0f / as);
        vs = as * ta;
        t->T = 2.0f * ta;
    } else {
        t->T = 1.0f / vs + ta;
    }
    t->vs = vs; t->as = as; t->ta = ta;
    return true;
}

void traj_eval(const traj_t *t, float tm, float q[TRAJ_NJ], float qd[TRAJ_NJ])
{
    float s, sd;
    if (t->T <= 0.0f || tm >= t->T) {
        s = 1.0f; sd = 0.0f;
    } else if (tm <= 0.0f) {
        s = 0.0f; sd = 0.0f;
    } else if (tm < t->ta) {
        s = 0.5f * t->as * tm * tm;
        sd = t->as * tm;
    } else if (tm < t->T - t->ta) {
        s = 0.5f * t->as * t->ta * t->ta + t->vs * (tm - t->ta);
        sd = t->vs;
    } else {
        float td = t->T - tm;
        s = 1.0f - 0.5f * t->as * td * td;
        sd = t->as * td;
    }
    for (int j = 0; j < TRAJ_NJ; j++) {
        q[j]  = t->q0[j] + s * t->dq[j];
        qd[j] = sd * t->dq[j];
    }
}
