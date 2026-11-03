#include "rst.h"

void rst_set_pid(rst_ctl_t *c, float kp, float ki, float kd, float s1, float b, float c_w, float kff, float ts)
{
    const float p[3] = { 1.0f, s1 - 1.0f, -s1 };      /* (1 - q^-1)(1 + s1 q^-1) */
    const float i[3] = { 1.0f, s1, 0.0f };            /* (1 + s1 q^-1) */
    const float d[3] = { 1.0f, -2.0f, 1.0f };         /* (1 - q^-1)^2 */
    const float kiT = ki * ts, kdT = kd / ts;
    for (int n = 0; n < 3; n++) {
        c->r[n] = kp * p[n] + kiT * i[n] + kdT * d[n];
        c->t[n] = b * kp * p[n] + kiT * i[n] + c_w * kdT * d[n];
    }
    /* prealimentación: (Kff/Ts) S (1 - q^-1) */
    const float ff = kff / ts;
    c->t[3] = -p[2] * ff;
    c->t[2] += (p[2] - p[1]) * ff;
    c->t[1] += (p[1] - p[0]) * ff;
    c->t[0] += p[0] * ff;
    c->s1 = s1; c->kp = kp; c->ki = ki; c->kd = kd; c->b = b; c->c = c_w; c->kff = kff;
}

void rst_reset(rst_ctl_t *c, float y0)
{
    c->y1 = c->y2 = c->r1 = c->r2 = c->r3 = y0;
    c->u1 = c->u2 = 0.0f;
}

float rst_step(rst_ctl_t *c, float r, float y, float umax)
{
    /* T(1) = R(1) por construcción: se resta y(k) a todo para no perder precisión en float32 con
     * coeficientes de decenas de miles y posiciones de varios radianes. */
    float v = c->t[0] * (r - y) + c->t[1] * (c->r1 - y) + c->t[2] * (c->r2 - y) + c->t[3] * (c->r3 - y)
            - c->r[1] * (c->y1 - y) - c->r[2] * (c->y2 - y);
    float u = v + (1.0f - c->s1) * c->u1 + c->s1 * c->u2;

    if (u >  umax) u =  umax;
    if (u < -umax) u = -umax;

    c->y2 = c->y1; c->y1 = y;
    c->r3 = c->r2; c->r2 = c->r1; c->r1 = r;
    c->u2 = c->u1; c->u1 = u;
    return u;
}

void rst_get_pid(const rst_ctl_t *c, float out[7])
{
    out[0] = c->kp; out[1] = c->ki; out[2] = c->kd; out[3] = c->s1; out[4] = c->b; out[5] = c->c; out[6] = c->kff;
}
