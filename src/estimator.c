#include "estimator.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ e^(M) de 3x3 (ZOH aumentado)
 * Escalado y elevación al cuadrado + serie de Taylor, en double (solo se usa al diseñar, no en el lazo). */
static void mat3_mul(const double a[3][3], const double b[3][3], double r[3][3])
{
    double t[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double s = 0;
            for (int k = 0; k < 3; k++) s += a[i][k] * b[k][j];
            t[i][j] = s;
        }
    memcpy(r, t, sizeof t);
}

static void expm3(const double m[3][3], double r[3][3])
{
    double norm = 0;
    for (int i = 0; i < 3; i++) {
        double s = 0;
        for (int j = 0; j < 3; j++) s += fabs(m[i][j]);
        if (s > norm) norm = s;
    }
    int sq = 0;
    while (norm > 0.25 && sq < 30) { norm *= 0.5; sq++; }
    double a[3][3], term[3][3], sum[3][3];
    double esc = ldexp(1.0, -sq);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            a[i][j] = m[i][j] * esc;
            term[i][j] = sum[i][j] = (i == j) ? 1.0 : 0.0;
        }
    for (int n = 1; n <= 14; n++) {
        mat3_mul(term, a, term);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) { term[i][j] /= n; sum[i][j] += term[i][j]; }
    }
    for (int s = 0; s < sq; s++) mat3_mul(sum, sum, sum);
    memcpy(r, sum, sizeof sum);
}

/* ------------------------------------------------------------------ diseño */
bool est_design(est_t *e, const motor_model_t *m, const est_params_t *p, float ts)
{
    memset(e, 0, sizeof *e);
    e->m = *m;
    e->par = *p;
    e->ts = ts;

    /* Ac, Bc del motor */
    double Ac[2][2] = { { -m->ra / m->la, -m->kv / m->la }, { m->kv / m->j, -m->b / m->j } };
    double Bc[2] = { 1.0 / m->la, 0.0 };
    double M[3][3] = { { Ac[0][0] * ts, Ac[0][1] * ts, Bc[0] * ts },
                       { Ac[1][0] * ts, Ac[1][1] * ts, Bc[1] * ts },
                       { 0, 0, 0 } };
    double E[3][3];
    expm3(M, E);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) e->A[i][j] = (float)E[i][j];
        e->B[i] = (float)E[i][2];
    }

    /* Luenberger por Ackermann. Phi(z) = z^2 - 2 re z + (re^2 + im^2) */
    double re = p->polo_re, im = p->polo_im;
    double a1 = -2.0 * re, a0 = re * re + im * im;
    double A11 = E[0][0], A12 = E[0][1], A21 = E[1][0], A22 = E[1][1];
    /* columna 1 de Phi(A) = A^2 + a1 A + a0 I */
    double phi11 = A11 * A11 + A12 * A21 + a1 * A11 + a0;
    double phi21 = A21 * A11 + A22 * A21 + a1 * A21;
    bool estable = (sqrt(a0) < 1.0);                  /* |polo| < 1 */
    bool observable = fabs(A21) > 1e-9;               /* rango [C; CA] = 2  <=>  A21 != 0 */
    if (observable) {
        e->L[0] = (float)(phi11 / A21);
        e->L[1] = (float)(phi21 / A21);
    }
    e->ok = estable && observable && isfinite(e->A[0][0]) && isfinite(e->L[0]) && isfinite(e->L[1]);
    est_reset(e);
    return e->ok;
}

void est_reset(est_t *e)
{
    e->xl[0] = e->xl[1] = 0.0f;
    e->xp[0] = e->xp[1] = 0.0f;
    e->xk[0] = e->xk[1] = 0.0f;
    e->y_prev = 0.0f;
    e->u_prev = 0.0f;
    e->K[0] = e->K[1] = 0.0f;
    float pi = e->par.p0_i * e->par.p0_i, pw = e->par.p0_w * e->par.p0_w;
    e->Pm[0][0] = e->Pk[0][0] = pi;
    e->Pm[1][1] = e->Pk[1][1] = pw;
    e->Pm[0][1] = e->Pm[1][0] = e->Pk[0][1] = e->Pk[1][0] = 0.0f;
    e->primero = true;
}

/* ------------------------------------------------------------------ un paso */
void est_step(est_t *e, float u_prev, float y, bool med)
{
    const float (*A)[2] = (const float (*)[2])e->A;
    const float *B = e->B;

    /* ---- Luenberger (predictor): usa la medida y la entrada del instante anterior ---- */
    if (e->primero) {
        e->y_prev = med ? y : 0.0f;
    } else {
        float x0 = e->xl[0], x1 = e->xl[1];
        float inn = med ? (e->y_prev - x1) : 0.0f;                   /* y(k-1) - C x^(k-1) */
        e->xl[0] = A[0][0] * x0 + A[0][1] * x1 + B[0] * u_prev + e->L[0] * inn;
        e->xl[1] = A[1][0] * x0 + A[1][1] * x1 + B[1] * u_prev + e->L[1] * inn;
        e->y_prev = med ? y : 0.0f;
    }

    /* ---- Kalman ---- */
    if (e->primero) {
        /* primera muestra (igual que el laboratorio): x+ = condición inicial, P+ = P0, sin corrección */
        e->xp[0] = e->xk[0];
        e->xp[1] = e->xk[1];
        e->K[0] = e->K[1] = 0.0f;
        e->u_prev = u_prev;
        e->primero = false;
        return;
    } else {
        float x0 = e->xk[0], x1 = e->xk[1];
        e->xp[0] = A[0][0] * x0 + A[0][1] * x1 + B[0] * u_prev;
        e->xp[1] = A[1][0] * x0 + A[1][1] * x1 + B[1] * u_prev;
        /* P- = A P+ A' + Q   (P simétrica) */
        float p00 = e->Pk[0][0], p01 = e->Pk[0][1], p11 = e->Pk[1][1];
        float m00 = A[0][0] * p00 + A[0][1] * p01, m01 = A[0][0] * p01 + A[0][1] * p11;
        float m10 = A[1][0] * p00 + A[1][1] * p01, m11 = A[1][0] * p01 + A[1][1] * p11;
        e->Pm[0][0] = m00 * A[0][0] + m01 * A[0][1] + e->par.sig_i * e->par.sig_i;
        e->Pm[0][1] = e->Pm[1][0] = m00 * A[1][0] + m01 * A[1][1];
        e->Pm[1][1] = m10 * A[1][0] + m11 * A[1][1] + e->par.sig_w * e->par.sig_w;
    }
    if (med) {
        float S = e->Pm[1][1] + e->par.sig_y * e->par.sig_y;       /* C P- C' + R */
        e->K[0] = e->Pm[0][1] / S;
        e->K[1] = e->Pm[1][1] / S;
        float inn = y - e->xp[1];
        e->xk[0] = e->xp[0] + e->K[0] * inn;
        e->xk[1] = e->xp[1] + e->K[1] * inn;
        /* P+ = (I - K C) P-  con C = [0 1] */
        e->Pk[0][0] = e->Pm[0][0] - e->K[0] * e->Pm[1][0];
        e->Pk[0][1] = e->Pm[0][1] - e->K[0] * e->Pm[1][1];
        e->Pk[1][0] = e->Pk[0][1];
        e->Pk[1][1] = e->Pm[1][1] - e->K[1] * e->Pm[1][1];
    } else {
        e->K[0] = e->K[1] = 0.0f;
        e->xk[0] = e->xp[0]; e->xk[1] = e->xp[1];
        memcpy(e->Pk, e->Pm, sizeof e->Pk);
    }
    e->u_prev = u_prev;
    e->primero = false;
}

float est_voltaje(float duty_frac, float vbus, float vgain, float v0)
{
    float v = vgain * vbus * duty_frac;
    float a = fabsf(v) - v0;
    if (a <= 0.0f) return 0.0f;
    return copysignf(a, v);
}
