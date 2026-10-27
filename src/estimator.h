/*
 * OBSERVADORES DE ESTADO DEL MOTOR DC: Luenberger (predictor) y filtro de Kalman lineal.
 *
 * Mismo diseño del Laboratorio 4 de Control Digital, llevado al ESP32:
 *   estado  x = [ i ; w ]   corriente de armadura [A] y velocidad del eje de salida [rad/s]
 *   entrada u = voltaje aplicado al motor [V]
 *   medida  y = w  (velocidad del encoder: diferencia de cuentas en Ts)       C = [ 0  1 ]
 *
 *   Modelo continuo (eje de salida):   La di/dt = u - Ra i - Kv w      J dw/dt = Kv i - B w
 *   Discretización exacta con retenedor de orden cero (ZOH):  A = e^(Ac Ts),  B = int_0^Ts e^(Ac t) dt Bc
 *
 *   Luenberger (predictor, como en la guía):
 *       x^(k) = A x^(k-1) + B u(k-1) + L [ y(k-1) - C x^(k-1) ]
 *       L por Ackermann:  L = Phi(A) [C; CA]^-1 [0; 1],  Phi(z) = (z - p1)(z - p2)
 *       con C = [0 1] eso se reduce a  L = (columna 1 de Phi(A)) / A21.
 *
 *   Kalman (predicción con u(k-1), corrección con y(k)):
 *       x-(k) = A x+(k-1) + B u(k-1)          P-(k) = A P+(k-1) A' + Q
 *       K = P- C' (C P- C' + R)^-1             x+(k) = x- + K (y(k) - C x-)        P+ = (I - K C) P-
 *
 * Código C puro (sin ESP-IDF): se prueba en el PC contra los números del laboratorio (tools/host_tests).
 */
#pragma once
#include <stdbool.h>

typedef struct {
    float ra, la, kv, b, j;          /* modelo del motor referido al eje de salida (config.h, MOT_*) */
} motor_model_t;

typedef struct {
    float polo_re, polo_im;          /* polos del Luenberger: re +/- j im (im = 0 -> polo doble real) */
    float sig_i, sig_w, sig_y;       /* Kalman: Q = diag(sig_i^2, sig_w^2), R = sig_y^2   [A], [rad/s], [rad/s] */
    float p0_i, p0_w;                /* Kalman: P0 = diag(p0_i^2, p0_w^2) */
} est_params_t;

typedef struct {
    /* diseño */
    motor_model_t m;
    est_params_t  par;
    float ts;
    float A[2][2], B[2];             /* modelo discreto */
    float L[2];                      /* ganancia del Luenberger */
    bool  ok;                        /* false si el diseño falló (no observable, polos fuera del círculo) */
    /* estado del Luenberger */
    float xl[2];                     /* x^(k) */
    float y_prev;
    /* estado del Kalman */
    float xp[2], Pm[2][2];           /* a priori  x-(k), P-(k) */
    float xk[2], Pk[2][2];           /* a posteriori x+(k), P+(k) */
    float K[2];
    float u_prev;
    bool  primero;
} est_t;

/* Diseña (discretiza, calcula L) y reinicia. Devuelve false si el diseño no es válido. */
bool est_design(est_t *e, const motor_model_t *m, const est_params_t *p, float ts);

/* x^ = 0, x+ = 0, P = P0. */
void est_reset(est_t *e);

/*
 * Un paso del observador en el instante k.
 *   u_prev: voltaje medio aplicado durante el intervalo (k-1, k]   [V]
 *   y:      velocidad medida en k [rad/s]  (ignorada si tiene_medida = false: solo modelo)
 */
void est_step(est_t *e, float u_prev, float y, bool tiene_medida);

/* Voltaje del motor a partir del PWM con signo (duty_frac en [-1, 1]): ganancia y zona muerta. */
float est_voltaje(float duty_frac, float vbus, float vgain, float v0);
