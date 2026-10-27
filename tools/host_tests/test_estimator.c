/*
 * Prueba en el PC de src/estimator.c (Luenberger + Kalman).
 *  1) El diseño reproduce los números del Laboratorio 4 (mismo modelo, Ts = 10 ms, polos 0,25 +/- 0,25j).
 *  2) Los polos de A - LC son los pedidos.
 *  3) La ganancia de Kalman en estado estable es la del laboratorio (K de velocidad = 0,266).
 *  4) Con una planta simulada y ruido de cuantización, los dos observadores convergen a la corriente y velocidad reales.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "estimator.h"

static int fallos = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("  >>> FALLO: " __VA_ARGS__); printf("\n"); fallos++; } else { printf("  [ok] " __VA_ARGS__); printf("\n"); } } while (0)

static int cerca(double a, double b, double tol) { return fabs(a - b) <= tol * (1.0 + fabs(b)); }

int main(void)
{
    const double k = M_PI / 30.0;                   /* rpm -> rad/s */
    const double N = 102.08;
    /* modelo del laboratorio (lado motor) llevado al eje de salida */
    motor_model_t m = { 50.0f, 0.060f, (float)(1.6368711e-3 * N), (float)(2.679347e-7 * N * N), (float)(3.4858e-8 * N * N) };
    /* Q, R y P0 del laboratorio en unidades del eje de salida */
    est_params_t p = { 0.25f, 0.25f, 0.01f, (float)(3.20694 / N), (float)(0.80956 * k), 0.10f, (float)(1.0 * k) };
    est_t e;
    bool ok = est_design(&e, &m, &p, 0.01f);
    printf("A = [%.7f %.7f; %.7f %.7f]  B = [%.7f; %.7f]  L = [%.7f; %.7f]\n",
           e.A[0][0], e.A[0][1], e.A[1][0], e.A[1][1], e.B[0], e.B[1], e.L[0], e.L[1]);
    CHECK(ok, "diseno valido");
    /* valores calculados con scipy (expm) y Ackermann a partir del modelo del laboratorio */
    CHECK(cerca(e.A[0][0], -0.0014785, 1e-4) && cerca(e.A[0][1], -0.00308602, 1e-4) &&
          cerca(e.A[1][0], 0.50976035, 1e-5) && cerca(e.A[1][1], 0.91345396, 1e-5), "A (ZOH) igual a la del laboratorio");
    CHECK(cerca(e.B[0], 0.01976948, 1e-5) && cerca(e.B[1], 0.07782983, 1e-5), "B (ZOH) igual a la del laboratorio");
    /* L del laboratorio = [0,02551 A/rpm; 4,4039 (rad/s motor)/rpm] -> en unidades del eje de salida: */
    CHECK(cerca(e.L[0], 0.02550786346 / k, 1e-5) && cerca(e.L[1], 4.403933927 / N / k, 1e-5),
          "L (Ackermann) igual a la del laboratorio: [%.5f, %.5f]", e.L[0], e.L[1]);

    /* polos de A - LC (C = [0 1]): traza = 2 re, determinante = re^2 + im^2 */
    double f11 = e.A[0][0], f12 = e.A[0][1] - e.L[0], f21 = e.A[1][0], f22 = e.A[1][1] - e.L[1];
    CHECK(cerca(f11 + f22, 0.5, 1e-5) && cerca(f11 * f22 - f12 * f21, 0.125, 1e-5), "polos de A - LC = 0,25 +/- 0,25j");

    /* ganancia de Kalman en estado estable (sin entrada) */
    for (int i = 0; i < 300; i++) est_step(&e, 0.0f, 0.0f, true);
    printf("K estable = [%.6g, %.6g]  (lab: K1 = -5,831e-5 A/rpm, K2 = 0,26597)\n", e.K[0], e.K[1]);
    CHECK(cerca(e.K[1], 0.26597, 2e-3) && cerca(e.K[0], -5.831e-5 / k, 2e-2), "ganancia de Kalman igual a la del laboratorio");

    /* planta simulada = el mismo modelo discreto, con medida cuantizada (encoder de 6424 cuentas, 10 ms) */
    est_reset(&e);
    double x0 = 0.0, x1 = 0.0;
    const double paso = 2.0 * M_PI / 6424.0 / 0.01;          /* rad/s por cuenta en 10 ms */
    double acum = 0.0, th = 0.0;
    double eiK = 0, eiL = 0, ewK = 0, ewL = 0; int n = 0;
    double u_prev = 0.0;
    for (int kk = 0; kk < 600; kk++) {
        double u = (kk < 300) ? 6.0 : -4.0;                     /* escalones de voltaje */
        /* medida: diferencia de cuentas enteras en 10 ms */
        double th_new = th + x1 * 0.01;
        double c0 = floor(th / (2 * M_PI / 6424.0)), c1 = floor(th_new / (2 * M_PI / 6424.0));
        double y = (c1 - c0) * paso;
        th = th_new; (void)acum;
        est_step(&e, (float)u_prev, (float)y, true);
        if ((kk > 100 && kk < 300) || kk > 400) {
            eiK += fabs(e.xk[0] - x0); eiL += fabs(e.xl[0] - x0);
            ewK += fabs(e.xk[1] - x1); ewL += fabs(e.xl[1] - x1); n++;
        }
        /* avanza la planta con la entrada de este intervalo */
        double n0 = e.A[0][0] * x0 + e.A[0][1] * x1 + e.B[0] * u;
        double n1 = e.A[1][0] * x0 + e.A[1][1] * x1 + e.B[1] * u;
        x0 = n0; x1 = n1; u_prev = u;
    }
    printf("errores medios: corriente Kalman %.2f mA, Luenberger %.2f mA | velocidad Kalman %.3f rpm, Luenberger %.3f rpm\n",
           1000 * eiK / n, 1000 * eiL / n, ewK / n / k, ewL / n / k);
    CHECK(eiK / n < 0.003 && ewK / n < 0.6 * k, "Kalman converge con medida cuantizada (corriente < 3 mA, velocidad < 0,6 rpm)");
    CHECK(eiL / n < 0.02 && ewL / n < 1.0 * k, "Luenberger converge con medida cuantizada (corriente < 20 mA, velocidad < 1 rpm)");
    CHECK(eiL > eiK, "el Kalman es menos ruidoso que el Luenberger en la corriente (como en el laboratorio)");

    /* sin medida (motor sin encoder): solo modelo */
    est_reset(&e); est_step(&e, 0, 0, false);
    for (int kk = 0; kk < 200; kk++) est_step(&e, 12.0f, 999.0f, false);
    CHECK(cerca(e.xk[0], 0.2, 2e-3) && cerca(e.xk[1] / k, 114.3, 2e-3), "sin encoder: solo modelo, a 12 V da 0,2 A y 114,3 rpm (%.4f A, %.2f rpm)",
          e.xk[0], e.xk[1] / k);

    /* diseños inválidos */
    est_params_t malo = p; malo.polo_re = 1.2f; malo.polo_im = 0;
    CHECK(!est_design(&e, &m, &malo, 0.01f), "polos fuera del circulo unitario -> diseno invalido");

    /* voltaje con zona muerta */
    CHECK(fabsf(est_voltaje(1.0f, 12, 1, 0.8f) - 11.2f) < 1e-5f && est_voltaje(0.05f, 12, 1, 0.8f) == 0.0f &&
          fabsf(est_voltaje(-0.5f, 12, 0.9782f, 1.197f) + (0.9782f * 6 - 1.197f)) < 1e-5f, "voltaje del modelo (ganancia y zona muerta, con signo)");

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
