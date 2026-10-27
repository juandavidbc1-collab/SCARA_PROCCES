/* Pruebas en PC de cinemática y trayectoria.  Compilar: ver tools/host_tests/run.sh */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "kinematics.h"
#include "traj.h"

static int fallos = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FALLO: " __VA_ARGS__); printf("\n"); fallos++; } } while (0)
#define D2R(d) ((d) * (float)M_PI / 180.0f)

int main(void)
{
    kin_cfg_t c = { .l1 = 160.0f, .l2 = 160.0f, .sing_margin = D2R(5.0f) };
    float qmin[2] = { D2R(-120), D2R(-140) }, qmax[2] = { D2R(120), D2R(140) };

    /* 1) FK(IK(p)) = p en una malla dentro del espacio de trabajo */
    int n = 0, ok = 0, lim = 0, sing = 0, inal = 0;
    float emax = 0;
    for (float x = -340; x <= 340; x += 10)
        for (float y = -340; y <= 340; y += 10) {
            float q[2]; int cod;
            ik_status_t s = kin_ik(&c, x, y, +1, qmin, qmax, q, &cod);
            n++;
            if (s == IK_OK) {
                ok++;
                float xr, yr; kin_fk(&c, q[0], q[1], &xr, &yr);
                float e = hypotf(xr - x, yr - y);
                if (e > emax) emax = e;
                CHECK(e < 0.05f, "FK(IK) error %.4f mm en (%g,%g)", e, x, y);
                CHECK(q[0] >= qmin[0] && q[0] <= qmax[0] && q[1] >= qmin[1] && q[1] <= qmax[1], "limites");
            } else if (s == IK_LIMITES) lim++;
            else if (s == IK_SINGULAR) sing++;
            else inal++;
        }
    printf("IK malla: %d puntos -> ok %d, limites %d, singular %d, inalcanzable %d; error FK(IK) max = %.5f mm\n",
           n, ok, lim, sing, inal, emax);

    /* 2) casos puntuales */
    float q[2]; int cod;
    CHECK(kin_ik(&c, 400, 0, 1, qmin, qmax, q, &cod) == IK_INALCANZABLE, "400,0 debe ser inalcanzable");
    CHECK(kin_ik(&c, 319.9f, 0, 1, qmin, qmax, q, &cod) == IK_SINGULAR, "borde exterior debe ser singular");
    CHECK(kin_ik(&c, 0, 0, 1, qmin, qmax, q, &cod) == IK_SINGULAR, "origen (brazo plegado) singular");
    CHECK(kin_ik(&c, 200, 100, 1, qmin, qmax, q, &cod) == IK_OK && cod == 1 && q[1] > 0, "codo +");
    CHECK(kin_ik(&c, 200, 100, -1, qmin, qmax, q, &cod) == IK_OK && cod == -1 && q[1] < 0, "codo -");
    /* límites que solo permiten un codo: q2 en [0,140] -> pedir codo - debe caer al codo + */
    float qmin2[2] = { D2R(-120), D2R(0) };
    CHECK(kin_ik(&c, 200, 100, -1, qmin2, qmax, q, &cod) == IK_OK && cod == 1, "debe conmutar de codo");
    float qmin3[2] = { D2R(100), D2R(0) }, qmax3[2] = { D2R(120), D2R(10) };
    CHECK(kin_ik(&c, 200, 100, 1, qmin3, qmax3, q, &cod) == IK_LIMITES, "debe violar limites");

    /* 3) trayectoria: continuidad, respeto de límites y llegada exacta */
    float q0[2] = { 0, 0 }, q1[2] = { D2R(60), D2R(-20) };
    float vmax[2] = { D2R(60), D2R(60) }, amax[2] = { D2R(120), D2R(120) };
    traj_t t;
    CHECK(traj_plan(&t, q0, q1, vmax, amax), "plan");
    float qa[2], qda[2], qp[2] = { 0, 0 };
    float vpk[2] = { 0, 0 }, apk[2] = { 0, 0 }, qdp[2] = { 0, 0 };
    float dt = 0.001f;
    for (float tm = 0; tm <= t.T + 0.05f; tm += dt) {
        traj_eval(&t, tm, qa, qda);
        for (int j = 0; j < 2; j++) {
            if (fabsf(qda[j]) > vpk[j]) vpk[j] = fabsf(qda[j]);
            float a = (qda[j] - qdp[j]) / dt;
            if (tm > dt && fabsf(a) > apk[j]) apk[j] = fabsf(a);
            qdp[j] = qda[j];
            CHECK(fabsf(qa[j] - qp[j]) < D2R(60) * dt * 1.05f + 1e-6f || tm == 0, "salto de posicion");
            qp[j] = qa[j];
        }
    }
    CHECK(fabsf(qa[0] - q1[0]) < 1e-5f && fabsf(qa[1] - q1[1]) < 1e-5f, "no llega al destino");
    printf("Trayectoria 60/-20 deg: T = %.3f s, vmax J1 = %.1f deg/s, J2 = %.1f deg/s, amax J1 = %.1f, J2 = %.1f deg/s2\n",
           t.T, vpk[0] * 180 / M_PI, vpk[1] * 180 / M_PI, apk[0] * 180 / M_PI, apk[1] * 180 / M_PI);
    CHECK(vpk[0] <= vmax[0] * 1.001f && vpk[1] <= vmax[1] * 1.001f, "excede vmax");
    CHECK(apk[0] <= amax[0] * 1.02f && apk[1] <= amax[1] * 1.02f, "excede amax");

    /* triangular */
    float q1b[2] = { D2R(3), D2R(1) };
    traj_plan(&t, q0, q1b, vmax, amax);
    printf("Trayectoria corta 3/1 deg (triangular): T = %.3f s\n", t.T);
    traj_eval(&t, t.T * 0.5f, qa, qda);
    CHECK(fabsf(qa[0] - q1b[0] * 0.5f) < 1e-5f, "punto medio triangular");

    CHECK(!traj_plan(&t, q0, q0, vmax, amax), "movimiento nulo");

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
