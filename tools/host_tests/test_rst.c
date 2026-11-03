/* Prueba en el PC de src/rst.c: arranque sin salto, paridad con Python, saturación y seguimiento en lazo cerrado. */
#include <math.h>
#include <stdio.h>
#include "rst.h"
#include "rst_vectors.h"

static int fallos = 0;
#define CHECK(c, ...) do { printf((c) ? "  [ok] " : "  [FALLO] "); printf(__VA_ARGS__); printf("\n"); if (!(c)) fallos++; } while (0)

/* Planta discreta ZOH: K = 0.9974 rad/s/V, tau = 0.14 s, Ts = 10 ms, en rad por cuenta de PWM. */
typedef struct { float y, w; } planta_t;
static float planta(planta_t *p, float u)
{
    const float Ts = 0.01f, tau = 0.14f, K = 0.9974f * 12.0f / 1023.0f;
    const float a = exp(-Ts / tau);
    p->y += K * u * Ts + (p->w - K * u) * tau * (1.0f - a);
    p->w = K * u + (p->w - K * u) * a;
    return p->y;
}

int main(void)
{
    rst_ctl_t c;

    /* 1) sin salto: consigna = medición -> u = 0, aunque la posición no sea cero */
    rst_set_pid(&c, 5000, 8000, 200, -0.2f, 1, 1, 85.0f, 0.01f);
    rst_reset(&c, 1.234f);
    float umax = 0;
    for (int k = 0; k < 20; k++) umax = fmaxf(umax, fabsf(rst_step(&c, 1.234f, 1.234f, 650)));
    CHECK(umax == 0.0f, "con r = y = 1.234 rad la salida es 0 (arranque sin salto)");

    /* 2) paridad con la referencia en Python */
    float err = 0; int nv = (int)(sizeof RV / sizeof RV[0]);
    for (int i = 0; i < nv; i++) {
        if (RV[i].k == 0) { rst_set_pid(&c, RV_KP, RV_KI, RV_KD, RV_S1, RV[i].b, RV[i].c, RV_KFF, RV_TS); rst_reset(&c, 0.5f); }
        err = fmaxf(err, fabsf(rst_step(&c, RV[i].r, RV[i].y, RV_UMAX) - RV[i].u));
    }
    CHECK(err < 0.02f, "coincide con rst_design.py en %d muestras (error max %.5f PWM)", nv, err);

    /* 3) seguimiento y saturación: escalón grande con u limitada */
    planta_t p = { 0, 0 };
    rst_set_pid(&c, 5000, 8000, 200, -0.2f, 1, 1, 0.0f, 0.01f);   /* posicional: el que más exige a la salida */
    rst_reset(&c, 0);
    float y = 0, ymax = 0, usat = 0;
    for (int k = 0; k < 400; k++) {
        float u = rst_step(&c, 1.0f, y, 150);
        usat = fmaxf(usat, fabsf(u));
        y = planta(&p, u); ymax = fmaxf(ymax, y);
    }
    CHECK(usat == 150.0f, "llega a la saturacion y no la pasa (max %.1f)", usat);
    CHECK(fabsf(y - 1.0f) < 0.002f, "llega a la consigna sin error (y = %.4f)", y);
    CHECK(ymax < 1.15f, "sin enrollar el integrador: sobreimpulso limitado (ymax = %.3f)", ymax);

    /* 4) las ganancias se pueden leer de vuelta */
    float g[7];
    rst_set_pid(&c, 1, 2, 3, -0.5f, 0, 1, 7, 0.01f);
    rst_get_pid(&c, g);
    CHECK(g[0] == 1 && g[1] == 2 && g[2] == 3 && g[3] == -0.5f && g[4] == 0 && g[5] == 1 && g[6] == 7, "rst_get_pid devuelve lo configurado");

    printf(fallos ? "%d FALLOS\n" : "TODO OK\n", fallos);
    return fallos != 0;
}
