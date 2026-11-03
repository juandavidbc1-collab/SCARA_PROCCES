/*
 * CONTROLADOR RST DE UNA ARTICULACIÓN (Ts = RST_TS_S).
 *
 *     S(q^-1) u(k) = T(q^-1) r(k) - R(q^-1) y(k)
 *     S = (1 - q^-1)(1 + s1 q^-1)
 *     R = Kp (1 - q^-1)(1 + s1 q^-1) + Ki Ts (1 + s1 q^-1) + (Kd/Ts)(1 - q^-1)^2
 *     T = b Kp (1 - q^-1)(1 + s1 q^-1) + Ki Ts (1 + s1 q^-1) + c (Kd/Ts)(1 - q^-1)^2 + (Kff/Ts) S (1 - q^-1)
 *
 * El último término de T es la prealimentación de velocidad: no cambia polos ni ganancia estática.
 * (b, c) eligen cómo entra la consigna:  posicional (1,1), incremental I-PD (0,0), enmascarado PI-D (1,0).
 * r e y en rad; u en cuentas de PWM.  Diseño y simulaciones: tools/rst/ y docs/rst/REPORTE_RST.md.
 *
 * Anti-windup: la historia de u guarda u ya saturada, así que el integrador contenido en S no se enrolla.
 * La rampa de PWM NO va aquí: en esta forma, recortar el cambio por muestra borra el término proporcional
 * de la historia y el eje solo se corrige con el integrador. La rampa se aplica a la salida (control.c).
 */
#pragma once

typedef struct {
    float r[3], t[4];        /* coeficientes de R y T */
    float s1;
    float kp, ki, kd, b, c, kff;  /* parámetros con los que se calculó */
    float y1, y2, r1, r2, r3, u1, u2;
} rst_ctl_t;

void  rst_set_pid(rst_ctl_t *c, float kp, float ki, float kd, float s1, float b, float c_w, float kff, float ts);
void  rst_reset(rst_ctl_t *c, float y0);               /* historia en y0 y u = 0: arranque sin salto */
/* Una muestra. umax: saturación [PWM]. */
float rst_step(rst_ctl_t *c, float r, float y, float umax);
void  rst_get_pid(const rst_ctl_t *c, float out[7]);   /* kp ki kd s1 b c kff */
