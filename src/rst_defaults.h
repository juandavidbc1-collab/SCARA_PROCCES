/* Generado por tools/rst/simulate.py: no editar a mano. Ver docs/rst/REPORTE_RST.md. */
#pragma once
/* Forma con mejor resultado en la simulación */
#define RST_FORM_DEFAULT RST_FORM_MASK

/* [forma: pos, inc, pi_d, mask][eje] = { kp, ki, kd, s1, kff }   (PWM/rad, Ts = 1 ms) */
static const float RST_GAINS[4][2][5] = {
    { { 2779.435f, 9938.865f, 14.65242f, -0.9201301f, 85.472f }, { 2055.329f, 7361.250f, 8.70360f, -0.9228635f, 85.472f } },
    { { 8120.400f, 52906.729f, 43.01584f, -0.8849445f, 85.472f }, { 5939.246f, 38748.353f, 28.06740f, -0.8876284f, 85.472f } },
    { { 2779.435f, 9938.865f, 14.65242f, -0.9201301f, 85.472f }, { 2055.329f, 7361.250f, 8.70360f, -0.9228635f, 85.472f } },
    { { 5000.000f, 3000.000f, 16.36364f, -0.9090909f, 260.000f }, { 5000.000f, 3000.000f, 16.36364f, -0.9090909f, 260.000f } },
};
