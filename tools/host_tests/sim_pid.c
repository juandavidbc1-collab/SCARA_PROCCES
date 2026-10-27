/*
 * Simulación del lazo de posición de UNA articulación con el modelo del motor Pololu 37D (valores
 * del documento de diseño, lado motor): R=2.4 ohm, L=1.5 mH, Ke=0.01056, J=3e-6, B=2.97e-6.
 * Se añade: reductor 100.37:1, inercia de carga reflejada, fricción seca, encoder cuantizado
 * (6424 cuentas/rev de salida), 1 ms de retardo de cómputo, límite de PWM y rampa de PWM.
 * Usa EXACTAMENTE src/pid.c y src/traj.c.
 *   uso: sim_pid kp ki kd kff fric  [carga_kgm2]
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pid.h"
#include "traj.h"

#define N 100.37f
#define CPR 6424.0f

int main(int argc, char **argv)
{
    pid_ctl_t p = { .kp = 4000, .ki = 3000, .kd = 150, .kff = 80, .i_max = 250, .fric = 60, .e_zone = 0.0026f };
    if (argc >= 6) { p.kp = atof(argv[1]); p.ki = atof(argv[2]); p.kd = atof(argv[3]); p.kff = atof(argv[4]); p.fric = atof(argv[5]); }
    float Jload_joint = (argc >= 7) ? atof(argv[6]) : 0.04f;
    const float R = 2.4f, L = 1.5e-3f, Ke = 0.01056f, B = 2.97e-6f, Tc = 3e-3f;
    float J = 3e-6f + Jload_joint / (N * N);
    const float dt = 1e-4f, dtc = 1e-3f, VM = 12.0f, OUTMAX = 650, SLEW = 3.0f;

    float amp_deg = 45.0f;
    float q0[2] = { 0, 0 }, q1[2] = { amp_deg * M_PI / 180, 0 };
    float vmax[2] = { 60 * M_PI / 180, 60 * M_PI / 180 }, amax[2] = { 120 * M_PI / 180, 120 * M_PI / 180 };
    traj_t tr; traj_plan(&tr, q0, q1, vmax, amax);

    float i = 0, w = 0, th = 0;            /* corriente, vel. motor, ángulo motor */
    float duty = 0, u_cmd = 0, u_next = 0;
    float qmeas = 0, qd = 0, qprev = 0;
    float errmax_move = 0, ipk = 0, tsettle = -1, overshoot = 0, tlast_out = 0;
    float tend = tr.T + 1.5f;
    int step = 0;
    for (float t = 0; t < tend; t += dt, step++) {
        if (step % 10 == 0) {                         /* lazo de control a 1 kHz */
            int cnt = (int)floorf(th / N / (2 * M_PI) * CPR);   /* encoder cuantizado */
            qmeas = cnt * 2 * M_PI / CPR;
            float qdraw = (qmeas - qprev) / dtc; qprev = qmeas;
            float a = dtc / (0.010f + dtc); qd += a * (qdraw - qd);
            float qr[2], qdr[2]; traj_eval(&tr, t, qr, qdr);
            float u = pid_step(&p, qr[0] - qmeas, qdr[0], qd, dtc, OUTMAX);
            float d = u - duty;                        /* rampa de PWM */
            if (d > SLEW) d = SLEW; if (d < -SLEW) d = -SLEW;
            duty += d;
            u_cmd = u_next; u_next = duty;             /* 1 muestra de retardo */
            float e = fabsf(qr[0] - qmeas);
            if (t < tr.T && e > errmax_move) errmax_move = e;
            if (t > tr.T) {
                float ov = (qmeas - q1[0]); if (ov > overshoot) overshoot = ov;
                if (fabsf(qmeas - q1[0]) > 0.005f) tlast_out = t - tr.T;     /* fuera de ±0.29 deg */
            }
        }
        float V = u_cmd / 1023.0f * VM;
        float di = (V - R * i - Ke * w) / L;
        float Tl = Tc * (w > 0.5f ? 1 : (w < -0.5f ? -1 : w / 0.5f));
        float dw = (Ke * i - B * w - Tl) / J;
        i += di * dt; w += dw * dt; th += w * dt;
        if (fabsf(i) > ipk) ipk = fabsf(i);
    }
    printf("kp=%.0f ki=%.0f kd=%.0f kff=%.0f fric=%.0f carga=%.3f kg m2 | mov %.0f deg en %.2f s\n",
           p.kp, p.ki, p.kd, p.kff, p.fric, Jload_joint, amp_deg, tr.T);
    printf("  error max siguiendo la trayectoria = %.2f deg | sobreimpulso = %.2f deg | tiempo hasta quedar en ±0.29 deg = %.2f s | corriente pico = %.2f A | error final = %.3f deg\n",
           errmax_move * 180 / M_PI, overshoot * 180 / M_PI, tlast_out, ipk, (q1[0] - qmeas) * 180 / M_PI);
    return 0;
}
