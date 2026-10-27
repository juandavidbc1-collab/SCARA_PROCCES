#include "uros_cmd.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "control.h"
#include "kinematics.h"
#include "motion.h"

#define RAD2DEG (180.0f / (float)M_PI)
#define RPM2RADS ((float)M_PI / 30.0f)

static void cmd_status(char *reply, size_t n)
{
    ctl_status_t s; ctl_get_status(&s);
    float x, y; kin_fk(&KIN, s.q[0], s.q[1], &x, &y);
    snprintf(reply, n, "#INFO: [%s] q1=%.2f q2=%.2f deg x=%.1f y=%.1f mm home J1:%d J2:%d motores:%c%c%c falla:%s",
             ctl_mode_str(s.mode), s.q[0] * RAD2DEG, s.q[1] * RAD2DEG, x, y, s.homed[0], s.homed[1],
             (s.enabled & 1) ? '1' : '-', (s.enabled & 2) ? '2' : '-', (s.enabled & 4) ? '3' : '-',
             s.fault == F_NONE ? "ninguna" : ctl_fault_str(s.fault));
}

static void cmd_motors(char *reply, size_t n, const char *pre)
{
    uint8_t e = ctl_get_enabled();
    snprintf(reply, n, "%s motores: M1 %s, M2 %s, M3 %s", pre, (e & 1) ? "ON" : "off", (e & 2) ? "ON" : "off", (e & 4) ? "ON" : "off");
}

static void cmd_est(int j, char *reply, size_t n)
{
    est_params_t p; float L[2]; bool ok; float g, v0;
    ctl_get_est(j, &p, L, &ok); ctl_get_vmap(&g, &v0);
    snprintf(reply, n, "#INFO: M%d polos %.3f+-%.3fj L=[%.4g, %.4g]%s | Kalman sig_i=%.1f mA sig_w=%.2f rpm sig_y=%.2f rpm | V=%.3f*12*duty-%.2f",
             j + 1, p.polo_re, p.polo_im, L[0], L[1], ok ? "" : " INVALIDO", p.sig_i * 1000.0f, p.sig_w / RPM2RADS,
             p.sig_y / RPM2RADS, g, v0);
}

bool uros_cmd_exec(const char *line, char *reply, size_t n)
{
    char l[96];
    size_t k = 0;
    while (*line && (unsigned char)*line <= ' ') line++;                 /* espacios iniciales */
    for (; *line && k < sizeof l - 1; line++) l[k++] = (char)tolower((unsigned char)*line);
    l[k] = 0;
    while (k && (unsigned char)l[k - 1] <= ' ') l[--k] = 0;              /* espacios finales / \n */

    float f[5]; int j;
    if (!strcmp(l, "home")) {
        if (ctl_cmd_home()) { snprintf(reply, n, "#OK: home solicitado"); return true; }
        snprintf(reply, n, "#ERROR: home rechazado (falla activa o movimiento en curso)"); return false;
    }
    if (!strcmp(l, "stop"))  { ctl_cmd_stop();  snprintf(reply, n, "#OK: detenido (mantiene posicion)"); return true; }
    if (!strcmp(l, "off"))   { ctl_cmd_off();   snprintf(reply, n, "#OK: motores apagados"); return true; }
    if (!strcmp(l, "clear")) { ctl_cmd_clear(); snprintf(reply, n, "#OK: falla borrada"); return true; }
    if (!strcmp(l, "status")) { cmd_status(reply, n); return true; }
    if (!strcmp(l, "help")) {
        snprintf(reply, n, "#INFO: home stop off clear status | goto x y [z] | ang q1 q2 | elbow +-1 | pulse j | gain [j ...] | "
                           "motor j on/off | motors | est [j|reset] | luen j re im | kal j mA rpm rpm | vmap g v0");
        return true;
    }
    if (sscanf(l, "goto %f %f %f", &f[0], &f[1], &f[2]) == 3) return motion_goto_xyz(f[0], f[1], f[2], reply, n) == MOTION_OK;
    if (sscanf(l, "goto %f %f", &f[0], &f[1]) == 2)            return motion_goto_xyz(f[0], f[1], Z_FIJO_MM, reply, n) == MOTION_OK;
    if (sscanf(l, "ang %f %f", &f[0], &f[1]) == 2)             return motion_goto_ang(f[0], f[1], reply, n) == MOTION_OK;
    if (sscanf(l, "elbow %d", &j) == 1) {
        motion_set_elbow(j);
        snprintf(reply, n, "#OK: codo preferido = %+d", motion_get_elbow());
        return true;
    }
    if (sscanf(l, "pulse %d", &j) == 1) {
        if (ctl_cmd_pulse(j - 1)) { snprintf(reply, n, "#OK: pulso en el eje %d", j); return true; }
        snprintf(reply, n, "#ERROR: pulso rechazado (eje 1 o 2, y sin movimiento en curso)"); return false;
    }
    if (sscanf(l, "gain %d %f %f %f %f %f", &j, &f[0], &f[1], &f[2], &f[3], &f[4]) == 6) {
        if (ctl_cmd_gain(j - 1, f[0], f[1], f[2], f[3], f[4])) { snprintf(reply, n, "#OK: ganancias del eje %d actualizadas", j); return true; }
        snprintf(reply, n, "#ERROR: ganancias fuera de rango o eje invalido (eje 1 o 2)"); return false;
    }
    {
        char onoff[8];
        if (sscanf(l, "motor %d %7s", &j, onoff) == 2) {
            bool on = !strcmp(onoff, "on") || !strcmp(onoff, "1");
            bool off = !strcmp(onoff, "off") || !strcmp(onoff, "0");
            if ((on || off) && ctl_cmd_enable(j - 1, on)) { snprintf(reply, n, "#OK: motor %d %s", j, on ? "habilitado" : "deshabilitado"); return true; }
            snprintf(reply, n, "#ERROR: usa 'motor j on' o 'motor j off' con j = 1, 2 o 3"); return false;
        }
    }
    {
        int a, b, c;
        if (sscanf(l, "motors %d %d %d", &a, &b, &c) == 3) {
            int v[3] = { a, b, c };
            for (int m = 0; m < 3; m++) ctl_cmd_enable(m, v[m] != 0);
            snprintf(reply, n, "#OK: motores M1 %s, M2 %s, M3 %s", a ? "ON" : "off", b ? "ON" : "off", c ? "ON" : "off");
            return true;
        }
    }
    if (!strcmp(l, "motors") || !strcmp(l, "motor")) { cmd_motors(reply, n, "#INFO:"); return true; }
    if (!strcmp(l, "est reset")) { ctl_cmd_est_reset(); snprintf(reply, n, "#OK: observadores reiniciados"); return true; }
    if (sscanf(l, "est %d", &j) == 1) {
        if (j < 1 || j > 3) { snprintf(reply, n, "#ERROR: motor 1, 2 o 3"); return false; }
        cmd_est(j - 1, reply, n); return true;
    }
    if (!strcmp(l, "est")) { cmd_est(0, reply, n); return true; }
    if (sscanf(l, "luen %d %f %f", &j, &f[0], &f[1]) == 3) {
        if (ctl_cmd_luen(j - 1, f[0], f[1])) { snprintf(reply, n, "#OK: M%d Luenberger con polos %.3f +- %.3fj", j, f[0], fabsf(f[1])); return true; }
        snprintf(reply, n, "#ERROR: luen j re im con j = 1..3 y |polo| < 1 (ej: luen 1 0.25 0.25)"); return false;
    }
    if (sscanf(l, "kal %d %f %f %f", &j, &f[0], &f[1], &f[2]) == 4) {
        if (ctl_cmd_kal(j - 1, f[0] / 1000.0f, f[1] * RPM2RADS, f[2] * RPM2RADS)) {
            snprintf(reply, n, "#OK: M%d Kalman sig_i=%.1f mA sig_w=%.2f rpm sig_y=%.2f rpm", j, f[0], f[1], f[2]); return true;
        }
        snprintf(reply, n, "#ERROR: kal j sigma_i[mA] sigma_w[rpm] sigma_y[rpm], todos > 0 (ej: kal 1 10 0.3 0.81)"); return false;
    }
    if (sscanf(l, "vmap %f %f", &f[0], &f[1]) == 2) {
        if (ctl_cmd_vmap(f[0], f[1])) { snprintf(reply, n, "#OK: V = %.3f * (12 V * duty) - %.2f V", f[0], f[1]); return true; }
        snprintf(reply, n, "#ERROR: vmap g v0 con 0.3 < g < 1.5 y 0 <= v0 < 6 (ej: vmap 1 0.8)"); return false;
    }
    if (!strcmp(l, "gain")) {
        float a[5], b[5];
        ctl_get_gain(0, a); ctl_get_gain(1, b);
        snprintf(reply, n, "#INFO: J1 kp=%.0f ki=%.0f kd=%.0f kff=%.0f fric=%.0f | J2 kp=%.0f ki=%.0f kd=%.0f kff=%.0f fric=%.0f",
                 a[0], a[1], a[2], a[3], a[4], b[0], b[1], b[2], b[3], b[4]);
        return true;
    }
    snprintf(reply, n, "#ERROR: comando desconocido: %.40s  (usa 'help')", l);
    return false;
}
