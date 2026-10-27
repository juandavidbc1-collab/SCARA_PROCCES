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

static void cmd_status(char *reply, size_t n)
{
    ctl_status_t s; ctl_get_status(&s);
    float x, y; kin_fk(&KIN, s.q[0], s.q[1], &x, &y);
    snprintf(reply, n, "#INFO: [%s] q1=%.2f q2=%.2f deg x=%.1f y=%.1f mm home J1:%d J2:%d falla:%s",
             ctl_mode_str(s.mode), s.q[0] * RAD2DEG, s.q[1] * RAD2DEG, x, y, s.homed[0], s.homed[1],
             s.fault == F_NONE ? "ninguna" : ctl_fault_str(s.fault));
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
        snprintf(reply, n, "#INFO: home stop off clear | goto x y [z] | ang q1 q2 | elbow +1/-1 | pulse 1/2 | gain [j kp ki kd kff fric] | status");
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
