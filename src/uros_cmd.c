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
    snprintf(reply, n, "#INFO: [%s] q1=%.2f q2=%.2f deg x=%.1f y=%.1f mm home J1:%d J2:%d motores J1:%d J2:%d M3:%d falla:%s",
             ctl_mode_str(s.mode), s.q[0] * RAD2DEG, s.q[1] * RAD2DEG, x, y, s.homed[0], s.homed[1],
             s.en & 1, (s.en >> 1) & 1, (s.en >> 2) & 1,
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

    float f[6]; int j;
    if (!strncmp(l, "m3 ", 3)) {
        const char *a = l + 3;
        bool ok;
        if (!strcmp(a, "home")) ok = ctl_cmd_m3_home();
        else if (!strcmp(a, "subir")) ok = ctl_cmd_m3_ir(M3_VUELTAS_ARRIBA);
        else if (!strcmp(a, "bajar")) ok = ctl_cmd_m3_ir(M3_VUELTAS_ATORNILLAR);
        else if (!strcmp(a, "trabajo")) ok = ctl_cmd_m3_ir(M3_VUELTAS_TRABAJO);
        else if (sscanf(a, "ir %f", &f[0]) == 1) ok = ctl_cmd_m3_ir(f[0]);
        else if (!strcmp(a, "estado")) {
            ctl_status_t s; ctl_get_status(&s);
            if (s.m3_ref) snprintf(reply, n, "#INFO: M3 a %.3f vueltas desde arriba, altura %.1f mm (limites %.0f a %.0f mm)", s.m3_vueltas,
                                   M3_ALTURA_ARRIBA_MM - s.m3_vueltas * M3_PASO_MM, M3_ALTURA_ABAJO_MM, M3_ALTURA_ARRIBA_MM);
            else snprintf(reply, n, "#INFO: M3 sin referencia: usa m3 home");
            return true;
        } else { snprintf(reply, n, "#ERROR: usa m3 home | subir | trabajo | bajar | ir v | estado"); return false; }
        if (ok) { snprintf(reply, n, "#OK: M3: %s", a); return true; }
        snprintf(reply, n, "#ERROR: M3 rechazado (de 0 a %.2f vueltas)", M3_VUELTAS_ATORNILLAR); return false;
    }
    if (!strcmp(l, "home")) {
        if (ctl_cmd_home()) { snprintf(reply, n, "#OK: home solicitado"); return true; }
        snprintf(reply, n, "#ERROR: home rechazado (falla activa o movimiento en curso)"); return false;
    }
    if (!strcmp(l, "stop"))  { ctl_cmd_stop();  snprintf(reply, n, "#OK: detenido (mantiene posicion)"); return true; }
    if (!strcmp(l, "off"))   { ctl_cmd_off();   snprintf(reply, n, "#OK: motores apagados"); return true; }
    if (!strcmp(l, "clear")) { ctl_cmd_clear(); snprintf(reply, n, "#OK: falla borrada"); return true; }
    if (!strcmp(l, "status")) { cmd_status(reply, n); return true; }
    if (!strcmp(l, "estop")) { ctl_cmd_estop(); snprintf(reply, n, "#OK: paro de emergencia solicitado"); return true; }
    {
        char w[8];
        if (sscanf(l, "motor %d %7s", &j, w) == 2) {
            bool on = !strcmp(w, "on");
            if ((on || !strcmp(w, "off")) && j >= 1 && j <= NJ + 1 && ctl_cmd_motor(j - 1, on)) {
                snprintf(reply, n, "#OK: motor %d %s", j, on ? "activar" : "desactivar");
                return true;
            }
            snprintf(reply, n, "#ERROR: usa motor 1|2|3 on|off"); return false;
        }
        if (sscanf(l, "ctrl %7s", w) == 1) {
            bool pid = !strcmp(w, "pid");
            if ((pid || !strcmp(w, "rst")) && ctl_cmd_ctrl(pid ? CTRL_PID : CTRL_RST)) { snprintf(reply, n, "#OK: cambio a %s solicitado", pid ? "PID" : "RST"); return true; }
            snprintf(reply, n, "#ERROR: usa ctrl pid|rst"); return false;
        }
        if (sscanf(l, "rstform %7s", w) == 1) {
            int fm = !strcmp(w, "pos") ? RST_FORM_POS : !strcmp(w, "inc") ? RST_FORM_INC : !strcmp(w, "pi_d") ? RST_FORM_PI_D : !strcmp(w, "mask") ? RST_FORM_MASK : -1;
            if (fm >= 0 && ctl_cmd_rstform((rst_form_t)fm)) { snprintf(reply, n, "#OK: forma %s solicitada", RST_FORM_NAME[fm]); return true; }
            snprintf(reply, n, "#ERROR: usa rstform pos|inc|pi_d|mask"); return false;
        }
    }
    if (!strcmp(l, "servo subir") || !strcmp(l, "servo bajar") || sscanf(l, "servo %f", &f[0]) == 1) {
        float deg = !strcmp(l, "servo subir") ? SERVO_ARRIBA_DEG : !strcmp(l, "servo bajar") ? SERVO_ABAJO_DEG : f[0];
        if (ctl_cmd_servo(deg)) { snprintf(reply, n, "#OK: servo a %.0f grados", deg); return true; }
        snprintf(reply, n, "#ERROR: servo va de %.0f a %.0f grados (o subir / bajar)", SERVO_GRADOS_MIN, SERVO_GRADOS_MAX); return false;
    }
    if (!strcmp(l, "screw") || sscanf(l, "screw %f %f", &f[0], &f[1]) == 2) {
        float v = !strcmp(l, "screw") ? 0.0f : f[0], t = !strcmp(l, "screw") ? 0.0f : f[1];
        if (ctl_cmd_screw(v, t)) { snprintf(reply, n, "#OK: atornillado solicitado"); return true; }
        snprintf(reply, n, "#ERROR: usa screw [vueltas 0-40, torque 0-1.5 Nm]"); return false;
    }
    if (sscanf(l, "tool %f", &f[0]) == 1) {
        if (ctl_cmd_tool(f[0])) { snprintf(reply, n, "#OK: M3 a %.0f %%", f[0]); return true; }
        snprintf(reply, n, "#ERROR: tool va de -100 a 100 %%"); return false;
    }
    {
        float g[7];
        if (sscanf(l, "rst %d %f %f %f %f %f %f %f", &j, &g[0], &g[1], &g[2], &g[3], &g[4], &g[5], &g[6]) == 8) {
            if (ctl_cmd_rst(j - 1, g)) { snprintf(reply, n, "#OK: RST del eje %d actualizado", j); return true; }
            snprintf(reply, n, "#ERROR: RST fuera de rango o eje invalido (eje 1 o 2)"); return false;
        }
    }
    if (!strcmp(l, "rst")) {
        float a[7], b[7];
        ctl_get_rst(0, a); ctl_get_rst(1, b);
        snprintf(reply, n, "#RST: J1 kp=%.1f ki=%.1f kd=%.2f s1=%.4f b=%.2f c=%.2f kff=%.1f | J2 kp=%.1f ki=%.1f kd=%.2f s1=%.4f b=%.2f c=%.2f kff=%.1f",
                 a[0], a[1], a[2], a[3], a[4], a[5], a[6], b[0], b[1], b[2], b[3], b[4], b[5], b[6]);
        return true;
    }
    if (!strcmp(l, "help")) {
        snprintf(reply, n, "#INFO: home stop off clear estop goto ang elbow pulse status | motor n on/off | tool pct | m3 home/subir/trabajo/bajar/ir/estado | servo g/subir/bajar | ctrl pid/rst | rstform pos/inc/pi_d/mask | rst | gain");
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
