/* Prueba en el PC del intérprete de comandos de ROS 2 (src/uros_cmd.c) y de src/motion.c, con un control falso. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "control.h"
#include "kinematics.h"
#include "motion.h"
#include "uros_cmd.h"

static int n_home, n_stop, n_off, n_clear, n_move, n_pulse, n_gain;
static bool accept_move = true;
static float last_q[2], last_tgt[3];
static int last_gain_j;

void ctl_get_status(ctl_status_t *s) { memset(s, 0, sizeof *s); s->mode = M_HOLD; s->homed[0] = s->homed[1] = true; }
void ctl_get_gain(int j, float o[5]) { (void)j; for (int i = 0; i < 5; i++) o[i] = (float)(i + 1); }
bool ctl_cmd_home(void) { n_home++; return true; }
void ctl_cmd_stop(void) { n_stop++; }
void ctl_cmd_off(void) { n_off++; }
void ctl_cmd_clear(void) { n_clear++; }
bool ctl_cmd_pulse(int j) { n_pulse++; return j == 0 || j == 1; }
bool ctl_cmd_move(const float q[2], const float t[3])
{
    if (!accept_move) return false;
    n_move++; last_q[0] = q[0]; last_q[1] = q[1];
    for (int i = 0; i < 3; i++) last_tgt[i] = t[i];
    return true;
}
bool ctl_cmd_gain(int j, float a, float b, float c, float d, float e)
{ (void)a; (void)b; (void)c; (void)d; (void)e; n_gain++; last_gain_j = j; return j == 0 || j == 1; }
void ctl_get_limits(int j, float *lo, float *hi) { *lo = JC[j].q_min; *hi = JC[j].q_max; }
const char *ctl_mode_str(ctl_mode_t m) { static const char *s[] = { "OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT" }; return s[m]; }
const char *ctl_fault_str(fault_t f) { (void)f; return "x"; }

static int fallos = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("  >>> FALLO: %s\n", msg); fallos++; } else printf("  [ok] %s\n", msg); } while (0)

int main(void)
{
    char r[240];
    bool ok;

    ok = uros_cmd_exec("home", r, sizeof r);          CHECK(ok && n_home == 1 && !strncmp(r, "#OK", 3), "home");
    ok = uros_cmd_exec("  STOP \n", r, sizeof r);     CHECK(ok && n_stop == 1, "stop (mayusculas, espacios y salto de linea)");
    ok = uros_cmd_exec("off", r, sizeof r);           CHECK(ok && n_off == 1, "off");
    ok = uros_cmd_exec("clear", r, sizeof r);         CHECK(ok && n_clear == 1, "clear");

    ok = uros_cmd_exec("goto 120 80 0", r, sizeof r);
    CHECK(ok && n_move == 1 && !strncmp(r, "#INFO", 5), "goto 120 80 0 acepta y encola");
    {   /* la cinematica directa de lo encolado debe dar el punto pedido */
        float x, y; kin_fk(&KIN, last_q[0], last_q[1], &x, &y);
        CHECK(fabsf(x - 120.0f) < 0.01f && fabsf(y - 80.0f) < 0.01f, "IK->FK devuelve (120, 80)");
        CHECK(last_tgt[0] == 120.0f && last_tgt[1] == 80.0f && last_tgt[2] == 0.0f, "el objetivo xyz se pasa al control");
    }
    ok = uros_cmd_exec("goto 100 -60", r, sizeof r);  CHECK(ok && n_move == 2, "goto sin z usa la altura fija");
    ok = uros_cmd_exec("goto 100 -60 40", r, sizeof r); CHECK(!ok && n_move == 2 && strstr(r, "z = 40.0"), "z fuera de la altura fija se rechaza");
    ok = uros_cmd_exec("goto 400 0", r, sizeof r);    CHECK(!ok && n_move == 2 && strstr(r, "Alcance"), "punto inalcanzable se rechaza e informa el alcance");
    ok = uros_cmd_exec("goto 190 0", r, sizeof r);    CHECK(!ok && n_move == 2, "singularidad (brazo extendido) se rechaza");
    ok = uros_cmd_exec("goto 10 0", r, sizeof r);     CHECK(!ok && n_move == 2, "punto interior inalcanzable se rechaza");

    accept_move = false;
    ok = uros_cmd_exec("goto 120 80", r, sizeof r);   CHECK(!ok && strstr(r, "cola"), "cola llena se informa");
    accept_move = true;

    ok = uros_cmd_exec("ang 20 90", r, sizeof r);     CHECK(ok && fabsf(last_q[0] - 20 * (float)M_PI / 180) < 1e-5f && n_move == 3, "ang 20 90");
    ok = uros_cmd_exec("ang 500 0", r, sizeof r);     CHECK(!ok && n_move == 3 && strstr(r, "fuera de sus limites"), "ang fuera de limites se rechaza");

    ok = uros_cmd_exec("elbow -1", r, sizeof r);      CHECK(ok && motion_get_elbow() == -1, "elbow -1");
    ok = uros_cmd_exec("goto 120 80", r, sizeof r);
    {
        CHECK(ok && last_q[1] < 0, "con codo -1 la solucion tiene q2 < 0");
    }
    ok = uros_cmd_exec("elbow +1", r, sizeof r);      CHECK(ok && motion_get_elbow() == 1, "elbow +1");

    ok = uros_cmd_exec("pulse 2", r, sizeof r);       CHECK(ok && n_pulse == 1, "pulse 2");
    ok = uros_cmd_exec("pulse 3", r, sizeof r);       CHECK(!ok, "pulse 3 se rechaza");
    ok = uros_cmd_exec("gain 2 5000 4500 180 80 60", r, sizeof r); CHECK(ok && n_gain == 1 && last_gain_j == 1, "gain 2 ... (eje 2 -> indice 1)");
    ok = uros_cmd_exec("gain", r, sizeof r);          CHECK(ok && strstr(r, "J1") && strstr(r, "J2"), "gain muestra ambos ejes");
    ok = uros_cmd_exec("status", r, sizeof r);        CHECK(ok && strstr(r, "HOLD"), "status");
    ok = uros_cmd_exec("help", r, sizeof r);          CHECK(ok, "help");
    ok = uros_cmd_exec("baila", r, sizeof r);         CHECK(!ok && strstr(r, "desconocido"), "comando desconocido");
    ok = uros_cmd_exec("", r, sizeof r);              CHECK(!ok, "linea vacia");

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
