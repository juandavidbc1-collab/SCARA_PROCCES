/* Prueba en el PC del intérprete de comandos de ROS 2 (src/uros_cmd.c) y de src/motion.c, con un control falso. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "control.h"
#include "kinematics.h"
#include "motion.h"
#include "uros_cmd.h"

static int n_home, n_stop, n_off, n_clear, n_move, n_pulse, n_gain, n_est_reset;
static uint8_t g_en = 0x3;
static int last_luen_m = -1, last_kal_m = -1; static float last_kal[3], last_vmap[2];
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
const char *ctl_mode_str(ctl_mode_t m) { static const char *s[] = { "OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT", "POWER" }; return s[m]; }
uint8_t ctl_get_enabled(void) { return g_en; }
bool ctl_cmd_enable(int m, bool on) { if (m < 0 || m > 2) return false; if (on) g_en |= 1u << m; else g_en &= ~(1u << m); return true; }
void ctl_cmd_est_reset(void) { n_est_reset++; }
bool ctl_cmd_luen(int m, float re, float im) { if (m < 0 || m > 2 || sqrtf(re * re + im * im) >= 0.999f) return false; last_luen_m = m; return true; }
bool ctl_cmd_kal(int m, float a, float b, float c) { if (m < 0 || m > 2 || !(a > 0 && b > 0 && c > 0)) return false; last_kal_m = m; last_kal[0] = a; last_kal[1] = b; last_kal[2] = c; return true; }
bool ctl_cmd_vmap(float g, float v0) { if (!(g > 0.3f && g < 1.5f) || !(v0 >= 0 && v0 < 6)) return false; last_vmap[0] = g; last_vmap[1] = v0; return true; }
void ctl_get_est(int m, est_params_t *p, float L[2], bool *ok) { (void)m; memset(p, 0, sizeof *p); p->polo_re = 0.25f; p->polo_im = 0.25f; L[0] = 0.24f; L[1] = 0.41f; *ok = true; }
void ctl_get_vmap(float *g, float *v0) { *g = 1.0f; *v0 = 0.8f; }
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

    /* ---- motores ---- */
    ok = uros_cmd_exec("motor 3 on", r, sizeof r);    CHECK(ok && g_en == 0x7, "motor 3 on");
    ok = uros_cmd_exec("motor 2 off", r, sizeof r);   CHECK(ok && g_en == 0x5, "motor 2 off");
    ok = uros_cmd_exec("motors", r, sizeof r);        CHECK(ok && strstr(r, "M2 off") && strstr(r, "M3 ON"), "motors muestra cuales estan activos");
    int nm = n_move;
    ok = uros_cmd_exec("goto 120 80", r, sizeof r);   CHECK(!ok && n_move == nm && strstr(r, "habilitados"), "goto con un eje deshabilitado se rechaza");
    ok = uros_cmd_exec("ang 10 20", r, sizeof r);     CHECK(ok && n_move == nm + 1 && strstr(r, "solo J1"), "ang con J2 deshabilitado mueve solo J1");
    ok = uros_cmd_exec("motors 1 1 0", r, sizeof r);  CHECK(ok && g_en == 0x3, "motors 1 1 0");
    ok = uros_cmd_exec("motor 4 on", r, sizeof r);    CHECK(!ok, "motor 4 se rechaza");
    ok = uros_cmd_exec("motor 1 talvez", r, sizeof r); CHECK(!ok && g_en == 0x3, "motor 1 <basura> se rechaza");
    ok = uros_cmd_exec("status", r, sizeof r);        CHECK(ok && strstr(r, "motores:"), "status incluye los motores");

    /* ---- observadores ---- */
    ok = uros_cmd_exec("est reset", r, sizeof r);     CHECK(ok && n_est_reset == 1, "est reset");
    ok = uros_cmd_exec("est 2", r, sizeof r);         CHECK(ok && strstr(r, "M2") && strstr(r, "L=["), "est 2 muestra el diseno");
    ok = uros_cmd_exec("est 5", r, sizeof r);         CHECK(!ok, "est 5 se rechaza");
    ok = uros_cmd_exec("luen 3 0.1 0.2", r, sizeof r); CHECK(ok && last_luen_m == 2, "luen 3 0.1 0.2");
    ok = uros_cmd_exec("luen 1 0.9 0.9", r, sizeof r); CHECK(!ok, "luen con |polo| >= 1 se rechaza");
    ok = uros_cmd_exec("kal 1 10 0.3 0.81", r, sizeof r);
    CHECK(ok && last_kal_m == 0 && fabsf(last_kal[0] - 0.010f) < 1e-6f && fabsf(last_kal[2] - 0.81f * (float)M_PI / 30) < 1e-5f,
          "kal 1 10 0.3 0.81 (mA y rpm -> A y rad/s)");
    ok = uros_cmd_exec("kal 1 0 0.3 0.81", r, sizeof r); CHECK(!ok, "kal con sigma 0 se rechaza");
    ok = uros_cmd_exec("vmap 0.978 1.197", r, sizeof r); CHECK(ok && fabsf(last_vmap[1] - 1.197f) < 1e-6f, "vmap del L298N");
    ok = uros_cmd_exec("vmap 3 0", r, sizeof r);      CHECK(!ok, "vmap fuera de rango se rechaza");
    ok = uros_cmd_exec("help", r, sizeof r);          CHECK(ok && strlen(r) < 200 && strstr(r, "motor"), "help cabe en el evento y lista los comandos nuevos");

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
