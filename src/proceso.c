#include "proceso.h"
#include <stdio.h>
#include <string.h>
#include "control.h"
#include "esp_timer.h"
#include "motion.h"

typedef enum { P_GOTO, P_ANG, P_ESPERA, P_SERVO, P_M3 } tipo_t;

typedef struct {
    tipo_t tipo;
    float a, b;           /* P_GOTO: x, y [mm]. P_ESPERA: a [s]. P_SERVO: a [grados]. P_M3: a [vueltas desde arriba] */
    int hueco;            /* índice en HUECOS, o -1 */
    const char *txt;
} paso_t;

typedef struct { const char *nombre; float x, y; } hueco_t;

/* 4 placas de 38 mm en una fila recta, a x = 170 mm, con 30 mm entre ellas y huecos a 3,5 mm del borde, en mm desde la base
 * (+x: brazo recto; +y: izquierda). Centros: A (170; 102), B (170; 34), C (170; -34), D (170; -102).
 * Orden: los dos huecos de arriba y luego los dos de abajo. Con codo -1: q1 de -17 a 67 grados, q2 de -95 a -35. */
static const hueco_t HUECOS[] = {
    { "A-H2", 154.500f, 117.500f }, { "A-H4", 185.500f, 117.500f }, { "A-H3", 185.500f, 86.500f }, { "A-H1", 154.500f, 86.500f },
    { "B-H2", 154.500f, 49.500f }, { "B-H4", 185.500f, 49.500f }, { "B-H3", 185.500f, 18.500f }, { "B-H1", 154.500f, 18.500f },
    { "C-H2", 154.500f, -18.500f }, { "C-H4", 185.500f, -18.500f }, { "C-H3", 185.500f, -49.500f }, { "C-H1", 154.500f, -49.500f },
    { "D-H2", 154.500f, -86.500f }, { "D-H4", 185.500f, -86.500f }, { "D-H3", 185.500f, -117.500f }, { "D-H1", 154.500f, -117.500f },
};
#define N_HUECOS   ((int)(sizeof HUECOS / sizeof HUECOS[0]))
#define CODO       (-1)
#define HUECOS_POR_PCB 4
#define MAX_PASOS  (4 + N_HUECOS * 11)

static paso_t pasos[MAX_PASOS];
static int n_pasos, idx, ciclos, repeticiones;
static bool activo, llego, servo_arriba;
static int64_t t_paso;

static void agregar(tipo_t tipo, float a, float b, int hueco, const char *txt)
{
    pasos[n_pasos++] = (paso_t){ tipo, a, b, hueco, txt };
}

static void armar(bool ensayo, int pcb)
{
    n_pasos = 0;
    for (int h = 0; h < N_HUECOS; h++) {
        if (pcb && h / HUECOS_POR_PCB != pcb - 1) continue;
        agregar(P_GOTO, HUECOS[h].x, HUECOS[h].y, h, "Ir al hueco");
        agregar(P_ESPERA, PROC_ASENTAR_S, 0, h, "Asentar");
        if (ensayo) {
            agregar(P_M3, M3_ENSAYO_VUELTAS, 0, h, "Bajar M3");
            agregar(P_ESPERA, PROC_ASENTAR_S, 0, h, "Pausa abajo");
        } else {
            /* La secuencia del robot: sube el servo, baja M3, baja el servo y M3 sube 1/4 de la barra. El servo no para en el medio. */
            agregar(P_SERVO, SERVO_ARRIBA_DEG, 0, h, "Subir el servo");
            agregar(P_ESPERA, PROC_SERVO_ARRIBA_S, 0, h, "Servo subiendo");
            agregar(P_M3, M3_VUELTAS_ATORNILLAR, 0, h, "Bajar M3");
            agregar(P_ESPERA, PROC_ASENTAR_S, 0, h, "Pausa abajo");
            agregar(P_SERVO, SERVO_ABAJO_DEG, 0, h, "Bajar el servo");
            agregar(P_ESPERA, PROC_SERVO_BAJAR_S, 0, h, "Servo bajando");
        }
        agregar(P_M3, M3_VUELTAS_TRABAJO, 0, h, "Subir M3 1/4 de la barra");
    }
    agregar(P_ANG, 0, 0, -1, "Volver a 0 0");
}

static void imprimir_paso(int i)
{
    const paso_t *p = &pasos[i];
    printf("[%3d/%3d] %s%s%s", i + 1, n_pasos, p->hueco >= 0 ? HUECOS[p->hueco].nombre : "", p->hueco >= 0 ? ": " : "", p->txt);
    switch (p->tipo) {
    case P_GOTO:   printf(" (%.1f, %.1f mm)", p->a, p->b); break;
    case P_ANG:    printf(" (%.0f, %.0f grados)", p->a, p->b); break;
    case P_ESPERA: printf(" (%.1f s)", p->a); break;
    case P_SERVO:  printf(" (%.0f grados)", p->a); break;
    case P_M3:     printf(" (a %.1f vueltas desde arriba)", p->a); break;
    default: break;
    }
    printf("\n");
}

static void cortar(const char *motivo, bool enviar_stop)
{
    if (!activo) return;
    activo = false;
    if (enviar_stop) ctl_cmd_stop();     /* detiene el brazo y cancela el tornillo */
    if (pasos[idx].tipo == P_M3) ctl_cmd_tool(0.0f);      /* también cancela la ida de M3 */
    if (servo_arriba) { ctl_cmd_servo(SERVO_ABAJO_DEG); servo_arriba = false; }
    printf("\n#PROCESO: detenido en el paso %d: %s\n", idx + 1, motivo);
    ctl_status_t s; ctl_get_status(&s);
    if (s.m3_ref && s.m3_vueltas > M3_VUELTAS_TRASLADO) printf("#PROCESO: M3 quedo abajo. Revisa el hueco y usa: m3 subir\n");
}

static void empezar_paso(void)
{
    const paso_t *p = &pasos[idx];
    char m[200];
    llego = false;
    t_paso = esp_timer_get_time();
    imprimir_paso(idx);
    switch (p->tipo) {
    case P_GOTO:
        if (motion_goto_xyz(p->a, p->b, Z_FIJO_MM, m, sizeof m) != MOTION_OK) cortar(m, false);
        break;
    case P_ANG:
        if (motion_goto_ang(p->a, p->b, m, sizeof m) != MOTION_OK) cortar(m, false);
        break;
    case P_M3:
        if (!ctl_cmd_m3_ir(p->a)) cortar("M3 rechazo la altura", false);
        break;
    case P_SERVO:
        if (!ctl_cmd_servo(p->a)) cortar("el servo rechazo el angulo", false);
        else servo_arriba = p->a == SERVO_ARRIBA_DEG;
        break;
    default:
        break;
    }
}

static void siguiente_paso(void)
{
    if (++idx < n_pasos) { empezar_paso(); return; }
    ciclos++;
    if (repeticiones && ciclos >= repeticiones) {
        activo = false;
        printf("\n#PROCESO: terminado.\n");
        return;
    }
    idx = 0;
    empezar_paso();
}

static bool arrancar(int rep)
{
    idx = ciclos = 0;
    repeticiones = rep;
    servo_arriba = false;
    activo = true;
    empezar_paso();
    return activo;
}

bool proceso_iniciar(bool ensayo, int pcb, int rep)
{
    if (activo) { printf("#ERROR: ya hay un proceso en marcha.\n"); return false; }
    ctl_status_t s; ctl_get_status(&s);
    if (s.mode == M_FAULT) { printf("#ERROR: hay una falla activa. Revisa el robot y borrala con 0.\n"); return false; }
    if (!s.homed[0] || !s.homed[1]) { printf("#ERROR: primero haz Home del brazo (tecla 5).\n"); return false; }
    if (!s.m3_ref) { printf("#ERROR: primero busca la referencia de M3: m3 home\n"); return false; }
    if (s.m3_vueltas > M3_VUELTAS_TRASLADO) { printf("#ERROR: M3 esta abajo. Usa: m3 subir\n"); return false; }
    ctl_cmd_motor(2, true);
    motion_set_elbow(CODO);
    armar(ensayo, pcb);
    if (pcb) printf("#PROCESO: %s de la PCB %d (%c). Cualquier tecla lo detiene.\n", ensayo ? "ensayo" : "proceso", pcb, 'A' + pcb - 1);
    else printf("#PROCESO: %s de las 4 PCB, %d huecos. Cualquier tecla lo detiene.\n", ensayo ? "ensayo" : "proceso", N_HUECOS);
    return arrancar(rep);
}

void proceso_detener(void) { cortar("detenido por tecla", true); }

bool proceso_activo(void) { return activo; }

void proceso_listar(bool ensayo, int pcb)
{
    if (activo) { printf("#ERROR: hay un proceso en marcha.\n"); return; }
    armar(ensayo, pcb);
    for (int i = 0; i < n_pasos; i++) imprimir_paso(i);
}

void proceso_evento(const char *ev)
{
    if (!activo) return;
    if (!strncmp(ev, "#LLEGO", 6) || !strncmp(ev, "#M3", 3)) {
        llego = true;
    } else if (!strncmp(ev, "#FALLA", 6) || !strncmp(ev, "#ERROR", 6)) {
        cortar(ev, false);
    }
}

void proceso_tick(void)
{
    if (!activo) return;
    const paso_t *p = &pasos[idx];
    const int64_t ahora = esp_timer_get_time();
    const float t = (float)(ahora - t_paso) / 1e6f;
    switch (p->tipo) {
    case P_GOTO:
    case P_ANG:
        if (llego) siguiente_paso();
        else if (t > PROC_LIMITE_BRAZO_S) cortar("el brazo no llego a tiempo", true);
        break;
    case P_ESPERA:
        if (t >= p->a) siguiente_paso();
        break;
    case P_SERVO:
        siguiente_paso();
        break;
    case P_M3:
        if (llego) siguiente_paso();
        else if (t > PROC_LIMITE_M3_S) cortar("M3 no llego a tiempo", true);
        break;
    }
}
