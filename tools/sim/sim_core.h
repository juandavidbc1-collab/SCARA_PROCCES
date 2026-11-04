/*
 * Planta y sustitutos de FreeRTOS y de la capa de hardware para correr src/control.c en el PC. Lo incluyen la
 * simulación de pruebas (tools/host_tests/sim_control.c) y el simulador de ROS 2 (tools/sim/scara_sim.c), siempre
 * después de incluir src/control.c.
 *
 * Modelo: dos motores Pololu 37D (R, L, Ke, J, B de la hoja de datos) con reductor 100.37:1, fricción seca, encoder
 * cuantizado, finales de carrera y tope duro 5 grados más allá de ellos. Ejes desacoplados.
 * M3: Pololu 37D 50:1 con un tornillo que cae cuando el servo sube el destornillador y se aprieta al asentar la cabeza.
 */
#pragma once
/* ---------------- reloj simulado y stubs de FreeRTOS ---------------- */
static int64_t g_now = 0;
int64_t esp_timer_get_time(void) { return g_now; }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(g_now / 1000); }
void vTaskDelayUntil(TickType_t *p, TickType_t i) { (void)p; (void)i; }
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t f, const char *n, uint32_t s, void *a, int p, TaskHandle_t *h, int c)
{ (void)f; (void)n; (void)s; (void)a; (void)p; (void)h; (void)c; return 1; }
esp_err_t esp_task_wdt_add(void *p) { (void)p; return 0; }
esp_err_t esp_task_wdt_reset(void) { return 0; }

typedef struct { int n, sz, head, count; char *data; } q_t;
QueueHandle_t xQueueCreate(int n, int sz) { q_t *q = calloc(1, sizeof *q); q->n = n; q->sz = sz; q->data = malloc((size_t)n * sz); return q; }
BaseType_t xQueueSend(QueueHandle_t h, const void *item, TickType_t t) { (void)t; q_t *q = h; if (q->count == q->n) return 0;
    memcpy(q->data + (size_t)((q->head + q->count) % q->n) * q->sz, item, q->sz); q->count++; return 1; }
BaseType_t xQueueReceive(QueueHandle_t h, void *item, TickType_t t) { (void)t; q_t *q = h; if (!q->count) return 0;
    memcpy(item, q->data + (size_t)q->head * q->sz, q->sz); q->head = (q->head + 1) % q->n; q->count--; return 1; }

/* ---------------- planta simulada ---------------- */
typedef struct { double i, w, th; double V; int mode; /*0 libre,1 drive,2 freno*/ } mot_t;
static mot_t P[NJ];
static bool g_stby = false;
static bool g_power = true;              /* 12 V de motores (el paro físico los corta en serie) */
static int  g_tool_duty = 0, g_tool_max = 0;
typedef enum { SIM_TORNILLO_NORMAL = 0, SIM_TORNILLO_ATASCO, SIM_TORNILLO_BARRIDO, SIM_TORNILLO_NO_CAE } sim_tornillo_t;
static sim_tornillo_t g_screw_mode = SIM_TORNILLO_NORMAL;
static bool   g_m3_enc_ok = true;
static float  g_servo_deg = -1.0f;
static double g_m3_fr = 1.0, g_m3_fke = 1.0, g_m3_fkt = 1.0;   /* motor real respecto a la hoja de datos */
static double g_m3_w, g_m3_th, g_m3_i_real, g_m3_torque_real;  /* rad/s, rad, A, N·m en el eje de salida */
static double g_m3_th_tornillo = NAN;    /* ángulo de M3 cuando cayó el tornillo, que puede ser negativo; NAN: no hay tornillo */
static bool   g_m3_barrido = false;
static bool g_lock[NJ] = { false, false };   /* eje trabado (inyección de fallas) */
static const double J_carga[NJ] = { 0.04, 0.01 };      /* inercia del eslabón en el eje de salida [kg m²], estimada */
static double phys_off_deg[NJ] = { 10.0, 30.0 };       /* posición física inicial */
static double sw_min_deg[NJ] = { -125.0, -145.0 }, sw_max_deg[NJ] = { 125.0, 145.0 };
static bool enc_desconectado[NJ] = { false, false };
static int enc_sign_fisico[NJ] = { -1, +1 };            /* sentido en que cuenta el encoder respecto a la física */
static int motor_dir_fisico[NJ] = { -1, -1 };           /* sentido físico del motor con duty>0 (antes de motor_sign): el cableado real necesita motor_sign = -1 */
static int32_t enc_frozen[NJ];

#define NGEAR 100.37
static double phys_deg(int j) { return P[j].th / NGEAR / (2 * M_PI) * 360.0 + phys_off_deg[j]; }

void hal_stby(bool on) { g_stby = on; }
bool hal_stby_get(void) { return g_stby; }
void hal_motor_free(int j) { P[j].mode = 0; P[j].V = 0; }
void hal_motor_brake(int j) { P[j].mode = 2; P[j].V = 0; }
void hal_all_free(void) { for (int j = 0; j < NJ; j++) hal_motor_free(j); }
void hal_motor_drive(int j, int duty)
{
    duty *= JC[j].motor_sign;
    if (duty == 0) { hal_motor_free(j); return; }
    if (abs(duty) > PWM_MAX) duty = duty > 0 ? PWM_MAX : -PWM_MAX;
    P[j].mode = 1; P[j].V = duty / 1023.0 * 12.0 * motor_dir_fisico[j];
}
int32_t hal_enc_raw(int j)
{
    if (enc_desconectado[j]) return enc_frozen[j];
    return (int32_t)floor(P[j].th / NGEAR * JC[j].k) * enc_sign_fisico[j];   /* q del firmware = ángulo físico */
}
int hal_enc_levels(int j) { (void)j; return 0; }
bool hal_fc_raw(int j, int side)
{
    double a = phys_deg(j);
    return side ? (a >= sw_max_deg[j]) : (a <= sw_min_deg[j]);
}
void hal_tool_drive(int duty) { g_tool_duty = duty; if (abs(duty) > g_tool_max) g_tool_max = abs(duty); }
void hal_tool_free(void) { g_tool_duty = 0; }
int32_t hal_tool_enc_raw(void)
{
    return g_m3_enc_ok ? (int32_t)floor(g_m3_th / (2 * M_PI) * TOOL_ENC_CUENTAS) * TOOL_ENC_SIGN : 0;
}
void hal_servo_set_deg(float deg)
{
    /* Al subir el destornillador, cae un tornillo nuevo sobre el hueco; tiene que llegar a M3 antes de atornillar. */
    const float mitad = fabsf(SERVO_ARRIBA_DEG - SERVO_ABAJO_DEG) / 2;
    if (fabsf(deg - SERVO_ABAJO_DEG) > mitad && fabsf(g_servo_deg - SERVO_ABAJO_DEG) <= mitad) {
        g_m3_th_tornillo = g_screw_mode == SIM_TORNILLO_NO_CAE ? NAN : g_m3_th;
        g_m3_barrido = false;
    }
    g_servo_deg = deg;
}
void hal_init(void) {}

static double g_phys_max_exceso = 0;     /* cuánto se pasó de los límites duros (grados) */

/* Torque que el tornillo opone a M3 [N·m]. La rosca cuesta poco hasta que asienta la cabeza; ahí sube como un resorte. */
static double m3_par_tornillo(void)
{
    if (isnan(g_m3_th_tornillo)) return 0.0;
    const double vueltas = (g_m3_th - g_m3_th_tornillo) / (2 * M_PI);
    if (vueltas < 0.0) return 0.0;
    if (g_m3_barrido) return 0.005;
    const double asiento = g_screw_mode == SIM_TORNILLO_ATASCO ? 3.0 : 12.0;   /* tornillo de 6 mm */
    const double par = M3_MODEL.torque_rosca + (vueltas > asiento ? 0.32 * (vueltas - asiento) * 2 * M_PI : 0.0);
    if (g_screw_mode == SIM_TORNILLO_BARRIDO && par > 0.25) { g_m3_barrido = true; return 0.005; }
    return par;
}

static void m3_planta_1ms(void)
{
    const double R = M3_MODEL.r * g_m3_fr, Ke = M3_MODEL.ke * g_m3_fke, Kt = M3_MODEL.kt * g_m3_fkt;
    const double J = 0.0049, dt = 1e-4;   /* inercia en el eje de salida: constante mecánica de unos 50 ms, estimada */
    const double Tf = Kt * M3_MODEL.i0;
    const bool activo = g_stby && g_power && g_tool_duty != 0;
    const double V = activo ? g_tool_duty / 1023.0 * 12.0 : 0.0;   /* la planta supone los signos bien puestos: duty > 0 aprieta */
    if (V < 0.0) g_m3_th_tornillo = NAN;   /* supuesto sin medir: al girar al revés, la punta se suelta sola del tornillo */
    for (int k = 0; k < 10; k++) {
        const double par = m3_par_tornillo();
        g_m3_i_real = activo ? (V - Ke * g_m3_w) / R : 0.0;
        const double Tm = Kt * g_m3_i_real;
        const double resiste = Tf + par;
        double a;
        if (fabs(g_m3_w) < 1e-3 && fabs(Tm) <= resiste) { g_m3_w = 0.0; a = 0.0; }   /* calado o quieto */
        else a = (Tm - copysign(resiste, g_m3_w != 0.0 ? g_m3_w : Tm)) / J;
        double wn = g_m3_w + a * dt;
        if ((g_m3_w > 0.0 && wn < 0.0) || (g_m3_w < 0.0 && wn > 0.0)) wn = 0.0;
        g_m3_w = wn;
        g_m3_th += g_m3_w * dt;
        if (g_m3_th < -2 * M_PI * 0.66) { g_m3_th = -2 * M_PI * 0.66; g_m3_w = 0.0; }   /* tope físico de arriba, medido en el robot */
        g_m3_torque_real = par;
    }
}
static void planta_1ms(void)
{
    const double R = 2.4, L = 1.5e-3, Ke = 0.01056, B = 2.97e-6, Tc = 3e-3;
    const double dt = 1e-4;
    m3_planta_1ms();
    for (int j = 0; j < NJ; j++) {
        mot_t *m = &P[j];
        const double J = 3e-6 + J_carga[j] / (NGEAR * NGEAR);
        double V = ((g_stby && g_power) ? m->V : 0.0);
        bool abierto = (!g_stby || m->mode == 0);
        for (int k = 0; k < 10; k++) {
            if (g_lock[j]) { m->w = 0; m->i = 0; continue; }
            if (abierto) m->i = 0; else m->i += (V - R * m->i - Ke * m->w) / L * dt;
            double Tl = Tc * (m->w > 0.5 ? 1 : (m->w < -0.5 ? -1 : m->w / 0.5));
            m->w += (Ke * m->i - B * m->w - Tl) / J * dt;
            m->th += m->w * dt;
            double a = phys_deg(j);                    /* tope mecánico duro 5° más allá del final de carrera */
            if (a < sw_min_deg[j] - 5 || a > sw_max_deg[j] + 5) {
                double lim = a < 0 ? sw_min_deg[j] - 5 : sw_max_deg[j] + 5;
                m->th = (lim - phys_off_deg[j]) / 360.0 * 2 * M_PI * NGEAR; m->w = 0;
            }
            double ex = fmax(sw_min_deg[j] - a, a - sw_max_deg[j]); if (ex > g_phys_max_exceso) g_phys_max_exceso = ex;
        }
    }
}

