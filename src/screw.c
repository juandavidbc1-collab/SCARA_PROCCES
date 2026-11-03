#include "screw.h"
#include <math.h>
#include <string.h>

float screw_corriente(const screw_model_t *m, float duty, float w)
{
    return (duty / m->pwm_max * m->vbus - m->ke * w) / m->r;
}

screw_est_t screw_estimar(const screw_model_t *m, float duty, float w)
{
    screw_est_t e;
    e.i = screw_corriente(m, duty, w);
    e.torque = m->kt * (e.i - m->i0);
    if (e.torque < 0.0f) e.torque = 0.0f;
    float apriete = e.torque - m->torque_rosca;
    if (apriete < 0.0f) apriete = 0.0f;
    e.fuerza = apriete / (m->k_tuerca * m->d_mm * 1e-3f);
    e.presion = e.fuerza / m->area_mm2;
    return e;
}

float screw_profundidad_mm(const screw_model_t *m, float vueltas) { return vueltas * m->paso_mm; }

float screw_duty_para_torque(const screw_model_t *m, float torque)
{
    return (torque / m->kt + m->i0) * m->r / m->vbus * m->pwm_max;
}

void screw_start(screw_t *s, const screw_model_t *m, const screw_cfg_t *cfg)
{
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    s->fase = SCREW_RAPIDO;
    s->duty = cfg->duty_rapido;
    s->duty_apriete = screw_duty_para_torque(m, cfg->torque_meta);
}

bool screw_activo(const screw_t *s) { return s->fase != SCREW_IDLE && s->fase != SCREW_FIN; }

static void terminar(screw_t *s, screw_result_t r)
{
    s->fase = SCREW_FIN;
    s->res = r;
    s->duty = 0.0f;
}

void screw_cancel(screw_t *s)
{
    if (screw_activo(s)) terminar(s, SCREW_R_CANCELADO);
    s->duty = 0.0f;
}

static void cambiar(screw_t *s, screw_phase_t f, float duty)
{
    s->fase = f;
    s->duty = duty;
    s->t_fase = s->t;
    s->t_torque = s->t_quieto = 0.0f;
}

float screw_step(screw_t *s, const screw_model_t *m, float dt, float duty_aplicado, float w, float dvueltas)
{
    if (!screw_activo(s)) return 0.0f;
    const screw_cfg_t *c = &s->cfg;
    s->t += dt;
    s->vueltas += dvueltas;

    const screw_est_t e = screw_estimar(m, duty_aplicado, w);
    /* el último valor con el motor empujando: al terminar, el PWM cae y el modelo ya no diría nada del tornillo */
    s->i_final = e.i; s->torque_final = e.torque; s->fuerza_final = e.fuerza; s->presion_final = e.presion;

    /* Al arrancar, y justo después de bajar el PWM, la corriente refleja la aceleración y no el tornillo. */
    const bool asentado = s->t - s->t_fase >= c->t_arranque_s;
    if (asentado && e.torque > s->torque_max) s->torque_max = e.torque;

    s->t_corriente = (fabsf(e.i) > c->i_max) ? s->t_corriente + dt : 0.0f;
    if (s->t_corriente >= 0.1f) {
        /* Motor quieto y corriente alta antes de tiempo: atasco o encoder sin señal, que para el modelo es lo mismo. */
        terminar(s, (fabsf(w) < c->w_quieto && s->vueltas < c->vueltas_min) ? SCREW_R_TRABADO : SCREW_R_CORRIENTE);
        return 0.0f;
    }
    if (s->t >= c->t_max_s)     { terminar(s, SCREW_R_TIEMPO); return 0.0f; }
    if (s->vueltas > c->vueltas_max) { terminar(s, SCREW_R_SIN_APRIETE); return 0.0f; }

    switch (s->fase) {
    case SCREW_RAPIDO:
    case SCREW_LENTO:
        s->t_torque = (asentado && e.torque >= c->torque_asiento) ? s->t_torque + dt : 0.0f;
        if (s->t_torque >= c->t_confirma_s) {
            if (s->vueltas < c->vueltas_min) { terminar(s, SCREW_R_TRABADO); return 0.0f; }
            cambiar(s, SCREW_APRIETE, s->duty_apriete);
        } else if (s->fase == SCREW_RAPIDO && s->vueltas >= c->vueltas_meta - c->vueltas_lento) {
            cambiar(s, SCREW_LENTO, c->duty_lento);
        }
        break;
    case SCREW_APRIETE:
        /* Termina cuando el torque llega, o cuando el motor cala: su torque de calado con este PWM es la meta. */
        s->t_torque = (e.torque >= c->torque_meta * 0.95f) ? s->t_torque + dt : 0.0f;
        s->t_quieto = (fabsf(w) < c->w_quieto) ? s->t_quieto + dt : 0.0f;
        if (s->t_torque >= c->t_confirma_s || s->t_quieto >= 0.2f) {
            terminar(s, e.torque >= c->torque_meta * 0.85f ? SCREW_R_OK : SCREW_R_FLOJO);
            return 0.0f;
        }
        break;
    default:
        break;
    }
    return s->duty;
}

const char *screw_result_str(screw_result_t r)
{
    static const char *const n[] = { "ninguno", "ok", "sin_apriete", "trabado", "flojo", "tiempo", "sobrecorriente", "cancelado" };
    return r <= SCREW_R_CANCELADO ? n[r] : "?";
}
