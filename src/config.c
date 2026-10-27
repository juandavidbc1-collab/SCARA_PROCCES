#include "config.h"
#include <math.h>

#define DEG(x) ((x) * (float)M_PI / 180.0f)
#define K_RAD(extra) (ENC_CUENTAS_SALIDA * (extra) / (2.0f * (float)M_PI))

/* Longitudes de los eslabones (centro de eje a centro de eje): L1 = 120 mm, L2 = 70 mm
 * -> alcance de 50 a 190 mm. */
const kin_cfg_t KIN = { .l1 = 120.0f, .l2 = 70.0f, .sing_margin = DEG(5.0f) };

/*
 * GANANCIAS DEL PID (por articulación, unidades de PWM: 1023 = 12 V).
 *
 * GANANCIAS_DEL_MODELO = 1 (por defecto): diseñadas con el modelo IDENTIFICADO del motor (config.h, MOT_*):
 *   planta de posición  theta/V = K / (s (tau s + 1)),  K = 0,9974 rad/s por V,  tau = 0,108 s
 *   lazo cerrado con PID (+ prealimentación kff = 1/K):  tau s^3 + (1 + K Kd) s^2 + K Kp s + K Ki = 0
 *   polos deseados  (s^2 + 2*0,9*20 s + 20^2)(s + 5)  ->  Kp = 62,8 V/rad, Ki = 216,6 V/(rad s), Kd = 3,44 V s/rad
 *   en PWM (x 1023/12):  kp = 5350, ki = 18500, kd = 295, kff = 85.
 *   Verificado en simulación (tools/host_tests/sim_pid.c) con el modelo identificado y con el de la hoja de datos.
 * GANANCIAS_DEL_MODELO = 0: las que se usaron en el Reporte 3 (simulación con el modelo de la hoja de datos):
 *   kp = 5000, ki = 3000, kd = 180, kff = 80, fric = 70.  Sirven de respaldo si las nuevas no te convencen.
 * En vivo: comando "gain j kp ki kd kff fric".
 *
 * Límites, q_home y signos son PROVISIONALES: [CAMBIAR] tras medir el robot.
 *   - q_home: ángulo del eslabón justo cuando se libera el final de carrera de Home.
 *   - q_min/q_max: déjalos ~3° dentro de los finales de carrera.
 */
#define GANANCIAS_DEL_MODELO 1
#if GANANCIAS_DEL_MODELO
#define G_KP 5350.0f
#define G_KI 18500.0f
#define G_KD 295.0f
#define G_KFF 85.0f
#else
#define G_KP 5000.0f
#define G_KI 3000.0f
#define G_KD 180.0f
#define G_KFF 80.0f
#endif
#define G_FRIC 70.0f

const joint_cfg_t JC[NJ] = {
    {
        .name = "J1 (eslabon 1)",
        .pwm = PIN_M1_PWM, .in1 = PIN_M1_IN1, .in2 = PIN_M1_IN2,
        /* topes cruzados: el final de carrera izquierdo (extremo + de q) esta en el conector FC1_MIN */
        .enc_a = PIN_ENC1_A, .enc_b = PIN_ENC1_B, .fc_min = PIN_FC1_MAX, .fc_max = PIN_FC1_MIN,
        .enc_sign = -1, .motor_sign = -1,
        .k = K_RAD(RELACION_EXTRA_J1),
        .q_min = DEG(-120.0f), .q_max = DEG(120.0f), .q_home = DEG(-108.0f), .home_dir = -1,
        .vmax = DEG(60.0f), .amax = DEG(120.0f), .jog_v = DEG(30.0f),
        .kp = G_KP, .ki = G_KI, .kd = G_KD, .kff = G_KFF, .fric = G_FRIC,
    },
    {
        .name = "J2 (eslabon 2)",
        .pwm = PIN_M2_PWM, .in1 = PIN_M2_IN1, .in2 = PIN_M2_IN2,
        .enc_a = PIN_ENC2_A, .enc_b = PIN_ENC2_B, .fc_min = PIN_FC2_MAX, .fc_max = PIN_FC2_MIN,
        .enc_sign = +1, .motor_sign = -1,
        .k = K_RAD(RELACION_EXTRA_J2),
        .q_min = DEG(-140.0f), .q_max = DEG(140.0f), .q_home = DEG(-143.0f), .home_dir = -1,
        .vmax = DEG(60.0f), .amax = DEG(120.0f), .jog_v = DEG(30.0f),
        .kp = G_KP, .ki = G_KI, .kd = G_KD, .kff = G_KFF, .fric = G_FRIC,
    },
};

/* Motor 3 (atornillador): sin reductor extra; mismos motores 37D 100:1. [CAMBIAR] signos tras probar. */
const tool_cfg_t TOOL = {
    .name = "M3 (atornillador)",
    .pwm = PIN_M3_PWM, .in1 = PIN_M3_IN1, .in2 = PIN_M3_IN2,
    .enc_a = PIN_ENC3_A, .enc_b = PIN_ENC3_B,
    .enc_sign = +1, .motor_sign = +1,
    .k = K_RAD(1.0f),
};
