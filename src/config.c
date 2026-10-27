#include "config.h"
#include <math.h>

#define DEG(x) ((x) * (float)M_PI / 180.0f)
#define K_RAD(extra) (ENC_CUENTAS_SALIDA * (extra) / (2.0f * (float)M_PI))

/* Longitudes de los eslabones (centro de eje a centro de eje): L1 = 120 mm, L2 = 70 mm
 * -> alcance de 50 a 190 mm. */
const kin_cfg_t KIN = { .l1 = 120.0f, .l2 = 70.0f, .sing_margin = DEG(5.0f) };

/*
 * Ganancias iniciales: salen de una simulación con el modelo del motor Pololu 37D
 * (tools/host_tests/sim_pid.c). Hay que afinarlas en el robot real con el comando "gain".
 *
 * Límites, q_home y signos son PROVISIONALES: [CAMBIAR] tras medir el robot.
 *   - q_home: ángulo del eslabón justo cuando se libera el final de carrera de Home.
 *   - q_min/q_max: déjalos ~3° dentro de los finales de carrera.
 */
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
        .kp = 5000.0f, .ki = 3000.0f, .kd = 180.0f, .kff = 80.0f, .fric = 70.0f,
    },
    {
        .name = "J2 (eslabon 2)",
        .pwm = PIN_M2_PWM, .in1 = PIN_M2_IN1, .in2 = PIN_M2_IN2,
        .enc_a = PIN_ENC2_A, .enc_b = PIN_ENC2_B, .fc_min = PIN_FC2_MAX, .fc_max = PIN_FC2_MIN,
        .enc_sign = +1, .motor_sign = -1,
        .k = K_RAD(RELACION_EXTRA_J2),
        .q_min = DEG(-140.0f), .q_max = DEG(140.0f), .q_home = DEG(-143.0f), .home_dir = -1,
        .vmax = DEG(60.0f), .amax = DEG(120.0f), .jog_v = DEG(30.0f),
        .kp = 5000.0f, .ki = 3000.0f, .kd = 180.0f, .kff = 80.0f, .fric = 70.0f,
    },
};
