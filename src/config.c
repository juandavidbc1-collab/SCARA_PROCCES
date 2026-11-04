#include "config.h"
#include <math.h>
#include "control.h"

const char *const RST_FORM_NAME[] = { "RST posicional", "RST incremental", "RST PI-D", "RST enmascarado", "RST a medida" };

#define DEG(x) ((x) * (float)M_PI / 180.0f)
#define K_RAD(extra) (ENC_CUENTAS_SALIDA * (extra) / (2.0f * (float)M_PI))

/* Longitudes de los eslabones (centro de eje a centro de eje): L1 = 125 mm, L2 = 105 mm (medidos: L2 llega hasta la punta de la herramienta)
 * -> alcance de 20 a 230 mm. */
const kin_cfg_t KIN = { .l1 = 125.0f, .l2 = 105.0f, .sing_margin = DEG(5.0f) };

/*
 * paso_mm es lo que baja el prismático por vuelta (medido); la profundidad que se muestra es ese recorrido.
 * Motor 3: Pololu 37D 50:1 a 12 V, referido al eje de salida. Hoja de datos: 200 rpm y 0,2 A en vacío; 5,5 A y 21 kg·cm
 * calado (valores extrapolados por el fabricante). Sin medir en este robot.
 *   R = 12 / 5,5;  Ke = (12 - R I0) / w0;  Kt = T_calado / (I_calado - I0)   (Kt < Ke: pérdidas del reductor)
 * Tornillo M3 x 0,5: K = 0,2 (acero seco, valor típico), área bajo la cabeza de 5,5 mm con hueco de 3,2 mm.
 * torque_rosca: lo que cuesta girar la rosca antes de que asiente la cabeza. Supuesto.
 */
const screw_model_t M3_MODEL = {
    .r = 2.1818f, .ke = 0.5522f, .kt = 0.3886f, .i0 = 0.20f, .vbus = 12.0f, .pwm_max = PWM_MAX,
    .paso_mm = M3_PASO_MM, .k_tuerca = 0.2f, .d_mm = 3.0f, .area_mm2 = 15.7f, .torque_rosca = 0.03f,
};

const screw_cfg_t SCREW_CFG = {
    .vueltas_meta = SCREW_VUELTAS_META, .torque_meta = SCREW_TORQUE_META, .vueltas_lento = 2.0f,
    .vueltas_min = 0.75f * SCREW_VUELTAS_META, .vueltas_max = SCREW_VUELTAS_META + 2.0f,
    .duty_rapido = SCREW_DUTY_RAPIDO, .duty_lento = SCREW_DUTY_LENTO, .torque_asiento = 0.15f,
    .i_max = 2.4f,                   /* U6 con sus dos canales en paralelo: 2,4 A continuos */
    .t_arranque_s = 0.4f, .t_confirma_s = 0.03f, .t_max_s = SCREW_T_MAX_S, .w_quieto = 0.3f,
};

/*
 * Ganancias iniciales: salen de una simulación con el modelo del motor Pololu 37D
 * (tools/host_tests/sim_pid.c). Hay que afinarlas en el robot real con el comando "gain".
 *
 * Límites y signos son PROVISIONALES: [CAMBIAR] tras medir el robot.
 *   - cuentas_a_cero: medidas en el Home del 2026-11-03 que dejó el brazo donde se quería (102,98 y 129,53 grados). Corrígelas con
 *     el ángulo que marque el brazo recto: cuentas = grados * ENC_CUENTAS_SALIDA / 360.
 *   - q_min/q_max: déjalos ~3° dentro de los finales de carrera.
 */
const joint_cfg_t JC[NJ] = {
    {
        .name = "J1 (eslabon 1)",
        .q_fin = 0.0f,                    /* al terminar el Home, los dos ejes van a 0: el brazo recto */
        .pwm = PIN_M1_PWM, .in1 = PIN_M1_IN1, .in2 = PIN_M1_IN2,
        /* Medido en el Home del robot: yendo hacia q negativo, J1 toca el final de carrera conectado en J9 (FC1_MIN). */
        .enc_a = PIN_ENC1_A, .enc_b = PIN_ENC1_B, .fc_min = PIN_FC1_MIN, .fc_max = PIN_FC1_MAX,
        .enc_sign = -1, .motor_sign = -1,
        .k = K_RAD(RELACION_EXTRA_J1),
        .q_min = DEG(-120.0f), .q_max = DEG(120.0f), .cuentas_a_cero = 1838, .home_dir = -1, .home_duty = HOME_DUTY_J1, .home_duty_slow = HOME_DUTY_SLOW_J1,
        .vmax = DEG(25.0f), .amax = DEG(30.0f), .jog_v = DEG(30.0f),
        .kp = 6000.0f, .ki = 6000.0f, .kd = 180.0f, .kff = 80.0f, .fric = 150.0f,
    },
    {
        .name = "J2 (eslabon 2)",
        .q_fin = 0.0f,
        .pwm = PIN_M2_PWM, .in1 = PIN_M2_IN1, .in2 = PIN_M2_IN2,
        /* Medido en el Home del robot: el final de carrera de la derecha (q negativo) está en J11; el de la izquierda va en J12. */
        .enc_a = PIN_ENC2_A, .enc_b = PIN_ENC2_B, .fc_min = PIN_FC2_MIN, .fc_max = PIN_FC2_MAX,
        .enc_sign = +1, .motor_sign = -1,
        .k = K_RAD(RELACION_EXTRA_J2),
        .q_min = DEG(-140.0f), .q_max = DEG(140.0f), .cuentas_a_cero = 2376, .home_dir = -1, .home_duty = HOME_DUTY, .home_duty_slow = HOME_DUTY_SLOW,
       
        .vmax = DEG(25.0f), .amax = DEG(30.0f), .jog_v = DEG(30.0f),
        .kp = 6000.0f, .ki = 6000.0f, .kd = 180.0f, .kff = 80.0f, .fric = 110.0f,
    },
};
