/*
 * CONFIGURACIÓN CENTRAL DEL ROBOT SCARA.
 *
 * >>> Todo lo marcado con  [CAMBIAR]  son valores PROVISIONALES: hay que medirlos en el robot. <<<
 * Pines y datos eléctricos vienen del esquema v5 (documento de contexto).
 */
#pragma once
#include <stdint.h>
#if defined(__has_include)
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"          /* CONFIG_SCARA_COMMS_UROS (src/Kconfig.projbuild) */
#  endif
#endif
#include "driver/gpio.h"
#include "kinematics.h"

#define NJ 2                       /* articulaciones controladas: eslabón 1 y eslabón 2 */

/* ------------------------------------------------------------------ PINES */
#define PIN_STBY      GPIO_NUM_17  /* habilita U5/U6; R10 lo mantiene en 0 al arrancar */

#define PIN_M1_PWM    GPIO_NUM_2
#define PIN_M1_IN1    GPIO_NUM_16
#define PIN_M1_IN2    GPIO_NUM_4
#define PIN_M2_PWM    GPIO_NUM_19
#define PIN_M2_IN1    GPIO_NUM_5
#define PIN_M2_IN2    GPIO_NUM_18
/* Motor 3 (atornillador): NO se usa por ahora. Sus pines se dejan como salidas en 0 para que
 * las entradas de U6 no queden flotando. */
#define PIN_M3_PWM    GPIO_NUM_21
#define PIN_M3_IN1    GPIO_NUM_23
#define PIN_M3_IN2    GPIO_NUM_22

#define PIN_ENC1_A    GPIO_NUM_33
#define PIN_ENC1_B    GPIO_NUM_25
#define PIN_ENC2_A    GPIO_NUM_26
#define PIN_ENC2_B    GPIO_NUM_27

#define PIN_FC1_MIN   GPIO_NUM_36
#define PIN_FC1_MAX   GPIO_NUM_39
#define PIN_FC2_MIN   GPIO_NUM_34
#define PIN_FC2_MAX   GPIO_NUM_35

/* Interruptor normalmente abierto a GND con pull-up -> presionado = 0 (1 si usas NC) */
#define FC_PRESIONADO_NIVEL 0

/* ------------------------------------------------------------------ GEOMETRÍA (cinemática) */
extern const kin_cfg_t KIN;
#define Z_FIJO_MM     0.0f   /* [CAMBIAR] altura a la que trabaja el efector (eje z fijo por ahora) */
#define Z_TOL_MM      5.0f   /* se rechaza una consigna cuya z difiera más que esto de Z_FIJO */
#define Z_CHECK       1      /* 0 = ignora z de la consigna */
#define ELBOW_DEFAULT (+1)   /* codo preferido: +1 (q2>=0) o -1 (q2<=0) */

/* ------------------------------------------------------------------ ARTICULACIONES */
typedef struct {
    const char *name;
    gpio_num_t pwm, in1, in2, enc_a, enc_b, fc_min, fc_max;
    int   enc_sign;      /* +1/-1: hace que q AUMENTE al girar el eslabón en el sentido positivo */
    int   motor_sign;    /* +1/-1: hace que u>0 (duty positivo) AUMENTE q (se calibra con "pulse") */
    float k;             /* cuentas de encoder por radian articular (se calcula en config.c) */
    float q_min, q_max;  /* límites por software [rad] (solo valen con la articulación referenciada) */
    float q_home;        /* ángulo en el borde de liberación del final de carrera de Home [rad] */
    int   home_dir;      /* -1: busca el final de carrera MIN; +1: busca el MAX */
    float vmax, amax;    /* velocidad [rad/s] y aceleración [rad/s^2] máximas en movimientos punto a punto */
    float jog_v;         /* velocidad en modo manual [rad/s] */
    float kp, ki, kd, kff, fric;   /* ver pid.h */
} joint_cfg_t;
extern const joint_cfg_t JC[NJ];

/* Pololu 37D: 64 cuentas por vuelta del motor (x4) con reductor ~100.37:1 */
#define ENC_CUENTAS_SALIDA   6424.0f
/* Relación extra entre el eje de salida del reductor y el eslabón (poleas/engranajes). 1 = directo. */
#define RELACION_EXTRA_J1    1.0f    /* [CAMBIAR] */
#define RELACION_EXTRA_J2    1.0f    /* [CAMBIAR] */

/* ------------------------------------------------------------------ CONTROL */
#define CONTROL_HZ        1000
#define PWM_FREQ_HZ       20000
#define PWM_MAX           1023
#define DUTY_MAX          650        /* tope de PWM en lazo cerrado (~63 %): protege U5 (<=0.8 A por canal) */
#define DUTY_SLEW_PER_MS  3          /* rampa de PWM: 0 -> PWM_MAX en 0.34 s (regla de seguridad >= 0.3 s) */
#define VEL_FILTRO_TAU_S  0.010f     /* filtro de la velocidad medida */

#define HOLD_TOL_RAD      0.0070f    /* "llegó": |error| < 0.4° (≈1.3 mm a 190 mm) ... */
#define REACH_SETTLE_MS   150        /* ... durante este tiempo */
#define REACH_TIMEOUT_MS  3000       /* si no llega en este tiempo tras terminar la trayectoria: aviso */

/* Manual (teclado) */
#if defined(CONFIG_SCARA_COMMS_UROS)
/* Con ROS 2 los latidos viajan por XRCE-DDS (mas latencia/jitter que el serial de texto): mas margen. */
#define DEADMAN_TIMEOUT_MS  300
#else
#define DEADMAN_TIMEOUT_MS  150      /* sin latido del PC en este tiempo -> se detiene (cae el USB, se cuelga el PC). Soltar el
                                      * espacio detiene de inmediato porque el PC envía "d 0 0 0" al soltar. */
#endif
#define JOG_ACCEL           5.0f     /* [rad/s^2] aceleración en manual (~290 °/s²) */
#define JOG_STOP_ACCEL      15.0f    /* [rad/s^2] desaceleración al soltar flecha/espacio (~860 °/s²) */
#define JOG_LEAD_RAD        0.12f    /* máximo adelanto de la referencia respecto a la medición */

/* Seguridad */
#define FOLLOW_ERR_RAD    0.20f      /* error de seguimiento máximo (~11°)... */
#define FOLLOW_ERR_MS     120        /* ...durante este tiempo -> falla (trabado, encoder invertido, etc.) */
#define STALL_DUTY        350        /* PWM por encima del cual se vigila que el encoder cuente */
#define STALL_WINDOW_MS   300
#define STALL_MIN_COUNTS  3
#define FC_ANTIREBOTE_MS  5          /* filtro por software (además de los 100 nF) */
#define FALLA_FRENO_MS    150

/* Home */
#define HOME_DUTY         170        /* búsqueda del final de carrera (~17 %); valor calibrado en el robot real */
#define HOME_DUTY_SLOW    140        /* retroceso hasta liberar el final de carrera (~14 %) */
#define HOME_TIMEOUT_MS   25000
#define HOME_STALL_MS     400
/* Orden en que se referencian las articulaciones (índices 0-based). [CAMBIAR] según colisiones. */
#define HOME_ORDER_INIT   { 1, 0 }
/* El Home recorre cada eje hasta un tope, luego hasta el contrario (mide el recorrido real) y al final
 * devuelve el robot a la posición en que estaba. Los límites por software quedan este margen DENTRO de
 * los bordes medidos. */
#define HOME_MARGEN_RAD   0.052f     /* ~3 grados */

/* Prueba de sentido ("pulse"): un pulso corto para saber si u>0 aumenta q */
#define PULSE_DUTY        190
#define PULSE_MS          350

#define TELE_PERIODO_MS   100
