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
/* Motor 3: mueve el prismático. */
#define PIN_M3_PWM    GPIO_NUM_21
#define PIN_M3_IN1    GPIO_NUM_23
#define PIN_M3_IN2    GPIO_NUM_22
/* Servo en J13 (donde iba el ACS712): pin 1 = 5 V, pin 2 = señal, pin 3 = GND. La señal sale de GPIO32 y pasa por R40 hacia J13.
 * En la placa: R40 cambia de 10 k a 330 ohm, C14 se retira y R41 (22 k a GND) se queda para que el servo no salte al reiniciar. */
#define PIN_SERVO     GPIO_NUM_32

#define PIN_ENC1_A    GPIO_NUM_33
#define PIN_ENC1_B    GPIO_NUM_25
#define PIN_ENC2_A    GPIO_NUM_26
#define PIN_ENC2_B    GPIO_NUM_27
#define PIN_ENC3_A    GPIO_NUM_14
#define PIN_ENC3_B    GPIO_NUM_13

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
#define ELBOW_DEFAULT (-1)   /* el mismo del proceso, para que goto a mano y proceso lleguen igual: +1 (q2>=0) o -1 (q2<=0) */

/* ------------------------------------------------------------------ ARTICULACIONES */
typedef struct {
    const char *name;
    gpio_num_t pwm, in1, in2, enc_a, enc_b, fc_min, fc_max;
    int   enc_sign;      /* +1/-1: hace que q AUMENTE al girar el eslabón en el sentido positivo */
    int   motor_sign;    /* +1/-1: hace que u>0 (duty positivo) AUMENTE q (se calibra con "pulse") */
    float k;             /* cuentas de encoder por radian articular (se calcula en config.c) */
    float q_min, q_max;  /* límites por software [rad] (solo valen con la articulación referenciada) */
    int32_t cuentas_a_cero; /* cuentas de encoder desde que se libera el tope de Home hasta q = 0 (brazo recto) */
    float q_fin;         /* adonde va al terminar el Home [rad] */
    int   home_dir;      /* -1: busca el final de carrera MIN; +1: busca el MAX */
    int   home_duty, home_duty_slow;   /* PWM de búsqueda y de retroceso en el Home */
    float vmax, amax;    /* velocidad [rad/s] y aceleración [rad/s^2] máximas en movimientos punto a punto */
    float jog_v;         /* velocidad en modo manual [rad/s] */
    float kp, ki, kd, kff, fric;   /* ver pid.h */
} joint_cfg_t;
extern const joint_cfg_t JC[NJ];
#include "screw.h"
extern const screw_model_t M3_MODEL;
extern const screw_cfg_t   SCREW_CFG;

/* Pololu 37D: 64 cuentas por vuelta del motor (x4) con reductor ~100.37:1 */
#define ENC_CUENTAS_SALIDA   6424.0f
/* Relación extra entre el eje de salida del reductor y el eslabón (poleas/engranajes). 1 = directo. */
#define RELACION_EXTRA_J1    (8400.0f / ENC_CUENTAS_SALIDA)   /* 8400 cuentas por vuelta: lo que dio el ajuste con 4 puntos medidos; coincide con un reductor 131:1 */
#define RELACION_EXTRA_J2    1.144f  /* ajuste con 4 puntos medidos: unas 7350 cuentas por vuelta. Poco determinado: comprobar con "ang 0 -90" */

/* ------------------------------------------------------------------ CONTROL */
#define CONTROL_HZ        1000
#define PWM_FREQ_HZ       20000
#define PWM_MAX           1023
#define DUTY_MAX          520        /* tope de PWM en lazo cerrado (~51 %): protege U5 (<=0.8 A por canal) */
#define DUTY_BAJA_PER_MS  40         /* bajar el PWM es casi inmediato: para que el eje frene al soltarse */
#define DUTY_SLEW_PER_MS  3          /* rampa de PWM: 0 -> PWM_MAX en 0.34 s (regla de seguridad >= 0.3 s) */
#define VEL_FILTRO_TAU_S  0.010f     /* filtro de la velocidad medida */

#define HOLD_TOL_RAD      0.0070f    /* "llegó": |error| < 0.4° (≈1.3 mm a 190 mm) ... */
#define HOLD_TOL_AVISO_RAD 0.0175f   /* pasado el plazo, hasta 1° cuenta como llegado con aviso; más es un error */
#define PID_FRIC_QUIETO   90.0f      /* compensación de fricción con la consigna quieta */
#define PID_E_EMPUJE      0.0042f    /* quieto, por debajo de 0,24° no se empuja */
#define PID_EMPUJE_RITMO  400.0f     /* [PWM/s] */
#define PID_EMPUJE_MAX    450.0f     /* [PWM] */
#define PID_SE_MUEVE      0.15f      /* [rad/s] más que el pico de una sola cuenta de encoder (0,09) */
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
#define FC_ANTIREBOTE_MS  20         /* filtro por software (además de los 100 nF) */
#define FALLA_FRENO_MS    150

/* Home */
#define HOME_DUTY_J1      230        /* J1 no llegaba al final de carrera con 170 ni siempre con 210: le faltaba fuerza */
#define HOME_DUTY_SLOW_J1 190        /* el retroceso sube en la misma proporción, para que suelte el final de carrera */
#define HOME_DUTY         170        /* búsqueda del final de carrera (~17 %); valor calibrado en el robot real */
#define HOME_DUTY_SLOW    140        /* retroceso hasta liberar el final de carrera (~14 %) */
#define HOME_CENTRO_VEL   0.436f     /* [rad/s] 25 grados/s al ir a 0 0 tras el Home */
#define HOME_CENTRO_ACC   0.524f     /* [rad/s^2] 30 grados/s^2 */
/* Movimientos a un punto x, y: la punta va en línea recta. A esta velocidad, ninguna articulación pasa de unos 28 grados/s en los huecos. */
#define LINEA_VEL         20.0f      /* [mm/s] */
#define LINEA_ACC         40.0f      /* [mm/s^2] */
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

/* Controlador RST: corre cada RST_PERIOD_TICKS del lazo de 1 kHz. Con Ts = 1 ms y la forma "orig" da el mismo PWM que
 * el PID original. Si cambias Ts, vuelve a generar src/rst_defaults.h con tools/rst/simulate.py. */
#define RST_PERIOD_TICKS  1
#define RST_TS_S          (RST_PERIOD_TICKS / (float)CONTROL_HZ)
#define CTRL_DEFAULT      CTRL_PID   /* el RST se movía muy fuerte en el robot; se puede probar con "ctrl rst" */

/* Motor 3 (atornillador). Arranca desactivado; se activa con "motor 3 on". */
#define MOTORES_INIT      { true, true }
#define TOOL_ACTIVO_INIT  false
#define TOOL_DUTY_MAX     650        /* M3 tiene su propio tope: DUTY_MAX es el del brazo */
#define TOOL_MOTOR_SIGN   (-1)       /* medido en el robot: con este signo, duty > 0 baja M3 */
#define TOOL_ENC_SIGN     (-1)       /* medido en el robot: al bajar, el encoder resta; con este signo suma vueltas */
#define TOOL_ENC_CUENTAS  3200.0f    /* 64 cuentas por vuelta del motor x 50:1. [CAMBIAR] si una vuelta no da 3200 */
#define TOOL_VEL_TAU_S    0.020f     /* filtro de la velocidad de M3 */
#define TOOL_MAX_ON_MS    10000      /* sin renovar "tool" en este tiempo, M3 se apaga (comunicación perdida) */

/* Modelo de M3 (src/config.c) y atornillado. Tornillo supuesto M3 x 0,5 de 6 mm: [CAMBIAR] según el tornillo real. */
#define SCREW_VUELTAS_META   M3_VUELTAS_ATORNILLAR   /* desde arriba hasta la altura de atornillar */
#define SCREW_TORQUE_META    0.40f   /* N·m: apriete final */
#define SCREW_DUTY_RAPIDO    460.0f  /* [CAMBIAR] solo se probó a 230 en el robot: sube esto cuando el proceso funcione */
#define SCREW_DUTY_LENTO     250.0f  /* las medidas se tomaron a 230 */
#define SCREW_T_MAX_S        45.0f   /* 25 vueltas: a 230 de PWM el robot midió 0,76 vueltas/s */

/* Servo: sube el destornillador para que baje un tornillo y luego lo vuelve a bajar. 50 Hz, pulso de 500 a 2400 us para -180 a 180 grados.
 * Siempre arranca en 0 grados. -180 grados es el destornillador abajo, confirmado en el robot.
 * [CAMBIAR] SERVO_ARRIBA_DEG: el ángulo al que sube lo suficiente para que pase el tornillo. */
#define SERVO_HZ             50
#define SERVO_US_MIN         500.0f
#define SERVO_US_MAX         2400.0f
#define SERVO_GRADOS_MIN     (-180.0f)
#define SERVO_GRADOS_MAX     180.0f
#define SERVO_INICIO_DEG     0.0f
#define SERVO_ABAJO_DEG      (-180.0f)
#define SERVO_ARRIBA_DEG     180.0f  /* confirmado en el robot: con este angulo baja el tornillo */

/* Medidas de M3 tomadas en el robot el 2026-11-03 (a 230 de PWM y con el destornillador abajo):
 *   tope físico de arriba: encoder +0,657 vueltas, altura 106 mm
 *   posición de atornillar, que es también el límite de abajo: encoder -24,198 vueltas, altura 6 mm
 * Las vueltas de aquí en adelante se cuentan desde el tope de arriba y crecen al bajar. */
#define M3_ALTURA_ARRIBA_MM    106.0f
#define M3_ALTURA_ABAJO_MM     6.0f
#define M3_VUELTAS_ATORNILLAR  24.855f
#define M3_PASO_MM             ((M3_ALTURA_ARRIBA_MM - M3_ALTURA_ABAJO_MM) / M3_VUELTAS_ATORNILLAR)   /* ~4,02 mm por vuelta */
#define M3_INERCIA             0.10f   /* vueltas que sigue girando tras cortar el PWM (0,066 medidas, más la rampa) */
#define M3_VUELTAS_ARRIBA      0.25f   /* adonde va con "m3 subir": sin golpear el tope */
#define M3_VUELTAS_TRABAJO     (M3_VUELTAS_ATORNILLAR * 0.75f)   /* entre huecos, M3 sube solo 1/4 de la barra: 25 mm, hasta 31 mm de altura */
#define M3_VUELTAS_TRASLADO    (M3_VUELTAS_TRABAJO + 0.5f)       /* por debajo de esto el brazo no se mueve */
#define M3_VUELTAS_LENTO       3.0f    /* a esta distancia del destino pasa a lento */
#define M3_ENSAYO_VUELTAS      M3_VUELTAS_ATORNILLAR   /* el ensayo baja lo mismo que el proceso */
#define M3_QUIETO_MS           500     /* yendo a una altura: sin avanzar este tiempo, está trabado o tocando algo */
#define M3_HOME_DUTY           230.0f  /* con este PWM ya se apoyó en el tope dos veces durante las medidas */
#define M3_HOME_QUIETO_MS      400     /* encoder quieto este tiempo = llegó al tope */
#define M3_HOME_MIN_CUENTAS    3
#define M3_HOME_LIMITE_S       60

/* Tiempos del proceso. [CAMBIAR] según el mecanismo real. */
#define PROC_ASENTAR_S         0.3f
#define PROC_SERVO_ARRIBA_S    1.0f    /* recorre de -180 a 180 grados y el tornillo debe terminar de caer */
#define PROC_SERVO_BAJAR_S     1.0f
#define PROC_LIMITE_BRAZO_S    25.0f
#define PROC_LIMITE_M3_S       60.0f
