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

#define NJ 2                       /* articulaciones del brazo (posición): eslabón 1 y eslabón 2 */
#define NM 3                       /* motores en la tarjeta: M1 = J1, M2 = J2, M3 = atornillador (herramienta) */
#define M_TOOL 2                   /* índice del motor 3 (atornillador): sin finales de carrera ni Home */

/* Motores habilitados al arrancar (bit m = motor m+1). Se cambian en vivo con "motor j on|off".
 * Un motor deshabilitado queda libre (IN1 = IN2 = 0): no recibe PWM, no entra al PID, al Home ni a la
 * supervisión de fallas. Su encoder sigue contando, así que no pierde la referencia si lo mueves a mano. */
#define MOTORES_HABILITADOS_INIT  0x03   /* M1 y M2 sí, M3 no */

/* ------------------------------------------------------------------ PINES */
#define PIN_STBY      GPIO_NUM_17  /* habilita U5/U6; R10 lo mantiene en 0 al arrancar */

#define PIN_M1_PWM    GPIO_NUM_2
#define PIN_M1_IN1    GPIO_NUM_16
#define PIN_M1_IN2    GPIO_NUM_4
#define PIN_M2_PWM    GPIO_NUM_19
#define PIN_M2_IN1    GPIO_NUM_5
#define PIN_M2_IN2    GPIO_NUM_18
/* Motor 3 (atornillador): canal B de U6. Arranca deshabilitado (MOTORES_HABILITADOS_INIT). */
#define PIN_M3_PWM    GPIO_NUM_21
#define PIN_M3_IN1    GPIO_NUM_23
#define PIN_M3_IN2    GPIO_NUM_22
/* Encoder del motor 3 (conector J8). [CAMBIAR] confirma los GPIO con el esquema de la tarjeta.
 * Se eligieron pines libres que NO son de arranque (evita 0, 2, 12 y 15). Si el motor 3 no tiene encoder
 * conectado pon M3_TIENE_ENCODER 0: su observador funciona solo con el modelo (sin corrección). */
#define M3_TIENE_ENCODER 1
#define PIN_ENC3_A    GPIO_NUM_32
#define PIN_ENC3_B    GPIO_NUM_14

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

/* Motor 3 (herramienta): solo potencia manual en lazo abierto + observadores. */
typedef struct {
    const char *name;
    gpio_num_t pwm, in1, in2, enc_a, enc_b;
    int   enc_sign, motor_sign;
    float k;             /* cuentas de encoder por radian del eje de salida */
} tool_cfg_t;
extern const tool_cfg_t TOOL;

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

/* ------------------------------------------------------------------ POTENCIA MANUAL (lazo abierto)
 * /scara/power = [dead-man, %M1, %M2, %M3]. El PWM pedido se limita a DUTY_MAX (~63 %) y pasa por la
 * misma rampa. Si un eje del brazo tiene 0 %, se queda en HOLD con su PID; los que tienen potencia van
 * en lazo abierto. Sin latidos frescos (DEADMAN_TIMEOUT_MS) todo vuelve a 0 y el brazo a HOLD.
 * Sigue prohibido empujar contra un final de carrera y, con Home hecho, salir de los límites. */
#define POWER_STALL_FALLA  1         /* 1 = en potencia manual también vigila "motor trabado" en J1/J2 */

/* ------------------------------------------------------------------ MODELO DEL MOTOR (identificado)
 * Laboratorio 4 de Control Digital (Pololu 37D 12 V 100:1): parámetros ajustados para que a 12 V el
 * motor dé 0,2 A y 114,3 rpm, y con la dinámica medida (tau dominante ~0,108 s). Están referidos al EJE
 * DE SALIDA del reductor (= la articulación), así no dependen de si la caja es 100,37:1 o 102,08:1:
 *    La di/dt = V - Ra i - Kv w        J dw/dt = Kv i - B w          (w en rad/s del eje de salida)
 *  Kv = 1,6369e-3 V s/rad (lado motor) x 102,08  ;  B = 2,6793e-7 x 102,08^2  ;  J = 3,4858e-8 x 102,08^2 */
#define MOT_RA     50.0f          /* [ohm]  (medido) */
#define MOT_LA     0.060f         /* [H]    (medido) */
#define MOT_KV     0.167092f      /* [V s/rad] constante contraelectromotriz en el eje de salida */
#define MOT_B      2.79197e-3f    /* [N m s/rad] fricción viscosa en el eje de salida */
#define MOT_J      3.63232e-4f    /* [kg m^2] inercia en el eje de salida (motor solo, sin eslabón) */
#define VBUS       12.0f          /* [V] alimentación de los motores */

/* Voltaje que de verdad recibe el motor:  V = OBS_VGAIN * (VBUS * duty) - signo * OBS_V0   (0 si |.| < OBS_V0)
 * En el laboratorio, con L298N, se midió  V = 0,9782 (12 duty) - 1,197 V  (caída del puente bipolar + zona
 * muerta). La tarjeta usa TB6612 (MOSFET), que casi no tiene caída: queda la zona muerta por fricción seca,
 * ~0,8 V (equivale al "fric" = 70 del PID). Si usas un L298N pon OBS_DRIVER_L298N 1.
 * Se cambia en vivo con el comando "vmap <ganancia> <v0>". */
#define OBS_DRIVER_L298N 0
#if OBS_DRIVER_L298N
#define OBS_VGAIN  0.9782f
#define OBS_V0     1.197f
#else
#define OBS_VGAIN  1.0f
#define OBS_V0     0.8f
#endif

/* ------------------------------------------------------------------ OBSERVADORES (Luenberger + Kalman)
 * Corren a 100 Hz (cada OBS_DIV ticks del lazo de 1 kHz), igual que en el laboratorio (Ts = 10 ms), con
 * la velocidad medida como diferencia de cuentas del encoder en 10 ms. Miden SOLO la velocidad y estiman
 * velocidad y corriente. Se cambian en vivo con "luen" y "kal" (ver uros_cmd.h). */
#define OBS_DIV          10
#define OBS_TS           (OBS_DIV / (float)CONTROL_HZ)
#define OBS_POLO_RE      0.25f    /* polos del Luenberger: 0,25 +/- 0,25j (los del laboratorio) */
#define OBS_POLO_IM      0.25f
#define OBS_SIG_I        0.010f   /* Kalman, Q: incertidumbre del modelo en la corriente [A]  (10 mA) */
#define OBS_SIG_W_RPM    0.30f    /* Kalman, Q: incertidumbre del modelo en la velocidad [rpm] */
#define OBS_SIG_Y_RPM    0.81f    /* Kalman, R: ruido de la medición [rpm]  (R = 0,655 rpm^2 medido en el lab) */
#define OBS_P0_I         0.10f    /* Kalman, P0: incertidumbre inicial [A] y [rpm] */
#define OBS_P0_W_RPM     1.0f
