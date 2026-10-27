/*
 * INTÉRPRETE DE COMANDOS PARA ROS 2 (tópico /scara/cmd, std_msgs/String).
 *
 * Sin dependencias de micro-ROS ni de ESP-IDF: se prueba en el PC (tools/host_tests).
 *
 *   home | stop | off | clear
 *   goto x y [z]          mm      (z por defecto = altura fija)
 *   ang q1 q2             grados
 *   elbow +1 | elbow -1
 *   pulse 1 | pulse 2     pulso corto para comprobar el sentido del motor
 *   gain                  muestra las ganancias;   gain j kp ki kd kff fric   las cambia
 *   status                resumen del estado
 *   motor j on|off        habilita / deshabilita el motor j (1, 2 o 3)     motors        muestra cuáles están activos
 *   motors a b c          los tres a la vez (1 = on, 0 = off)
 *   est [j]               parámetros de los observadores del motor j       est reset     reinicia los observadores
 *   luen j re im          polos del Luenberger del motor j: re +/- j im (|polo| < 1)
 *   kal j si sw sy        Kalman del motor j: sigma corriente [mA], sigma modelo vel. [rpm], sigma medida [rpm]
 *   vmap g v0             voltaje del modelo: V = g * (12 V * duty) - v0
 *   help
 *
 * Devuelve true si la orden se reconoció y se aceptó. En reply deja una línea de texto
 * ("#OK: ...", "#INFO: ...", "#ERROR: ...") SIN salto de línea, que sale por /scara/event.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>

bool uros_cmd_exec(const char *line, char *reply, size_t n);
