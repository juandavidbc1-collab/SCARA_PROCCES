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
 *   estop                 paro de emergencia (se rearma con clear)
 *   motor 1|2|3 on|off    activa o desactiva un motor (3 = atornillador)
 *   tool pct              motor 3 en lazo abierto, -100..100 %
 *   ctrl pid|rst          controlador activo (con el robot en OFF o HOLD)
 *   rstform pos|inc|pi_d|mask  forma del RST en ambos ejes
 *   rst                   muestra el RST;   rst j kp ki kd s1 b c kff   lo cambia
 *   help
 *
 * Devuelve true si la orden se reconoció y se aceptó. En reply deja una línea de texto
 * ("#OK: ...", "#INFO: ...", "#ERROR: ...") SIN salto de línea, que sale por /scara/event.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>

bool uros_cmd_exec(const char *line, char *reply, size_t n);
