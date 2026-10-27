/*
 * FIRMWARE SCARA v5 - OPERACIÓN PUNTO A PUNTO (ESP32-DevKitC WROOM-32, ESP-IDF 5.x)
 *
 *   hal.c         PWM/dirección TB6612, encoders PCNT, finales de carrera
 *   control.c     lazo de 1 kHz: máquina de estados, Home, manual, punto a punto, seguridad
 *   pid.c         PID + prealimentación + fricción         (puro, probado en PC)
 *   traj.c        perfil trapezoidal sincronizado           (puro, probado en PC)
 *   kinematics.c  cinemática directa e inversa              (puro, probado en PC)
 *   comms.c       protocolo de texto por serial (más adelante: capa micro-ROS con las mismas funciones)
 */
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "comms.h"
#include "control.h"
#include "hal.h"

static void al_apagar(void)
{
    /* en un reinicio (panic, watchdog, esp_restart) deja los drivers apagados */
    hal_stby(false);
}

void app_main(void)
{
    hal_init();                          /* STBY = 0 antes que nada */
    esp_register_shutdown_handler(al_apagar);
    ctl_init();
    comms_start();
    ctl_start();
}
