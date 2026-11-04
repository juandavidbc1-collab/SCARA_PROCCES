/*
 * Proceso de atornillado sin ROS: traducción de ros2/scara_sequence.py y ros2/secuencia.yaml.
 * Corre dentro de la tarea de comunicaciones: proceso_tick() no bloquea.
 */
#pragma once
#include <stdbool.h>

/* pcb: 1 a 4 para hacer solo esa, 0 para todas. repeticiones = 0: hasta que se detenga. false si no puede arrancar. */
bool proceso_iniciar(bool ensayo, int pcb, int repeticiones);
void proceso_detener(void);
bool proceso_activo(void);
void proceso_listar(bool ensayo, int pcb);             /* imprime los pasos sin mover nada */
void proceso_tick(void);
void proceso_evento(const char *ev);                   /* eventos del lazo de control (#LLEGO, #M3, #FALLA, #ERROR) */
