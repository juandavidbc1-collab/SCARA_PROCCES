/*
 * TRAYECTORIA PUNTO A PUNTO EN ESPACIO ARTICULAR (perfil trapezoidal sincronizado)
 *
 * Todas las articulaciones recorren una recta en el espacio articular parametrizada por
 * s(t) en [0,1]:   q_j(t) = q0_j + s(t) * (q1_j - q0_j).
 * s(t) sigue un perfil trapezoidal (o triangular si el recorrido es corto) cuyos límites
 * se eligen para que NINGUNA articulación supere su vmax ni su amax:
 *      vs = min_j(vmax_j / |dq_j|),   as = min_j(amax_j / |dq_j|)
 * Así todas llegan al mismo tiempo y la aceleración es continua por tramos.
 */
#pragma once
#include <stdbool.h>

#define TRAJ_NJ 2

typedef struct {
    float q0[TRAJ_NJ], dq[TRAJ_NJ];
    float vs, as, ta, T;      /* vel. y acel. de s, tiempo de aceleración y duración total [s] */
} traj_t;

/* Devuelve false si el movimiento es nulo (T = 0). */
bool traj_plan(traj_t *t, const float q0[TRAJ_NJ], const float q1[TRAJ_NJ],
               const float vmax[TRAJ_NJ], const float amax[TRAJ_NJ]);

/* Evalúa posición y velocidad de referencia en el instante tm [s] desde el inicio. */
void traj_eval(const traj_t *t, float tm, float q[TRAJ_NJ], float qd[TRAJ_NJ]);
