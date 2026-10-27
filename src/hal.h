/* Capa de hardware: PWM + dirección (TB6612), encoders (PCNT) y finales de carrera.
 * Los motores se indexan 0..NM-1: 0 = J1, 1 = J2, 2 = M3 (atornillador, ver TOOL en config.c). */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

void    hal_init(void);                 /* lo PRIMERO que hace es poner STBY = 0 */
void    hal_stby(bool on);
bool    hal_stby_get(void);
void    hal_motor_drive(int m, int duty);  /* duty con signo en [-PWM_MAX, PWM_MAX]; u>0 ya incluye motor_sign */
void    hal_motor_free(int m);          /* IN1 = IN2 = 0: motor libre */
void    hal_motor_brake(int m);         /* IN1 = IN2 = 1: freno */
void    hal_all_free(void);
int32_t hal_enc_raw(int m);             /* cuentas crudas (cuadratura x4), sin signo aplicado (0 si no hay encoder) */
int     hal_enc_levels(int m);          /* bit0 = A, bit1 = B (diagnóstico) */
bool    hal_fc_raw(int j, int side);    /* solo J1/J2. side 0 = MIN, 1 = MAX; true = presionado (sin antirrebote) */
bool    hal_tiene_encoder(int m);
