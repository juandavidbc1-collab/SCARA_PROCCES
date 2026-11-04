# Notas técnicas

Detalle del firmware, las pruebas y lo que falta verificar. Para empezar, mejor el [`README`](../README.md) y el [manual](MANUAL.md).

## Estado frente a los requisitos

| Requisito | Estado |
|---|---|
| RST posicional, incremental y enmascarado diseñados | Hecho. El enmascarado recibe valores de un PID y por dentro ejecuta un RST. Esa definición la escogió el equipo: confirmarla con el profesor |
| Simular la respuesta de posición de cada uno | Hecho: `python3 tools/rst/simulate.py` |
| Implementar en el robot el que mejor funcione | El firmware arranca con PID y las formas de RST se eligen en el panel. En la grabación `corrida_1`, el RST incremental hizo la PCB A en el robot. El equipo escogió el RST posicional; **falta dejarlo por defecto en el firmware** |
| Gemelo digital en RViz | Hecho; `/tf` y `/joint_states` comprobados con el simulador. `corrida_1` no grabó esos tópicos, así que **falta dejar constancia con el robot real** |
| Proceso repetitivo de la secuencia asignada | 16 huecos en 4 PCB, con servo y M3, y se puede elegir una sola PCB. Los 16 huecos pasaron en el robot con el firmware sin ROS. Por ROS, `corrida_1` muestra la PCB A (4 huecos) sin fallas. **Faltan las cuatro PCB por ROS, y la punta aún no gira: no atornilla** |
| Modelo de M3 de 200 rpm; corriente, presión y avance del tornillo | Hecho en `src/screw.c` para el diseño anterior, en el que M3 hacía girar la punta. Hoy M3 solo sube y baja, así que **no se usa en el proceso** |
| Servo del destornillador en lugar del sensor de corriente | Hecho y en uso (J13, GPIO32). Cambios de la placa en `docs/MANUAL.md` |
| Manual de uso | `docs/MANUAL.md` y su copia en Notion |
| Botón de paro de emergencia | En el firmware y el panel. El paro del panel se activó en el robot en `corrida_1`. El corte físico de 12 V es hardware, según `CONEXIONES_BORNERAS_v7-1.pdf`; **no hay registro de esa prueba** |
| Robot funcionando al 100 % | **Solo se puede comprobar en el robot** |

## Pruebas

```bash
source /opt/ros/jazzy/setup.bash && sh tools/host_tests/run.sh
```

Incluye cinemática, RST (contra la referencia en Python), simulación completa del firmware, intérprete de comandos, secuencia, panel sin pantalla y extremo a extremo con ROS 2 real. Medición del panel: `python3 tools/sim/bench_panel.py`.

`run.sh` se detiene en el primer programa que falla. **Estado al 4 de noviembre de 2026, corriendo todos:**

| Prueba | Resultado |
|---|---|
| `test_kin_traj`, `test_rst`, `test_screw`, `test_comms` | Pasan |
| `test_rst_design` | 10 de 10 |
| `test_sequence`, `test_link` | 79 de 79 |
| `test_panel` | 1 fallo de 49: espera A-H1 como primer hueco, y ahora es A-H2 |
| `test_uros_cmd` | 5 fallos: varios usan valores viejos |
| `sim_control` | 23 fallos: Home, `goto` en cascada, M3, tornillo y RST frente al PID |
| `test_e2e` | 5 fallos de 11: valores viejos, proceso con tornillo simulado y eje trabado |

Hay que actualizar estas pruebas. Los fallos no se han revisado todos uno por uno.

## Sin ROS

El firmware también funciona sin ROS: se maneja por texto desde el monitor de PlatformIO, y el proceso corre en el ESP32 (`src/proceso.c`).

```bash
python tools/uros_select.py text
pio run -t fullclean
pio run -t upload
pio device monitor
```

Después: `m3 home`, `5` y `proceso`. Para una sola PCB: `proceso pcb 2`. También hay `proceso ensayo` y `proceso lista`; `help` los muestra todos. Vuelve a ROS con `python tools/uros_select.py serial`.

## Controladores

El firmware arranca con **PID** (`CTRL_DEFAULT` en `src/config.h`). Con `ctrl rst` se activa el RST y `rstform` elige la forma. Al activarlo sin elegir, carga el enmascarado.

| Panel | Comando | Índice en `/scara/state` | Qué es |
|---|---|---|---|
| PID anterior | `ctrl pid` | 0 | `src/pid.c`, con reglas de fricción propias |
| RST posicional | `rstform pos` | 1 | Por polos. La consigna entra en P, I y D |
| RST incremental | `rstform inc` | 2 | Por polos (I-PD). La consigna entra solo por I |
| RST PI-D | `rstform pi_d` | 3 | Por polos. La consigna entra en P e I, no en D |
| RST enmascarado | `rstform mask` | 4 | Valores de un PID, ejecutados como RST. No se diseñó por polos |
| RST a medida | `rst j ...` | 5 | Cuando se cambian los parámetros a mano |

**El enmascarado usa las ganancias del PID anterior a la calibración:** Kp 5000 y Ki 3000. El PID actual tiene Kp 6000 y Ki 6000, además de reglas de fricción que el RST no tiene, así que hoy no da el mismo PWM. El panel muestra sus parámetros ya convertidos: `Kd_RST = Kd (1 + s1)` y `Kff_RST = Kff + Kd`. La conversión está en `pid_as_rst`, en `tools/rst/rst_design.py`.

**Cambio de nombres.** Antes, `rstform orig` se llamaba "RST PID original", y lo que hoy es `pi_d` se llamaba "RST enmascarado" (`rstform mask`). Los números de `/scara/state` no cambiaron. **El firmware y el panel deben ser de la misma versión:** con un firmware anterior, el botón "RST enmascarado" activaría la forma PI-D. `corrida_1` se grabó con los nombres anteriores.

## Prueba en el robot: `corrida_1`

Grabación del 4 de noviembre de 2026 (735 s), por micro-ROS. En ella, el RST incremental hizo el proceso de la PCB A (4 huecos), con servo y M3, sin fallas. Errores según los encoders; no hay medida externa de la punta:

| Hueco, en orden | Al llegar | Con M3 abajo | Al salir | Máximo en el hueco |
|---|---|---|---|---|
| A-H2 | 0,53 mm | 0,10 mm | 0,02 mm | 2,45 mm |
| A-H4 | 0,21 mm | 0,22 mm | 0,05 mm | 1,22 mm |
| A-H3 | 0,32 mm | 0,07 mm | 0,03 mm | 2,34 mm |
| A-H1 | 0,72 mm | 0,05 mm | 0,31 mm | 1,35 mm |

Durante el proceso, el error de seguimiento llegó a 5,4° y el PWM a 222 de 520. También se movieron el PID, el RST enmascarado y el RST posicional, en movimientos sueltos. Las gráficas y el análisis están en `SCARA_Controlador_RST.pdf`, sección 9, junto a la carpeta del proyecto.

## Calibración

Medida en el robot: L1 = 125 mm, L2 = 105 mm hasta la punta, J1 con 8400 cuentas por vuelta y J2 con unas 7350. M3 recorre 24,855 vueltas entre 106 y 6 mm. El detalle y lo que se pidió están en [`docs/CAMBIO_SERVO_DESTORNILLADOR.md`](CAMBIO_SERVO_DESTORNILLADOR.md).

## Lo que falta verificar

- **Por ROS en el robot:** las cuatro PCB seguidas. `corrida_1` solo tiene la PCB A.
- **RST PI-D:** se seleccionó en el robot, pero no lo ha movido.
- **Medida externa:** ninguna prueba por ROS midió dónde quedó de verdad la punta.
- **Pruebas en el PC:** hay fallos sin resolver; ver la sección Pruebas.
- **J2:** su escala está menos segura que la de J1, y su recorrido medido ha variado entre un Home y otro.
- **Juego del reductor:** el encoder está en el motor y no lo ve.
- **La punta no gira:** falta decidir cómo atornillar. Nada detecta un tornillo que no cayó.
- **RST posicional:** falta dejarlo por defecto y correr con él el proceso completo en el robot. En el simulador, con la PCB B, la punta se alejó hasta 3,37 mm del hueco después de llegar, contra 0,34 mm con el incremental.
- **Salto al cambiar de controlador en HOLD:** sin corregir. En `corrida_1`, al pasar a RST enmascarado con 0,17° de error, J1 saltó 6,1° con PWM 276. Mientras tanto, cambia de controlador en OFF.
- **PWM en reposo con RST incremental:** al final de `corrida_1`, J2 llegó a -520 PWM sin moverse y sin falla. No se sabe si el eje estaba pegado o si los motores ya no tenían alimentación. Al terminar, usa **Apagar motores**.
- **Enlace a 460800 baudios:** ver `UBUNTU_MICROROS.md`, sección 6.
- **Documentos anteriores:** `docs/m3/REPORTE_M3.md` y `Guia_operacion_SCARA_ROS2.pdf` son del diseño anterior.

## Estructura

| Archivo | Qué hace | ¿Probado en PC? |
|---|---|---|
| `src/kinematics.c` | Cinemática directa/inversa 2R, dos codos, límites, singularidades | Sí (`tools/host_tests`) |
| `src/traj.c` | Perfil trapezoidal sincronizado en espacio articular | Sí |
| `src/pid.c` | PID + prealimentación de velocidad + fricción seca, anti-windup (el activo al arrancar) | Sí (simulación con modelo del motor) |
| `src/rst.c` | Controlador RST (se activa con `ctrl rst`) | Sí (`tools/host_tests/test_rst.c`, y contra Python) |
| `src/screw.c` | Modelo de M3 y fases del atornillado | Sí (`tools/host_tests/test_screw.c`, `sim_control.c`) |
| `src/control.c` | Lazo 1 kHz: Home, manual, punto a punto, fallas | Sí (simulación completa con planta, `sim_control.c`) |
| `src/hal.c` | PWM TB6612, encoders PCNT, finales de carrera | **No** (necesita la tarjeta) |
| `src/comms.c` | Protocolo de texto por serial | Parcial |
| `src/config.c/.h` | **Todos los parámetros del robot** | — |
| `tools/teleop.py` | Teclado → latidos dead-man + comandos | Solo la lógica de teclas |

## Si cambia la mecánica: qué volver a medir

Están calibrados para este robot, en `config.h` y `config.c`: `KIN.l1`, `KIN.l2`, `RELACION_EXTRA_J1/J2`, `cuentas_a_cero`, y los signos `enc_sign` y `motor_sign`. `Z_FIJO_MM` sigue en 0, porque la altura la lleva M3.

## Terminal serie (monitor de PlatformIO o cualquier terminal)

Al arrancar muestra la configuración y un **menú de un toque**. Lo que escribes se ve en pantalla (eco), Backspace borra.

| Tecla | Acción |
|---|---|
| `1` | Estado actual |
| `2` | Sensores en vivo (topes y encoders) |
| `3` | Telemetría en vivo (q, x, y) |
| `4` | Pulso de prueba del sentido del motor (pregunta el eje) |
| `5` | HOME |
| `6` | Ir a x y z (pregunta la posición; si omites z usa la z fija) |
| `7` | Ir a ángulos q1 q2 |
| `8` / `9` / `0` | Detener (mantiene posición) / Apagar motores / Borrar falla |
| `m`, `help`, `cfg` | Menú, lista de comandos escritos, configuración |

**Salir de cualquier opción:** `Esc` o **cualquier tecla** detiene los monitores en vivo (2, 3, `tele`, `sens`) y los movimientos en curso (Home, goto, ang, pulso).
En una pregunta con datos (opciones 4, 6, 7), `Esc` o Enter vacío cancelan. Los dígitos solo son atajos con la línea vacía:
dentro de un comando, como `gain 1 5000 3000 180 80 60`, son números normales.

## Primera puesta en marcha (en este orden)

1. `pio run -t upload`. Abre el monitor de PlatformIO (o `python tools/teleop.py --port COM5`, pero no los dos a la vez: el puerto COM es de uno solo).
2. Con 12 V de lógica y **sin** 12 V de motores: opción `2` (sensores en vivo). Gira los ejes a mano y mira que cuenten los encoders y se activen los topes. Con la opción `3`, `q1` y `q2` deben **aumentar** al girar el eslabón en sentido antihorario; si baja, cambia `enc_sign` de ese eje.
3. Conecta 12 V de motores (paro de emergencia al alcance). Opción `4` con el eje 1 y luego el 2: dice si `u>0` aumenta `q`. Si sale INVERTIDO, cambia `motor_sign`. **No hagas Home antes de esto.**
4. Opción `5` (Home). Para cada eje (primero el 2, luego el 1): va despacio a un tope, retrocede hasta liberarlo (ahí fija `q_home`), va al tope contrario y retrocede (mide el recorrido real y deja los límites por software 3° dentro). Al final el brazo va a `0 0`, recto. Si un eje no se mueve, sube `HOME_DUTY`; si pasa mucho el interruptor, bájalo. Cualquier tecla lo detiene.
5. Manual: `python tools/teleop.py`, mantén **espacio** y usa las flechas. Soltar el espacio detiene.
6. Opción `6` o `goto 120 80 0` (x y z en mm) / `ang 20 90`.
7. Sintoniza con `gain j kp ki kd kff fric` (y `gain` para ver los valores).

## Protocolo serial (115200)

Dos modos. **Texto** (el de arranque, con eco y menú, ver arriba) y **máquina** (lo activa `teleop.py` con `fmt 1`: sin eco ni atajos, telemetría `ST,...` en CSV;
vuelve solo a texto tras 3 s sin tráfico del PC). Comandos: `d <dm> <a> <b>` (latido manual) · `home` · `goto x y z` · `ang q1 q2` · `stop` · `off` · `clear` ·
`elbow ±1` · `gain [j kp ki kd kff fric]` · `pulse j` · `tele 0|1` · `status` · `sens` · `cfg` · `fmt 0|1` · `menu` · `help`.
Eventos en texto claro: `#INFO`, `#ERROR`, `#FALLA`, `#HOME`, `#PULSO` y, al llegar a una consigna,
`#LLEGO: objetivo x= y= z= | medido x= y= | error ex= ey= mm | q1= q2=` (la pose medida sale de los encoders; **no es ground truth**).

## Seguridad implementada

STBY=0 al arrancar, en fallas y al reiniciar · rampa de PWM de 0.34 s a fondo de escala · tope de PWM de 520 sobre 1023 en el brazo (51 %) y 650 en M3 ·
dead-man con timeout de 150 ms en modo texto y 300 ms con micro-ROS (soltar = parada inmediata; USB caído = parada) · prohibido empujar contra un final de carrera presionado ·
fallas con freno de 150 ms: error de seguimiento, motor trabado, final de carrera inesperado en MOVE, tiempo de Home, MIN+MAX a la vez ·
Home exigido para `goto/ang` · límites por software con Home hecho · watchdog de tareas · `goto` rechaza z distinta de la altura fija.

## Cinemática (para el Notion)

`x = L1 cos q1 + L2 cos(q1+q2)`, `y = L1 sin q1 + L2 sin(q1+q2)`; `c2 = (x²+y²−L1²−L2²)/(2 L1 L2)`, `q2 = ±acos c2`,
`q1 = atan2(y,x) − atan2(L2 sin q2, L1 + L2 cos q2)`. Dos soluciones (codo ±): se intenta la preferida (`elbow`) y, si viola límites, la otra.
Singularidades en `q2 = 0` y `±180°` (`det J = L1 L2 sin q2`): se rechaza `|q2|` menor que 5° (`sing_margin`).

## Cobertura del Reporte 3

| Requisito | Estado |
|---|---|
| Home antes de punto a punto | Hecho: tope, tope contrario (mide el recorrido) y termina en `0 0`; bloquea `goto` sin Home |
| Movimiento manual por teclado | Hecho (flechas + espacio) |
| Teach pendant con dead-man | Dead-man por teclado hecho; **un pendant físico no existe** (ver nota) |
| Cinemática inversa en bajo nivel (C) | Hecho y probada en PC |
| Consigna (x,y,z) desde Python con ROS 2 / micro-ROS | Hecho: `/scara/goto` y `ros2/scara_client.py`. Probado con el simulador y en el robot, desde el panel (`corrida_1`) |
| Movimiento con retroalimentación de encoders | Hecho con PID y RST; probado en el robot (`corrida_1`) |
| Caracterización (5 puntos × 10 repeticiones) | Script listo (`tools/characterize.py`); falta el ground truth y ejecutarlo |

Nota: el reporte pide "teach pendant con dead man's switch" además del control por teclado/joystick. Confirma con el profesor si el
dead-man por teclado cuenta como pendant; si no, el puerto J8 (encoder 3, libre) sirve para un botón físico.

## Lo que NO está verificado

- `teleop.py` no se ha ejecutado en Windows ni con teclado real; solo se probó su lógica de teclas.
- Las ganancias del RST salen de una simulación con el modelo del motor, no del robot montado: la constante de tiempo es estimada y la ganancia viene de otra planta.
- La caracterización de exactitud y precisión no se ha ejecutado.

## Caracterización (Reporte 3) — preparada, pendiente de ejecutar

Lo que pide el reporte: exactitud (error medio vs. posición pedida) y precisión (dispersión) en
≥5 puntos del espacio de trabajo, ≥10 repeticiones por punto, con un método de ground truth
presentado y justificado; resultados en tablas o gráficas.

Ya está listo `tools/characterize.py` (5 puntos × 10 repeticiones, aproximación común, CSV y resumen
con RP/exactitud). Pendiente de decidir y hacer:

1. **Ground truth** (consultar con el profesor):
   - Regla / papel milimetrado bajo la punta (`--medir` o `--medir-cada`): mide (x,y) reales. Es lo que mejor cumple.
   - Encoders solos (sin `--medir`): solo caracteriza el control; el encoder está en el eje del motor y no ve el juego del reductor.
   - Finales de carrera con el ángulo entre topes conocido (CAD / transportador): referencia física del lado de la
     articulación; repetir Home N veces y comparar el recorrido medido; sirve para validar la escala
     (`RELACION_EXTRA_J1/J2`). No cubre puntos intermedios ni el cero.
2. Verificar el cero físico (`q_home`) y la escala antes de caracterizar.
3. Ejecutar: `python tools/characterize.py --port COM10 --reps 10 --medir-cada` (con Home hecho; teleop y monitor cerrados).

Nota de firmware: `HOLD_TOL_RAD` = 0.0070 rad (0.4°) para declarar "llegó"; con 0.2° los ejes, que se quedan cortos
0.12–0.18° por fricción seca, no entraban en la banda y daban "no llego a la consigna a tiempo".

## micro-ROS (ROS 2 Jazzy en Ubuntu)

El mismo firmware puede hablar ROS 2 en lugar del protocolo de texto: `src/uros_comms.c` reemplaza a
`src/comms.c` (se elige con `python tools/uros_select.py text|serial|wifi`). El control, la cinemática inversa
y la seguridad no cambian; la consigna (x, y, z) llega por `/scara/goto` y la validación/IK la hace
`src/motion.c`, compartido con el protocolo de texto.

- Guía completa (instalación, agente, pruebas, WiFi, fallos): **`UBUNTU_MICROROS.md`**
- Cliente Python: `ros2/scara_client.py` (home, goto, ang, cmd, state, monitor)
- Panel de control: `ros2/scara_panel.py` (`docs/MANUAL.md`). Muestra el error y el PWM en el tiempo y según la posición, y dónde quedó cada hueco
- Respaldo liviano por teclado, sin gráficas ni paro: `ros2/scara_teleop_ros.py`
- Pruebas en el PC: `sh tools/host_tests/run.sh` (incluye el intérprete de comandos y una comprobación de que la
  capa micro-ROS compila contra las firmas reales de `rclc`/`rmw_microxrcedds`). La ejecución con micro-ROS real
  se usó en el robot en `corrida_1`, por serial; WiFi está **sin verificar**.
