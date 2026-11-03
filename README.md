# Firmware SCARA — proyecto final

ESP32-DevKitC WROOM-32 · ESP-IDF 5.3 · ROS 2 Jazzy + micro-ROS. Control RST en el ESP32, panel en PyQt5, gemelo digital en RViz y simulador completo.

**Manual de uso:** [`docs/MANUAL.md`](docs/MANUAL.md) · **Diseño y simulación del RST:** [`docs/rst/REPORTE_RST.md`](docs/rst/REPORTE_RST.md) · **Motor 3 y atornillado:** [`docs/m3/REPORTE_M3.md`](docs/m3/REPORTE_M3.md)

## Probarlo sin el robot

```bash
source /opt/ros/jazzy/setup.bash
python3 ros2/scara_panel.py --sim       # simulador + RViz + panel
```

El simulador compila y corre el mismo C del firmware (`src/control.c`, `rst.c`, `motion.c`, `uros_cmd.c`) contra un modelo de los motores, y publica los mismos tópicos que el ESP32.

## Estado frente a los requisitos

| Requisito | Estado |
|---|---|
| RST posicional, incremental y enmascarado diseñados | Hecho. La definición de "masked" es provisional: confirmarla con el profesor |
| Simular la respuesta de posición de los tres | Hecho: `python3 tools/rst/simulate.py` |
| Implementar en el robot el que mejor funcione | Hecho en firmware. Por defecto va `orig`, el PID original escrito como RST, porque da el PWM que ya funcionaba; las otras tres se eligen en el panel. **Falta probarlo en el robot** |
| Gemelo digital en RViz | Hecho; `/tf` y `/joint_states` comprobados con el simulador. **Falta verlo con el robot real** |
| Proceso repetitivo de la secuencia asignada | Hecho en `ros2/secuencia.yaml`: 16 huecos en 4 placas, con servo y atornillado. 16 de 16 `ok` en simulación. **Falta probarlo en el robot** |
| Modelo de M3 de 200 rpm; corriente, presión y avance del tornillo | Hecho: `src/screw.c`. Motor supuesto Pololu 37D 50:1, sin medir. Ver `docs/m3/REPORTE_M3.md` |
| Servo del destornillador en lugar del sensor de corriente | Hecho en firmware (J13, GPIO32). **Falta probarlo y cambiar R40 y C14 en la placa** (ver `docs/MANUAL.md`) |
| Manual de uso | `docs/MANUAL.md` y su copia en Notion |
| Botón de paro de emergencia | En el firmware y el panel. El corte físico de 12 V es hardware, según `CONEXIONES_BORNERAS_v7-1.pdf`. **Falta probarlo en el robot** |
| Robot funcionando al 100 % | **Solo se puede comprobar en el robot** |

## Pruebas

```bash
source /opt/ros/jazzy/setup.bash && sh tools/host_tests/run.sh
```

Incluye cinemática, RST (contra la referencia en Python), simulación completa del firmware, intérprete de comandos, secuencia, panel sin pantalla y extremo a extremo con ROS 2 real. Medición del panel: `python3 tools/sim/bench_panel.py`.

## Lo que falta verificar en el robot

- Nada del RST, del paro por software ni de la activación por motor se ha ejecutado en la tarjeta. El firmware **compila** contra ESP-IDF con micro-ROS; no se ha cargado.
- La inercia de los eslabones es una estimación: medirla y volver a correr `tools/rst/simulate.py`.
- M3: sentido, encoder en J8, modelo y tornillo. La lista completa está en `docs/m3/REPORTE_M3.md`, sección 7.
- Servo: cambio de R40 y C14, ángulo de subida y caída de los 5 V cuando se mueve.
- M3: medir `m3.paso_mm` y `m3.bajada_max_mm` en `ros2/secuencia.yaml`, para que el destornillador no llegue al suelo.
- El brazo no tiene eje z: comprueba la altura y cómo baja M3 con el tornillo. Tras cada tornillo, M3 sube 18,5 s girando al revés, lo que añade unos 5 minutos a los 16 huecos. Se supone que la punta se suelta sola del tornillo, así que la subida no lo afloja, y el simulador lo modela así. Compruébalo con el primer tornillo. Si no se suelta, según el simulador M3 desenrosca unas 10 de las 12 vueltas: cambia esa subida en `ros2/secuencia.yaml`.
- Enlace a 460800 baudios (ver `UBUNTU_MICROROS.md`, sección 6).
- Sobrepaso del final de carrera en Home: la simulación da 4,5°, cerca del tope duro simulado de 5°. Antes esa prueba exigía menos de 3° y fallaba sin que nadie lo viera.
- `Guia_operacion_SCARA_ROS2.pdf` es anterior a estos cambios.

---

# Referencia del firmware

Punto a punto, primera puesta en marcha y protocolo de texto. Control por teclado por serial (flechas + espacio como dead-man). La corriente de M3 se calcula con su modelo, porque el servo ocupa el sitio del sensor; el motor 3 arranca desactivado y se activa con `motor 3 on`.

## Estructura

| Archivo | Qué hace | ¿Probado en PC? |
|---|---|---|
| `src/kinematics.c` | Cinemática directa/inversa 2R, dos codos, límites, singularidades | Sí (`tools/host_tests`) |
| `src/traj.c` | Perfil trapezoidal sincronizado en espacio articular | Sí |
| `src/pid.c` | PID + prealimentación de velocidad + fricción seca, anti-windup | Sí (simulación con modelo del motor) |
| `src/rst.c` | Controlador RST (el activo por defecto) | Sí (`tools/host_tests/test_rst.c`, y contra Python) |
| `src/screw.c` | Modelo de M3 y fases del atornillado | Sí (`tools/host_tests/test_screw.c`, `sim_control.c`) |
| `src/control.c` | Lazo 1 kHz: Home, manual, punto a punto, fallas | Sí (simulación completa con planta, `sim_control.c`) |
| `src/hal.c` | PWM TB6612, encoders PCNT, finales de carrera | **No** (necesita la tarjeta) |
| `src/comms.c` | Protocolo de texto por serial | Parcial |
| `src/config.c/.h` | **Todos los parámetros del robot** | — |
| `tools/teleop.py` | Teclado → latidos dead-man + comandos | Solo la lógica de teclas |

## Antes de mover nada: lo que hay que medir ([CAMBIAR] en `config.h` / `config.c`)

Todo esto es PROVISIONAL, no son datos de tu robot:
`KIN.l1`, `KIN.l2` (120/70 mm), `Z_FIJO_MM`, `RELACION_EXTRA_J1/J2` (1), `q_min/q_max/q_home` de cada articulación,
`HOME_ORDER_INIT`, pose de reposo (`PARK_*`), y los signos `enc_sign` / `motor_sign`.

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
4. Opción `5` (Home). Para cada eje (primero el 2, luego el 1): va despacio a un tope, retrocede hasta liberarlo (ahí fija `q_home`), va al tope contrario y retrocede (mide el recorrido real y deja los límites por software 3° dentro). Al final los dos ejes vuelven a la posición en que estaban al pulsar Home. Si un eje no se mueve, sube `HOME_DUTY`; si pasa mucho el interruptor, bájalo. Cualquier tecla lo detiene.
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

STBY=0 al arrancar, en fallas y al reiniciar · rampa de PWM de 0.34 s a fondo de escala · tope de PWM 63 % ·
dead-man con timeout de 150 ms (soltar = parada inmediata; USB caído = parada) · prohibido empujar contra un final de carrera presionado ·
fallas con freno de 150 ms: error de seguimiento, motor trabado, final de carrera inesperado en MOVE, tiempo de Home, MIN+MAX a la vez ·
Home exigido para `goto/ang` · límites por software con Home hecho · watchdog de tareas · `goto` rechaza z distinta de la altura fija.

## Cinemática (para el Notion)

`x = L1 cos q1 + L2 cos(q1+q2)`, `y = L1 sin q1 + L2 sin(q1+q2)`; `c2 = (x²+y²−L1²−L2²)/(2 L1 L2)`, `q2 = ±acos c2`,
`q1 = atan2(y,x) − atan2(L2 sin q2, L1 + L2 cos q2)`. Dos soluciones (codo ±): se intenta la preferida (`elbow`) y, si viola límites, la otra.
Singularidades en `q2 = 0` y `±180°` (`det J = L1 L2 sin q2`): se rechaza `|q2|` menor que 5° (`sing_margin`).

## Cobertura del Reporte 3

| Requisito | Estado |
|---|---|
| Home antes de punto a punto | Hecho: tope, tope contrario (mide el recorrido) y vuelve a la posición inicial; bloquea `goto` sin Home |
| Movimiento manual por teclado | Hecho (flechas + espacio) |
| Teach pendant con dead-man | Dead-man por teclado hecho; **un pendant físico no existe** (ver nota) |
| Cinemática inversa en bajo nivel (C) | Hecho y probada en PC |
| Consigna (x,y,z) desde Python con ROS 2 / micro-ROS | Hecho: `/scara/goto` y `ros2/scara_client.py`. Probado con el simulador; **falta probarlo en el robot** |
| Movimiento con retroalimentación de encoders | Hecho (PID), falta sintonizar en el robot |
| Caracterización (5 puntos × 10 repeticiones) | Script listo (`tools/characterize.py`); falta el ground truth y ejecutarlo |

Nota: el reporte pide "teach pendant con dead man's switch" además del control por teclado/joystick. Confirma con el profesor si el
dead-man por teclado cuenta como pendant; si no, el puerto J8 (encoder 3, libre) sirve para un botón físico.

## Pruebas en el PC

`sh tools/host_tests/run.sh` (gcc). Incluye la prueba de la interfaz de texto (eco, menú, salir con una tecla). Resultado actual: cinemática (4761 puntos, error FK(IK) 0.0001 mm), trayectoria (respeta vmax/amax),
simulación del lazo y simulación completa del firmware (Home, goto con error < 0.8 mm, dead-man, límites, 3 fallas).

## Lo que NO está verificado

- Compila contra ESP-IDF 5.3 con `pio run`, en modo micro-ROS y en modo texto, pero no se ha cargado en la tarjeta.
- `hal.c` y los tiempos reales del hardware no se han probado en la tarjeta.
- `teleop.py` no se ha ejecutado en Windows ni con teclado real; solo se probó su lógica de teclas.
- Las ganancias salen de una simulación con el modelo del motor del documento de diseño, no del robot montado.

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
- Panel de control: `ros2/scara_panel.py` (`docs/MANUAL.md`)
- Respaldo liviano por teclado, sin gráficas ni paro: `ros2/scara_teleop_ros.py`
- Pruebas en el PC: `sh tools/host_tests/run.sh` (incluye el intérprete de comandos y una comprobación de que la
  capa micro-ROS compila contra las firmas reales de `rclc`/`rmw_microxrcedds`). La ejecución con micro-ROS real
  está **sin verificar**.
