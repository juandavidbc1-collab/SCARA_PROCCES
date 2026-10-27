# Firmware SCARA v5 — operación punto a punto (Reporte 3)

ESP32-DevKitC WROOM-32 · ESP-IDF 5.3 desde PlatformIO · Windows primero, luego Ubuntu + micro-ROS.
Control por **teclado por serial** (flechas + espacio como dead-man). Motor 3 y sensado de corriente: **desactivados**.

## Estructura

| Archivo | Qué hace | ¿Probado en PC? |
|---|---|---|
| `src/kinematics.c` | Cinemática directa/inversa 2R, dos codos, límites, singularidades | Sí (`tools/host_tests`) |
| `src/traj.c` | Perfil trapezoidal sincronizado en espacio articular | Sí |
| `src/pid.c` | PID + prealimentación de velocidad + fricción seca, anti-windup | Sí (simulación con modelo del motor) |
| `src/control.c` | Lazo 1 kHz: Home, manual, punto a punto, fallas | Sí (simulación completa con planta, `sim_control.c`) |
| `src/hal.c` | PWM TB6612, encoders PCNT, finales de carrera | **No** (necesita la tarjeta) |
| `src/comms.c` | Protocolo de texto por serial | Parcial |
| `src/config.c/.h` | **Todos los parámetros del robot** | — |
| `tools/teleop.py` | Teclado → latidos dead-man + comandos | Solo la lógica de teclas |

## Antes de mover nada: lo que hay que medir ([CAMBIAR] en `config.h` / `config.c`)

Todo esto es PROVISIONAL, no son datos de tu robot:
`KIN.l1`, `KIN.l2` (160/160 mm), `Z_FIJO_MM`, `RELACION_EXTRA_J1/J2` (1), `q_min/q_max/q_home` de cada articulación,
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
| Consigna (x,y,z) desde Python con ROS 2 / micro-ROS | **Pendiente** (fase Ubuntu) |
| Movimiento con retroalimentación de encoders | Hecho (PID), falta sintonizar en el robot |
| Caracterización (5 puntos × 10 repeticiones) | Falta el script de automatización y el ground truth |

Nota: el reporte pide "teach pendant con dead man's switch" además del control por teclado/joystick. Confirma con el profesor si el
dead-man por teclado cuenta como pendant; si no, el puerto J8 (encoder 3, libre) sirve para un botón físico.

## Pruebas en el PC

`sh tools/host_tests/run.sh` (gcc). Incluye la prueba de la interfaz de texto (eco, menú, salir con una tecla). Resultado actual: cinemática (4761 puntos, error FK(IK) 0.0001 mm), trayectoria (respeta vmax/amax),
simulación del lazo y simulación completa del firmware (Home, goto con error < 0.8 mm, dead-man, límites, 3 fallas).

## Lo que NO está verificado

- No se compiló contra ESP-IDF real (la red de donde se escribió no permitía bajar la plataforma de PlatformIO). Se revisó la sintaxis con cabeceras simuladas; puede haber errores de compilación que se ven en cuanto hagas `pio run`.
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
- Teleop con dead-man por ROS 2: `ros2/scara_teleop_ros.py`
- Pruebas en el PC: `sh tools/host_tests/run.sh` (incluye el intérprete de comandos y una comprobación de que la
  capa micro-ROS compila contra las firmas reales de `rclc`/`rmw_microxrcedds`). La ejecución con micro-ROS real
  está **sin verificar**.
