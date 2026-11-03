# SCARA: RST, paro, motores, secuencia, panel y simulación — plan

> Para quien lo ejecute: superpowers:executing-plans, tarea por tarea. Sin commits salvo que el usuario los pida.

**Goal:** Cumplir los requisitos de los dos profesores sobre `scara_fw`: RST (posicional, incremental, masked) diseñados y simulados, el mejor en el robot, gemelo en RViz, secuencia repetitiva, manual en Notion, paro de emergencia; más panel nuevo y rápido, activar/desactivar por motor y simulación completa sin hardware.

**Architecture:** El firmware sigue siendo la única fuente de lógica: `rst.c` entra junto a `pid.c` y `control.c` elige. El simulador compila ese mismo C con una planta (`libscara_sim.so`) y un nodo ROS 2 publica los mismos tópicos que el ESP32, así que el panel y RViz no distinguen real de simulado. El diseño RST es Python puro y exporta los valores por defecto a C.

**Tech Stack:** C (ESP-IDF, gcc en PC), Python 3 + NumPy/Matplotlib, ROS 2 Jazzy (rclpy), PyQt5 + pyqtgraph.

**Spec:** imágenes `../control.jpeg`, `../robotica.jpeg` y el pedido del usuario. Base elegida por el usuario: solo `scara_fw`; de `SCARA_PROCCES` solo se usan como dato los parámetros identificados.

## Global Constraints

- No copiar código de `scara_nuevo/SCARA_PROCCES`.
- No hacer commits ni tocar `components/micro_ros_espidf_component`.
- `/scara/state`: los índices 0-12 no cambian; los campos nuevos van al final.
- Convención RST (Landau): `S(q⁻¹)u(k) = T(q⁻¹)r(k) − R(q⁻¹)y(k)`.
- Lo no verificable en hardware se dice como tal en el README y en el resumen final.

## Decisiones de control

- Ts = 10 ms (100 Hz dentro del lazo de 1 kHz), el mismo período del Lab 4.
- Planta de diseño `q/V = K/(s(τs+1))`, ZOH. K = 0,9974 rad/s/V (identificado). τ1 = 0,14 s y τ2 = 0,10 s (estimados con inercia de eslabón, **sin medir en el robot**). V = duty·12/1023.
- `S = (1−q⁻¹)(1+s1 q⁻¹)`, R de orden 2, ecuación diofántica `A S + B R = P`, 4 polos: par dominante (ωn, ζ) y 2 auxiliares.
- Forma PID: `R = Kp(1−q⁻¹)(1+s1q⁻¹) + Ki Ts(1+s1q⁻¹) + (Kd/Ts)(1−q⁻¹)²`.
- Las tres variantes comparten R y S; difieren en T con pesos (b, c): `T = b·Kp(1−q⁻¹)(1+s1q⁻¹) + Ki Ts(1+s1q⁻¹) + c·(Kd/Ts)(1−q⁻¹)²`.
  - posicional b=1, c=1 (T=R)
  - incremental b=0, c=0 (I-PD)
  - masked b=1, c=0 (PI-D) — **definición provisional**: el término no aparece en los apuntes ni en la web. Se confirma con el profesor; cambiarla es cambiar (b, c).
- Salida: saturación `DUTY_MAX − fric` y rampa de 30 PWM por muestra dentro del RST (la historia guarda lo aplicado: anti-windup); la compensación de fricción se suma después.
- `pid.c` se conserva, seleccionable con `ctrl pid`, como respaldo en el robot.

## Comandos nuevos (mismo intérprete para ROS y texto)

`estop` · `clear` (rearma) · `motor j on|off` (j=1,2,3) · `tool duty` (-100..100 %) · `ctrl pid|rst` · `rstform pos|inc|mask` · `rst j kp ki kd s1 b c` · `rst` (lee).

## Comportamiento

- **Motor desactivado:** driver libre, encoder sigue contando, sin supervisión. Home solo sus ejes activos. `goto` exige J1 y J2. `ang` mueve solo los activos. Al reactivar mantiene la posición. M3 arranca desactivado.
- **Paro:** bandera directa, no pasa por la cola. Freno 150 ms, motores libres, STBY=0, M3 apagado, enclavado hasta `clear`. Conserva el Home. Tras `clear`, el manual exige soltar el dead-man primero. Entrada física opcional en `PIN_ESTOP`, desactivada por defecto (`ESTOP_FISICO 0`).
- **Latido del panel:** corre en hilo aparte de la interfaz; solo manda dead-man=1 si la interfaz respondió hace menos de 150 ms.

## Review Focus

1. Paro pedido durante Home, durante una falla ya activa, o con la cola de órdenes llena: debe ganar siempre.
2. Interfaz congelada con espacio y flecha pulsados: el robot debe detenerse.
3. Motor desactivado en plena trayectoria o mientras se hace Home: sin falla falsa ni salto al reactivar.
4. Falla, paro o rechazo de punto a mitad de secuencia: la secuencia aborta y no manda más puntos.
5. Cortar 12 V y volver a darlos (paro físico): sin salto brusco.

---

### Task 1: Diseño RST en Python

**Files:** Create `tools/rst/rst_design.py`, `tools/rst/test_rst_design.py`.
**Produces:** `plant(K,tau,Ts)->(A,B)`, `diophantine(A,B,P)->(R,S)`, `pid_from_rs(R,S,Ts)->(kp,ki,kd,s1)`, `polys(kp,ki,kd,s1,b,c,Ts)->(R,S,T)`, `design_joint(K,tau,Ts,wn,zeta,aux)`, `FORMS`.

- [ ] Pruebas: polos de lazo cerrado = los pedidos (tol 1e-6); `S(1)=0`; `T(1)=R(1)` en las 3 formas; `polys` reconstruye R,S; `|s1|<1`.
- [ ] Verlas fallar, implementar, verlas pasar: `python3 -m unittest tools/rst/test_rst_design.py`.

### Task 2: `rst.c` y paridad C/Python

**Files:** Create `src/rst.c`, `src/rst.h`, `tools/host_tests/test_rst.c`.
**Produces:** `rst_set_pid(rst_ctl_t*,kp,ki,kd,s1,b,c,Ts)`, `rst_reset(rst_ctl_t*,y0)`, `float rst_step(rst_ctl_t*,r,y,umax,du_max)`, `const float RST_DEFAULTS`.

- [ ] Prueba C: arranque sin salto (`u=0` con r=y); escalón de consigna en lazo cerrado con planta discreta coincide con el modelo; saturación sin windup; vectores generados por Python coinciden en C (tol 1e-3).
- [ ] Implementar. Calcular con diferencias respecto a y(k) para no perder precisión en float32.

### Task 3: `control.c` con RST, motores, M3 y paro

**Files:** Modify `src/control.c/.h`, `src/config.c/.h`, `src/hal.c/.h`, `src/motion.c`, `src/uros_cmd.c/.h`, `src/comms.c`, `src/uros_comms.c`, `src/CMakeLists.txt`, `tools/host_tests/sim_control.c`, stubs de pruebas.

- [ ] Escenarios nuevos en `sim_control.c`: Home+goto con RST en las 3 formas (error < 0.8 mm); paro en Home, en falla previa y con cola llena; Home conservado; manual bloqueado tras `clear` hasta soltar dead-man; motor off en Home/goto/ang/jog, sin falla falsa; reactivar sin salto; M3 solo si activo y apagado por paro; `ctrl pid` sigue pasando los escenarios viejos.
- [ ] Pruebas de `uros_cmd` para cada comando nuevo.
- [ ] Implementar; `sh tools/host_tests/run.sh` todo OK.
- [ ] `/scara/state` añade `STATE_EN_BITS, STATE_CTRL, STATE_TOOL`. `ee_position` a 5 Hz. Compilar con `pio run`.

### Task 4: Simulaciones y reporte RST

**Files:** Create `tools/rst/simulate.py`, `docs/rst/REPORTE_RST.md`, `docs/rst/*.png`, `src/rst_defaults.h`.

- [ ] Simulación no lineal (fricción, saturación, rampa, cuantización) de las 3 formas para posición: escalón y trapezoide; tabla por eje de sobreimpulso, tiempo de asentamiento, error máximo y esfuerzo; barrido de robustez (τ ±50 %, K ±20 %).
- [ ] Elegir la mejor con criterio escrito y exportarla a `src/rst_defaults.h`.

### Task 5: Simulador ROS 2

**Files:** Create `tools/sim/sim_hal.c`, `tools/sim/build.sh`, `ros2/scara_sim_node.py`. Modify `ros2/scara_rviz.py`, `ros2/scara_rviz.launch.py`.

- [ ] Prueba de extremo a extremo: lanzar el nodo; `home`, `goto 120 80 0`; llega `#LLEGO`; `/joint_states` coincide con la cinemática inversa.
- [ ] Inyección por `/scara_sim/inject`: `lock j`, `free j`, `power 0|1`.
- [ ] `ros2 launch ros2/scara_rviz.launch.py sim:=true`; marcador de la referencia en RViz.

### Task 6: Secuencia repetitiva

**Files:** Create `ros2/scara_sequence.py`, `ros2/secuencia.yaml`, `tools/host_tests/test_sequence.py`.
**Produces:** `SequenceRunner(steps, repeat, send, clock)` con `start()`, `stop()`, `on_event(text)`, `tick()`, `load_sequence(path)->steps` (valida alcance y sintaxis).

- [ ] Pruebas: orden correcto, repeticiones, espera `wait`, aborta con `#FALLA`/`#ERROR`/timeout, no manda más tras abortar, archivo con punto inalcanzable se rechaza.
- [ ] La secuencia real es **desconocida**: el YAML trae un ejemplo marcado como provisional.

### Task 7: Panel nuevo

**Files:** Create `ros2/scara_link.py`, `ros2/scara_panel.py`, `tools/host_tests/test_link.py`, `tools/sim/bench_panel.py`.

- [ ] Pruebas: latido se suelta si la interfaz deja de responder 150 ms; decodificación de estado; bits de motores.
- [ ] Panel: banner de estado, PARO siempre visible, vista XY, gráficas, pestañas Operación / Motores / Secuencia / Control / Registro.
- [ ] Medir contra el panel Tk viejo, con el simulador: retardo teclado→latido, retardo de mensajes y tiempo de refresco. Capturas fuera de pantalla para revisar el diseño.

### Task 8: Documentos y manual

**Files:** Modify `README.md`, `UBUNTU_MICROROS.md`. Create `docs/MANUAL.md`.

- [ ] Actualizar la guía (baudios, sim, comandos, lo que falta verificar en el robot).
- [ ] Crear el manual en Notion como borrador privado y dar el enlace.

---

## Fase 2 (2026-11-03): atornillado real y PWM original

**Datos del usuario:** base en (15; 0) cm del plano GeoGebra; placas de 38 × 38 mm con centro en A(29.1421; −5), B(26.1803; −10), C(20.8788; −13.8), D(15; −15) cm, sin girar; huecos a 3,5 mm de cada borde (medidos en la imagen); origen (14,5; −20) cm, que queda a 20 cm de la base (alcance 19 cm): se usa el punto alcanzable más cercano. M3 de 200 rpm con encoder en J8 (GPIO14/13). Servo MG90S en J13 (señal por R40 hacia GPIO32) en lugar del ACS712.

**Proceso de cada hueco:** ir → soltar tornillo con el servo → atornillar con M3 → informar corriente, torque, fuerza, presión y profundidad, todo calculado con el modelo porque ya no hay sensor de corriente.

### Task 9: PWM igual al original
RST a 1 kHz, misma fricción de `pid.c`; forma nueva `orig` (b=1, c=0, s1=−τ/(τ+Ts), Kd·(1+s1), Kff+Kd). Prueba: Home + goto con PID y con RST dan el mismo PWM.

### Task 10: modelo de M3 y máquina de atornillado
`src/screw.c/.h`, C puro. `i = (V − Ke ω)/R`; torque = Kt (i − I0); fuerza = torque/(K d); presión = fuerza/A; profundidad = vueltas × paso. Fases: rápido, lento, apriete. Resultados: ok, sin apriete, trabado, tiempo, sobrecorriente.

### Task 11: firmware
Encoder 3, servo a 50 Hz, comandos `servo`, `screw`, `#TORNILLO`, estado con 8 campos nuevos, paro apaga M3 y cierra el servo. Se quita `PIN_ESTOP`: GPIO32 ahora es del servo.

### Task 12: simulación y secuencia
Planta de M3 + tornillo con inyección (`screw ok|none|jam|strip`), `proceso:` en YAML que expande a 16 huecos, validación con codo y límites articulares.

### Task 13: panel, RViz y reportes
Vista XY en el mismo marco que las placas, tabla de huecos, gráficas de M3, marcadores en RViz, `docs/m3/REPORTE_M3.md`, manual.

### Review Focus (fase 2)
1. Paro durante el atornillado o con el servo abierto.
2. Encoder de M3 sin señal: no puede reportar un tornillo bien apretado.
3. Motor 3 desactivado a mitad del proceso.
4. Tornillo ausente o barrido: la secuencia aborta y no sigue al siguiente hueco.
5. Pérdida de comunicación mientras atornilla: M3 se detiene solo.
