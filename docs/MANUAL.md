# Manual de uso del robot SCARA

Robot SCARA de dos eslabones (120 mm y 70 mm) con ESP32, controlado desde un PC con Ubuntu y ROS 2. Alcance: de 50 a 190 mm desde la base. Atornilla con un motor de 200 rpm (M3); un servo MG90S suelta cada tornillo.

## Antes de empezar: seguridad

- **Ten siempre a mano el paro físico.** Es el botón en serie con los 12 V de motores (bornera J15) y es el único que funciona aunque falle el PC o el ESP32.
- El botón **PARO DE EMERGENCIA** del panel (o la tecla **E**) depende de la comunicación. Úsalo, pero no confíes solo en él.
- Mantén las manos fuera del alcance del brazo mientras tenga corriente.
- J15 no tiene protección contra polaridad invertida: revisa + y − antes de conectar.

## Encender el robot

Usa dos terminales.

**Terminal 1, el agente que conecta el ESP32 con ROS 2:**

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 460800 -v6
```

Pulsa el botón EN del ESP32. El agente debe mostrar que se creó una sesión.

**Terminal 2, el panel y el gemelo digital:**

```bash
source /opt/ros/jazzy/setup.bash
cd scara_fw
ros2 launch ros2/scara_rviz.launch.py &     # gemelo digital en RViz
python3 ros2/scara_panel.py                 # panel de control
```

El encabezado del panel debe decir **Conectado · 20 Hz**. Si dice **SIN DATOS DEL ROBOT**, revisa el agente y el cable USB.

**Para practicar sin robot:** `python3 ros2/scara_panel.py --sim` arranca el simulador, RViz y el panel.

## El panel

![Panel de control](img/panel_operar.png)

- **Arriba:** estado del robot, posición, controlador activo y paro de emergencia, siempre visibles.
- **Izquierda:** vista superior del brazo y gráficas de medición, referencia y PWM.
- **Derecha:** pestañas Operar, Motores, Atornillador, Secuencia, Controlador y Registro.

| Estado | Qué significa |
|---|---|
| OFF | Motores apagados |
| HOMING | Buscando la referencia |
| HOLD | Manteniendo la posición |
| MOVE | Moviéndose a un punto |
| JOG | Moviéndose con el teclado |
| FALLA | Se detuvo por una falla. Hay que rearmar y repetir Home |
| PARO | Paro de emergencia activo. Hay que rearmar |

## Paso a paso

### 1. Hacer Home

Pulsa **Home** (tecla H) en la pestaña Operar. Cada eje va a un final de carrera, al contrario, y vuelve a donde estaba. Sin Home el robot no acepta órdenes de posición.

### 2. Mover a mano

Con el foco fuera de los cuadros de texto, mantén **ESPACIO** (dead-man) y usa las flechas:

| Tecla | Movimiento |
|---|---|
| ← → | J1 |
| ↑ ↓ | J2 |

Al soltar ESPACIO se detiene. Si estás escribiendo en un cuadro, el teclado no mueve el robot: pulsa Esc o haz clic afuera.

### 3. Ir a un punto

- **Ir a una posición:** escribe x, y en milímetros y pulsa **Ir**. También puedes hacer clic en la vista superior para llenar los cuadros; el clic no mueve el robot.
- **Ir a ángulos:** escribe q1, q2 en grados y pulsa **Ir**.
- **Codo:** elige cuál de las dos posturas prefieres para llegar a un punto.

### 4. Ejecutar el proceso repetitivo

![Secuencia](img/panel_secuencia.png)

`ros2/secuencia.yaml` ya trae la secuencia asignada: los 4 huecos de cada una de las 4 placas, 16 en total.

1. Haz **Home** y activa **M3** en la pestaña **Motores**. Sin M3 activo, la secuencia no arranca.
2. Carga los tornillos en el dispensador.
3. En la pestaña **Secuencia**, elige las repeticiones (0 = hasta que la detengas) y pulsa **Iniciar**.

En cada hueco, el robot:

1. Mueve el brazo al hueco y espera a que se asiente.
2. Abre el servo para soltar el tornillo y lo cierra.
3. Atornilla con M3 y guarda el resultado en la tabla de huecos.
4. Sube M3 girando al revés durante 18,5 s, las mismas 12 vueltas. Se supone que la punta se suelta sola del tornillo.

Al terminar vuelve al origen. Si un tornillo no queda bien, hay una falla o el robot rechaza un punto, la secuencia aborta y te dice por qué. Después solo apaga M3 y cierra el servo. Si M3 quedó abajo, el panel te avisa: **súbelo a mano antes de mover el brazo**. Revisa ese hueco antes de volver a iniciar: la secuencia empieza de nuevo desde el primero.

Si cambias el archivo, pulsa **Recargar**. Si el brazo no alcanza un hueco, el panel te dice cuál. Esto se ajusta en la sección `proceso`:

```yaml
proceso:
  base_cm: [15, 0]        # dónde está el eje de J1 en el plano
  origen_cm: [14.5, -20]  # si no se alcanza, se usa el punto más cercano en esa dirección
  codo: -1
  placas_cm: {A: [29.1421, -5], B: [26.1803, -10], C: [20.8788, -13.8], D: [15, -15]}
  lado_mm: 38
  hueco_borde_mm: 3.5
  servo: {abrir_s: 0.4, caida_s: 0.6}
  tornillo: {vueltas: 12, torque_nm: 0.4, subir_pct: 35, subir_s: 18.5}
```

**Ensayo sin tornillos.** Sirve para probar el recorrido sin servo ni tornillos:

1. En la pestaña **Secuencia**, marca **Ensayo**.
2. Pulsa **Iniciar**. El brazo visita los mismos huecos y, en cada uno, M3 baja 3,5 s, hace una pausa de 0,3 s y sube otros 3,5 s con la misma potencia.

Se ajusta en `ensayo: {potencia: 40, bajar_s: 3.5}`, dentro de `proceso`. M3 debe estar activo. Antes del primer ensayo, comprueba que el sentido sea el correcto (`TOOL_MOTOR_SIGN`): si está invertido, M3 subirá primero.

M3 baja y sube por tiempo, sin medir la altura, así que puede no volver exactamente al mismo punto. Si detienes el ensayo o hay un paro con M3 en movimiento, M3 no sube solo: **súbelo a mano antes de mover el brazo**. El panel te lo avisa.

También se puede escribir una lista de pasos, sin la sección `proceso`:

```yaml
repeticiones: 3
pasos:
  - ir: [150, 40]                            # x, y en mm
  - angulos: [20, 90]                        # q1, q2 en grados
  - esperar: 0.5                             # segundos
  - codo: -1
  - servo: {accion: abrir, tiempo: 0.4}      # abrir, cerrar o grados
  - atornillar: {vueltas: 12, torque: 0.4}
  - m3: {potencia: 40, tiempo: 1.5}          # motor 3 sin control: % y segundos
```

### 5. Activar o desactivar motores

![Motores](img/panel_motores.png)

En la pestaña **Motores** hay un interruptor por motor. El botón muestra lo que el robot confirma, no solo lo que pediste.

| Con un eje desactivado | Qué pasa |
|---|---|
| Driver | Libre: se puede mover a mano |
| Encoder | Sigue contando; no pierde la referencia |
| Home | Solo referencia los ejes activos |
| Ir a una posición | Se rechaza: necesita J1 y J2 |
| Ir a ángulos | Mueve solo los ejes activos |
| Al reactivar | Se queda en la posición donde esté |

**M3 (atornillador)** arranca desactivado. Actívalo, elige la potencia y pulsa **Girar**. Si el panel deja de comunicarse, M3 se apaga solo a los 10 s. Un atornillado en curso se detiene solo cuando termina o a los 20 s. Si desactivas M3 durante un atornillado, este se cancela.

### 6. Probar el atornillador

![Atornillador](img/panel_atornillador.png)

En la pestaña **Atornillador** puedes abrir y cerrar el servo y atornillar en el punto donde esté el brazo. El brazo debe estar quieto y, mientras M3 atornilla, no se mueve.

No hay sensor de corriente: corriente, torque, fuerza y presión salen del modelo del motor, del PWM y del encoder de M3. Durante el arranque, torque, fuerza y presión se muestran en 0 porque el motor está acelerando. **Son estimaciones hasta que calibres el modelo** con `docs/m3/REPORTE_M3.md`. La profundidad sale del encoder.

| Resultado | Qué significa | Qué hacer |
|---|---|---|
| `ok` | Dio las vueltas y llegó al torque | Sigue |
| `sin_apriete` | Dio todas las vueltas sin llegar al torque | Revisa si cayó el tornillo o si la rosca está barrida |
| `trabado` | Se detuvo antes de tiempo | Revisa si el tornillo entró torcido y el cable del encoder (J8) |
| `flojo` | El motor se quedó quieto antes de llegar al torque | Revisa M3 y los 12 V, y calibra el modelo |
| `tiempo` | No terminó en 20 s | Revisa la mecánica de M3 |
| `sobrecorriente` | Pasó de 2,4 A | Revisa M3 antes de repetir |
| `cancelado` | Lo detuviste o hubo un paro | Revisa el tornillo: quedó a medias |

### 7. Elegir el controlador

![Controlador](img/panel_controlador.png)

En la pestaña **Controlador** se elige entre RST PID original (por defecto, da el mismo PWM que el firmware original), RST posicional, RST incremental, RST enmascarado y el PID anterior. Se cambia con el robot en OFF o HOLD. Ahí mismo se pueden ajustar los parámetros en vivo; se pierden al reiniciar.

### 8. Apagar

Pulsa **Apagar motores** (tecla O), cierra el panel y corta los 12 V.

## Cuando algo sale mal

![Paro de emergencia](img/panel_paro.png)

**Paro de emergencia.** Frena, deja los motores libres, cancela el tornillo, cierra el servo y queda enclavado. Para continuar: revisa el robot, pulsa **Rearmar** y da la orden de nuevo. La referencia se conserva, salvo que el paro interrumpa un Home. Si mantenías el dead-man, suéltalo antes de volver a mover.

**Si la secuencia se corta con el servo abierto.** Al detenerla o si el robot informa una falla o un error, el servo se cierra solo.

**Paro físico.** Corta los 12 V de motores. El ESP32 sigue encendido y, si el brazo estaba en movimiento, declara falla porque el eje no responde.

| Mensaje | Causa probable | Qué hacer |
|---|---|---|
| Error de seguimiento | Eje trabado, encoder desconectado o signo invertido | Revisa el eje y sus cables; rearma y repite Home |
| Motor trabado | Mucho PWM y el encoder no cuenta | Revisa la mecánica y los 12 V |
| Final de carrera inesperado | Se perdió la referencia | Rearma y repite Home |
| MIN y MAX a la vez | Cableado de los finales de carrera | Revisa J9 a J12 |
| Tiempo de Home agotado | El eje no llegó al tope | Revisa el motor y el final de carrera |
| No llegó a la consigna a tiempo | Ganancias o fricción | Ajusta en la pestaña Controlador |
| SIN DATOS DEL ROBOT | Agente caído o cable USB | Revisa la terminal del agente |

## Atajos de teclado

| Tecla | Acción |
|---|---|
| ESPACIO + flechas | Mover a mano |
| E | Paro de emergencia |
| H | Home |
| S o Esc | Detener y mantener la posición |
| O | Apagar motores |
| C | Borrar falla o rearmar |
