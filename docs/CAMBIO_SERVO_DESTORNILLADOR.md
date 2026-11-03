# Cambio del servo: subir y bajar el destornillador

Fecha: 2026-11-03.

## Lo que se pidió

1. El servo no es para dejar pasar el tornillo. Es para subir el destornillador y que baje el tornillo: cuando el tornillo debe bajar, se sube el destornillador y luego se vuelve a bajar.
2. El servo se mueve de -180° a 180° y siempre debe estar en 0° al inicio.
3. Como el destornillador también baja, el prismático de M3 no puede bajar mucho, porque casi tocaría el suelo.
4. Indicar qué cambios necesita la PCB para que el servo funcione bien en el lugar donde va, y cómo se conecta.

## Lo que había que hacer

- Cambiar la lógica de "abrir y cerrar" por "subir y bajar" en el firmware, la secuencia, el panel, las pruebas y la documentación.
- Ampliar el rango del servo a -180° a 180°, con 0° como posición de arranque.
- Evitar que M3 baje más de lo que permite la altura disponible.
- Revisar la PCB de la carpeta `Version8PCB` (`J13`, `R40`, `R41`, `C14`) y explicar los cambios de soldadura y el cableado.

## Lo que se hizo

**Servo**

- El pulso de 500 a 2400 µs ahora corresponde a -180° a 180°, y 0° es 1450 µs (`src/config.h`, `src/hal.c`).
- Los comandos `servo abrir` y `servo cerrar` pasan a `servo subir` y `servo bajar`, también en los botones del panel. `servo <grados>` acepta ángulos negativos.
- En cada hueco la secuencia sube el destornillador, espera a que caiga el tornillo, lo baja, atornilla y sube M3 (`ros2/scara_sequence.py`, `ros2/secuencia.yaml`).

**Seguridad añadida**

- No se atornilla con el destornillador arriba.
- El destornillador no sube mientras M3 atornilla.
- Con un paro o una falla, el servo vuelve a 0° y no acepta subir.

**Bajada de M3**

- `ros2/secuencia.yaml` tiene `m3: {paso_mm, bajada_max_mm}`. El archivo se rechaza si el tornillo (vueltas + 2, donde corta el firmware) o el ensayo bajarían más que `bajada_max_mm`.
- El ensayo se calcula con el modelo de M3 sin carga, así que es una estimación.

## PCB

No hace falta rediseñarla: son tres cambios de soldadura. Las posiciones están en mm desde la esquina superior izquierda, vista desde arriba.

| Pieza | Cambio | Motivo |
|---|---|---|
| `R40` (10 kΩ), en (59,4; 14,0) y (59,4; 24,2) | Cambiar por 330 Ω | 10 kΩ era para el ACS712 y puede dejar débil la señal del servo |
| `C14` (100 nF), en (54,3; 24,0) y (54,3; 29,0) | Retirar | Era el filtro del sensor; ahora solo carga a GPIO32 en cada pulso |
| `R41` (22 kΩ), en (64,5; 18,8) y (64,5; 29,0) | Dejar | Mantiene la señal en bajo mientras el ESP32 reinicia |
| Condensador nuevo de 470 µF / 16 V | Soldar por abajo: + al pin 1 de `J13`, − al pin 3 | El servo comparte los 5 V con el ESP32 y los encoders |

Si el ACS712 está montado, hay que quitarlo y conectar el cable rojo de M3 directo a `J5`.

## Conexión del servo en J13

Vista desde arriba, de izquierda a derecha:

| Pin de `J13` | Señal | Cable típico del servo |
|---|---|---|
| 3 (izquierda) | GND | Marrón |
| 2 (centro) | Señal (GPIO32, por `R40`) | Naranja |
| 1 (derecha) | +5 V | Rojo |

El servo necesita los 12 V en `J1`: solo con el USB no recibe 5 V, porque `D2` bloquea ese camino. Antes de conectarlo, hay que medir que el LM2596 entregue 5,0 V.

## Pendiente

**Valores por medir**

- `m3.paso_mm` y `m3.bajada_max_mm`. Los valores actuales (0,5 mm por vuelta y 7 mm) no están medidos y solo dejan pasar lo que ya hacía, así que **todavía no reducen la bajada de M3**.
- `SERVO_ARRIBA_DEG` (ángulo y signo). El valor actual, 90°, es provisional.
- Rango real del servo. Con `servo 90` debe girar 90°. Si gira unos 45°, su rango real es de -90° a 90° y hay que cambiar `SERVO_GRADOS_MIN` y `SERVO_GRADOS_MAX`. Un servo de giro continuo no sirve, porque no mantiene la posición.
- `arriba_s` y `bajar_s` en `ros2/secuencia.yaml`. Los valores de 0,6 s son provisionales.

**Supuestos por confirmar**

- 0° corresponde al destornillador abajo.
- Con un paro o una falla, el destornillador baja solo. Debe decidirse si se prefiere que se quede quieto.

**Verificación**

- Pasan las 65 pruebas de `tools/host_tests/test_sequence.py`.
- En el PC donde se hizo el cambio no había `gcc`, ROS 2 ni Qt: el código C solo pasó revisión de sintaxis, y faltan por ejecutar `tools/host_tests/run.sh` y `tools/sim/test_e2e.py`.
- `tools/m3/report.py` tampoco se ejecutó: `docs/m3/REPORTE_M3.md` se editó a mano.
- El firmware completo no se compiló, porque `components/micro_ros_espidf_component` está vacío en el clon.
- Nada se ha probado en el robot.

**Sin actualizar**

- Los archivos de KiCad, donde `J13` sigue rotulado como ACS712.
- `Guia_operacion_SCARA_ROS2.pdf`, que puede seguir describiendo el servo anterior.

**Límite de la protección**

- `bajada_max_mm` solo aplica a la sección `proceso`. El comando `tool` manual y los pasos `m3` escritos a mano no están limitados.
