# Cambios del servo, M3 y la calibración

Fecha: 2026-11-03. Resume lo que se pidió y cómo quedó el robot después de probarlo.

## Lo que se pidió

1. El servo no deja pasar el tornillo: mueve el destornillador. Va de -180° a 180° y siempre arranca en 0°.
2. M3 no puede bajar más de lo medido, porque casi tocaría el suelo.
3. Indicar los cambios de la PCB para el servo y cómo se conecta.
4. Medir a mano M3 y el servo, y usar esas medidas.
5. Pasar el proceso a un proyecto de PlatformIO sin ROS, probarlo y calibrarlo en el robot.
6. Poner las PCB en una fila recta al frente, dos a cada lado, y poder elegir cuál hacer.
7. Traer todo de vuelta a ROS y actualizar el README y el manual.

## Secuencia de cada hueco

1. El brazo va al hueco en línea recta y espera a que se asiente.
2. El servo sube a +180°: baja el tornillo.
3. M3 baja hasta 6 mm.
4. El servo baja a -180°: baja el destornillador.
5. M3 sube 1/4 de la barra, hasta 31 mm.
6. El servo vuelve a 0°.

Al terminar, el brazo va a `0 0`. **La punta no gira, así que el proceso todavía no atornilla.** Se probó un motor de 5 V en J12 y se descartó.

## Medidas y calibración

| Dato | Valor | Origen |
|---|---|---|
| M3, tope de arriba | 106 mm | Medido con `g` |
| M3, abajo | 6 mm, a 24,855 vueltas del tope | Medido con `g` |
| Avance de M3 | ~4,02 mm por vuelta | Calculado de lo anterior |
| Servo | +180° baja el tornillo, -180° baja el destornillador | Confirmado en el robot |
| L1 y L2 | 125 mm y 105 mm, hasta la punta | Medidos con regla |
| J1 | 8400 cuentas por vuelta | Ajustado con puntos medidos; coincide con un reductor 131:1 |
| J2 | ~7350 cuentas por vuelta | Ajustado con puntos medidos; menos seguro que J1 |
| Cero de J1 y J2 | 1838 y 2376 cuentas desde su primer tope | Ajustado con puntos medidos |

Sentidos medidos: con PWM positivo M3 baja y las vueltas suben (`TOOL_MOTOR_SIGN` y `TOOL_ENC_SIGN` en -1).

## Posición de las PCB

Fila recta con 30 mm entre placas. Centros en mm desde el eje de J1 (+x: brazo recto; +y: izquierda):

| PCB | x | y |
|---|---|---|
| 1 (A) | 170 | 102 |
| 2 (B) | 170 | 34 |
| 3 (C) | 170 | -34 |
| 4 (D) | 170 | -102 |

## Control

- **PID por defecto:** el RST se movía muy fuerte en el robot. Se puede volver a probar con `ctrl rst`.
- **Línea recta:** hacia un punto x, y, la punta va en línea recta a 20 mm/s.
- **Fricción:** en movimiento se compensa según el sentido, para que no vaya a tirones. Quieto, un empuje crece a ritmo fijo hasta soltar el eje y se retira al moverse.
- **Home:** mide los dos topes de cada eje, cuenta el cero desde el primero y termina en `0 0`.
- **M3:** `m3 home` busca el tope de arriba y `m3 ir` lo lleva a una altura con el encoder. El firmware se detiene si M3 no avanza durante 0,5 s.

## Conexión del servo en J13

Vista desde arriba, de izquierda a derecha: pin 3 GND (marrón), pin 2 señal (naranja), pin 1 +5 V (rojo). Cambios en la placa: R40 a 330 Ω, retirar C14, dejar R41 y poner 470 µF / 16 V entre los pines 1 (+) y 3 (-). Necesita los 12 V en J1.

## Qué se comprobó y qué no

- **En el robot, con el firmware sin ROS:** el proceso completo de los 16 huecos, con la calibración final.
- **Después de esa prueba, sin probar en el robot:** la selección de PCB y mover M3 desde el lazo de control.
- **Con ROS, sin probar en el robot:** todo el camino por micro-ROS y el panel.
- **Pruebas en el PC:** las de la secuencia en Python pasan. El firmware en modo texto compila. El resto del C (incluido el modo micro-ROS) y el panel solo pasaron revisión de sintaxis, porque el PC usado no tiene `gcc`, ROS 2 ni Qt, así que `tools/host_tests/run.sh` y `tools/sim/test_e2e.py` siguen sin ejecutarse con estos cambios.
- **Sin actualizar:** los archivos de KiCad, donde `J13` sigue como ACS712, y `Guia_operacion_SCARA_ROS2.pdf`.
