# Lab 4 en el robot: observadores con el motor 1

Versión **mínima**, separada del firmware del SCARA y sin micro-ROS. Solo el motor 1 y lo que pide la guía del Laboratorio 4:

- Velocidad medida, **Luenberger**, **Kalman a priori** y **Kalman a posteriori**.
- Corriente estimada de cada observador (la corriente no se mide).
- Covarianza **P** (a priori y a posteriori) y ganancia **K** del Kalman.
- PWM de −100 a 100 %, con botones para **parar** y **máximo**.

| Archivo | Qué es |
|---|---|
| `src/main.c` | Firmware del ESP32: PWM del motor 1, encoder, Luenberger y Kalman cada 10 ms |
| `lab4_panel.py` | Programa del PC: gráficas en vivo, deslizador y botones de PWM, CSV |

## Pasos en Windows

1. **Copia la carpeta** `lab4_motor1` a una ruta corta y sin espacios, por ejemplo `C:\lab4_motor1`. ESP-IDF falla con rutas largas o con espacios.
2. **Carga el firmware.** En VS Code con PlatformIO: *File → Open Folder* → `C:\lab4_motor1`, conecta el ESP32 y pulsa **→ Upload** en la barra azul de abajo. La primera vez tarda varios minutos. Si no encuentra el puerto, quita el `;` de `upload_port` en `platformio.ini` y pon tu puerto (por ejemplo `COM5`).
3. **Cierra el monitor serie** de PlatformIO si lo abriste: el puerto COM solo lo puede usar un programa a la vez.
4. **Instala lo de Python** (una sola vez) en PowerShell o CMD:
   ```
   py -m pip install pyserial matplotlib
   ```
5. **Abre las gráficas:**
   ```
   cd C:\lab4_motor1
   py lab4_panel.py
   ```
   Busca solo el puerto del ESP32. Si no lo encuentra, mira el número en el *Administrador de dispositivos → Puertos (COM y LPT)* y usa `py lab4_panel.py --port COM5`.

Sin el robot: `py lab4_panel.py --demo` simula el motor con el mismo modelo.

<details>
<summary>En Linux</summary>

```bash
sudo apt install python3-matplotlib python3-serial python3-tk
cd lab4_motor1 && pio run -t upload
python3 lab4_panel.py            # o --port /dev/ttyUSB0
```
</details>

**Botones:** PARAR (0 %) · MÁXIMO (100 %) · 50 % · −100 % · Reiniciar observadores · Grabar CSV · Pausar.
**Teclas:** `espacio` parar · `m` máximo · `r` reiniciar observadores.

## Seguridad

- El motor 1 mueve el **eslabón 1**, que solo gira unos ±120°. A 100 % llega al final de carrera en menos de medio segundo. El firmware **frena** al tocar el final de carrera y después solo deja girar hacia el otro lado. Para tenerlo girando a velocidad constante, como en la guía, **desacopla el eslabón** o prueba con el motor suelto.
- Prueba primero con 20–30 % para comprobar que el final de carrera lo detiene.
- Si el PWM pasa de 34 % y el encoder no cuenta durante 0,3 s (motor trabado), se detiene.
- Si el programa del PC se cierra o se desconecta, el motor se detiene en 1,5 s.
- Rampa de PWM: de 0 a 100 % en 0,34 s.
- Si la corriente estimada pasa de 1 A durante 0,3 s con el PWM estable (por ejemplo, frenándolo con la mano a mucho PWM), se detiene. El TB6612 aguanta unos 1,2 A continuos: **frena con la mano solo con PWM de 50 % o menos** y por poco tiempo.

## Datos que manda el ESP32

Una línea cada 20 ms (los observadores calculan cada 10 ms; se envía una de cada dos muestras), a 115200 baudios:

```
D,t_ms,PWM_%,V,w_medida,w_L,i_L,w_Kpri,i_Kpri,w_K,i_K,P11,P22,P11_pri,P22_pri,K1,K2
```

Unidades: velocidades en rpm, corrientes en mA, `P11` en mA², `P22` en rpm², `K1` en mA/rpm. El CSV guarda lo mismo.

## Modelo y observadores

Todo está referido al eje de salida, con discretización ZOH y Ts = 10 ms.

**Por defecto: modelo corregido** (`MODELO_CORREGIDO 1` en `src/main.c`).

- **Motor:** Ra = 2,2 Ω, tomada de la corriente de bloqueo de la hoja de datos (5,5 A a 12 V). La = 2,64 mH, Kv = 0,966 V·s/rad, B = 0,0161 N·m·s/rad, J = 0,0477 kg·m².
- **Lo que conserva del laboratorio:** Kv, B y J se recalcularon para mantener 114,3 rpm y 0,2 A a 12 V y la misma constante de tiempo (0,108 s). La **velocidad responde igual** que con el modelo del laboratorio: misma ganancia de 0,9975 rad/s por V y misma τ.
- **Por qué se cambió:** con Ra = 50 Ω la corriente nunca puede pasar de 12/50 = 0,24 A, así que al frenar el motor con la mano la corriente estimada casi no subía. Con Ra = 2,2 Ω sí sube, como la real. Los datos de velocidad del laboratorio no distinguen entre una Ra y otra; la corriente de bloqueo sí.
- **Luenberger:** polos 0,1 (doble), L = [0,1241; 0,7112]. Con 0,25 ± 0,25j y este modelo, la L de corriente sale de 5,3 y, ante una carga, la estimación de corriente se va al lado contrario (en simulación: −470 mA cuando la real es +684 mA).
- **Kalman:** R = (0,81 rpm)², Q = diag(10 mA, 1 rpm)², P0 = diag(0,1 A, 1 rpm)². Con σw = 1 rpm el filtro sigue a la velocidad medida cuando hay una carga que el modelo no conoce. En simulación, frenando el motor a 50 % de PWM: corriente real 684 mA, estimada 629 mA.

**Modelo del laboratorio tal cual** (`MODELO_CORREGIDO 0`):

- **Motor:** Ra = 50 Ω, La = 0,060 H, Kv = 0,167 V·s/rad, B = 2,79e-3 N·m·s/rad, J = 3,63e-4 kg·m².
- **Luenberger:** polos 0,25 ± 0,25j, L = [0,2436; 0,4120].
- **Kalman:** Q = diag(10 mA, 0,30 rpm)².

**Comprobación:** el código del observador del firmware da los mismos números que la implementación en Python con la que se validó el laboratorio (diferencia máxima de 5×10⁻⁶).

**Driver:** si usas un L298N en lugar del TB6612, cambia en `src/main.c`: `VGAIN 0.9782f` y `V0 1.197f`.

En modo `--demo`, la tecla `f` simula que frenas el motor con la mano.
