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

## Datos que manda el ESP32

Una línea cada 20 ms (los observadores calculan cada 10 ms; se envía una de cada dos muestras), a 115200 baudios:

```
D,t_ms,PWM_%,V,w_medida,w_L,i_L,w_Kpri,i_Kpri,w_K,i_K,P11,P22,P11_pri,P22_pri,K1,K2
```

Unidades: velocidades en rpm, corrientes en mA, `P11` en mA², `P22` en rpm², `K1` en mA/rpm. El CSV guarda lo mismo.

## Modelo y observadores

Son los del Laboratorio 4, referidos al eje de salida:

- **Motor:** Ra = 50 Ω, La = 0,060 H, Kv = 0,167 V·s/rad, B = 2,79e-3 N·m·s/rad, J = 3,63e-4 kg·m².
- **Discretización:** ZOH con Ts = 10 ms.
- **Luenberger:** polos 0,25 ± 0,25j, L = [0,2436; 0,4120].
- **Kalman:** R = (0,81 rpm)², Q = diag(10 mA, 0,30 rpm)², P0 = diag(0,1 A, 1 rpm)².

El código del observador se comprobó en el PC contra la implementación ya validada con el laboratorio; la diferencia máxima es de 6×10⁻⁶.

Si tu driver es un L298N en lugar del TB6612, cambia en `src/main.c`: `VGAIN 0.9782f` y `V0 1.197f`.
