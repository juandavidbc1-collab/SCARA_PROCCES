# SCARA con micro-ROS en Ubuntu (ROS 2 Jazzy)

Pasa el firmware del protocolo de texto a **micro-ROS**: el ESP32 es un nodo ROS 2 (`/scara/...`) y el
PC con Ubuntu habla con él por un **agente micro-ROS**. Solo cambia la capa de comunicación
(`uros_comms.c` en lugar de `comms.c`); el control, la cinemática inversa en C y la seguridad son los mismos.

```
 Python / ROS 2 (Jazzy)            Agente micro-ROS                 ESP32 (micro-ROS cliente)
 scara_client.py  ──DDS──▶  micro_ros_agent  ──XRCE-DDS──▶  /scara/goto  → IK (C) → control 1 kHz
 scara_teleop_ros.py        (USB serial ó WiFi)             /scara/jog   → dead-man (150-300 ms)
```

## 0. Estado de verificación (léelo primero)

| Parte | Estado |
|---|---|
| Lógica de comandos, validación de consignas, cinemática inversa, control | **Probado** (simulación en el PC y robot real en modo texto) |
| `uros_comms.c` / `uros_serial_transport.c` | **Compilan** contra cabeceras que replican las firmas reales de `rclc` y `rmw_microxrcedds` (rama jazzy). **No se han ejecutado con micro-ROS real** |
| Compilar micro-ROS dentro de PlatformIO + ESP-IDF 5.3 | **Sin verificar** (la rama jazzy del componente declara pruebas con ESP-IDF 5.2 y 5.3; PlatformIO usa 5.3) |
| `scara_client.py`, `scara_teleop_ros.py` | Lógica probada con ROS simulado; **no corridos con ROS 2 real** |

La primera vez algo puede fallar. La sección 8 lista los fallos probables y la salida de emergencia
(ESP-IDF nativo con `idf.py`).

## 1. Qué instalar en Ubuntu 24.04

```bash
# ROS 2 Jazzy: sigue https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html  (ros-jazzy-desktop)
sudo apt install git cmake make python3-pip python3-venv curl

# permisos del puerto USB (cerrar sesión y volver a entrar después)
sudo usermod -aG dialout $USER

# agente micro-ROS: la forma que documenta el componente es Docker
sudo apt install docker.io && sudo usermod -aG docker $USER      # (cerrar sesión y volver a entrar)

# PlatformIO (CLI). En Ubuntu 24.04 'pip install --user' está bloqueado (PEP 668): usa pipx
sudo apt install pipx && pipx ensurepath && pipx install platformio      # o la extensión de VS Code
# reglas udev de PlatformIO y, si el puerto ttyUSB0 desaparece al conectar, quitar brltty
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt remove brltty
```

## 2. Traer el firmware y el componente micro-ROS

```bash
cd ~/scara_fw
mkdir -p components
git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git components/micro_ros_espidf_component
```

Dependencias de Python que el componente necesita **dentro del entorno de ESP-IDF de PlatformIO**
(no en el sistema). PlatformIO crea ese entorno al compilar por primera vez; si no existe, compila una vez el
modo texto (`pio run`) y luego:

```bash
ls ~/.platformio/penv/                                   # busca la carpeta  .espidf-5.3.x  (la ruta puede variar)
~/.platformio/penv/.espidf-5.3.*/bin/pip install catkin_pkg lark colcon-common-extensions "empy==3.3.4"
```

> Si esa carpeta no existe, instala lo mismo en `~/.platformio/penv/bin/pip`.

## 3. Elegir el modo y compilar

```bash
python3 tools/uros_select.py serial        # micro-ROS por el cable USB (primera prueba)
pio run -t fullclean
pio run -t upload                          # la 1.ª vez tarda 10-20 min: descarga y compila micro-ROS
```

**IMPORTANTE:** compila en una terminal **sin** `source /opt/ros/jazzy/setup.bash`. El componente
recompila partes de ROS 2 y se confunde si ya hay un ROS cargado.

`uros_select.py` deja coherentes `sdkconfig.defaults` y `app-colcon.meta`:
- consola y logs apagados (UART0 pasa a ser del transporte serial),
- transporte `custom` (serial) y límites de entidades micro-ROS (este nodo usa 3 publicadores y 3 suscriptores).

En modo micro-ROS por serial **el monitor de PlatformIO ya no sirve** (UART0 es del transporte).
Para depurar con texto vuelve a `python3 tools/uros_select.py text`.

## 4. Levantar el agente y probar

```bash
# cerrar cualquier monitor serie abierto; el puerto es de uno solo
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200 -v6
```

Resetea el ESP32 (botón EN). El agente debe mostrar que se creó una sesión y los tópicos. En otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list                          # /scara/scara_esp32
ros2 topic list                         # /scara/goto /scara/jog /scara/cmd /scara/state /scara/ee_position /scara/event
ros2 topic echo /scara/state            # 13 números, 20 Hz
ros2 topic echo /scara/event            # eventos (#LLEGO, #HOME, #ERROR, ...)
```

## 5. Usar el robot desde Python / ROS 2

```bash
source /opt/ros/jazzy/setup.bash
cd ~/scara_fw
python3 ros2/scara_client.py home                  # Home (tope, tope contrario y vuelve), espera el final
python3 ros2/scara_client.py goto 120 80 0         # consigna (x, y, z) en mm  → IK en C → movimiento
python3 ros2/scara_client.py goto 400 0            # inalcanzable: responde #ERROR con el alcance
python3 ros2/scara_client.py ang 20 90
python3 ros2/scara_client.py cmd "gain"            # cualquier comando (help)
python3 ros2/scara_client.py monitor               # estado y eventos en vivo
```

También desde la terminal de ROS, sin Python propio:

```bash
ros2 topic pub --once /scara/goto geometry_msgs/msg/Point "{x: 120.0, y: 80.0, z: 0.0}"
ros2 topic pub --once /scara/cmd  std_msgs/msg/String "{data: 'home'}"
```

**Orden obligatorio** (el firmware lo exige): `home` primero; `goto` sin Home se rechaza.

### Manual con teclado y dead-man

```bash
python3 ros2/scara_teleop_ros.py            # ESPACIO (mantener) + flechas;  h Home, s detener, o apagar, c borrar falla, q salir
```

- Con sesión **X11** y `pynput` detecta de verdad cuándo sueltas una tecla.
- En **Wayland** (el predeterminado en Ubuntu 24.04) o por SSH usa el modo de terminal automáticamente: más
  "a tirones" pero igual de seguro. Para que sea más suave: `xset r rate 200 40` (solo X11) o elige
  "Ubuntu on Xorg" al iniciar sesión.
- El dead-man **vive en el firmware**: sin latidos `/scara/jog` con `x = 1` durante 300 ms (`DEADMAN_TIMEOUT_MS`
  en modo ROS 2) el robot se detiene. Si se cae el agente, también.
- Mismo mensaje a mano: `ros2 topic pub -r 25 /scara/jog geometry_msgs/msg/Vector3 "{x: 1.0, y: 1.0, z: 0.0}"`
  (mueve J1 mientras el comando siga publicando; Ctrl+C = para).

## 6. Interfaz ROS 2 (resumen)

| Tópico | Tipo | Dirección | Contenido |
|---|---|---|---|
| `/scara/goto` | `geometry_msgs/Point` | PC → ESP32 | x, y, z [mm] (z fija: `Z_FIJO_MM`, tolerancia `Z_TOL_MM`) |
| `/scara/jog` | `geometry_msgs/Vector3` (best-effort) | PC → ESP32 | x = dead-man (>0.5), y = dir J1, z = dir J2 (−1/0/+1), a 25 Hz |
| `/scara/cmd` | `std_msgs/String` | PC → ESP32 | `home`, `stop`, `off`, `clear`, `ang a b`, `elbow ±1`, `gain …`, `pulse 1`, `status`, `help` |
| `/scara/state` | `std_msgs/Float32MultiArray` (best-effort) | ESP32 → PC | `[q1°, q2°, x, y, modo, falla, home1, home2, bits_topes, qref1°, qref2°, duty1, duty2]`, 20 Hz |
| `/scara/ee_position` | `geometry_msgs/Point` (best-effort) | ESP32 → PC | posición medida del efector [mm] |
| `/scara/event` | `std_msgs/String` | ESP32 → PC | `#LLEGO …`, `#HOME …`, `#ERROR …`, `#FALLA …`, `#INFO …` |

`modo`: 0 OFF, 1 HOMING, 2 HOLD, 3 MOVE, 4 JOG, 5 PULSE, 6 FAULT.

## 7. Por WiFi (sin cable USB) — siguiente fase

```bash
python3 tools/uros_select.py wifi --ssid MI_RED --password MI_CLAVE --agent-ip 192.168.1.50
pio run -t fullclean && pio run -t upload
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6
```

- `--agent-ip` es la IP del PC con Ubuntu en esa red. El ESP32 y el PC deben estar en la misma red; el
  firewall del PC debe permitir UDP 8888 (`sudo ufw allow 8888/udp`).
- Con WiFi la consola USB queda libre para depurar. Esto es lo que habilita luego el control desde una
  página web (un puente ROS 2 ↔ web, p. ej. rosbridge, sin tocar el firmware).
- La clave WiFi queda en `sdkconfig.defaults`: no lo subas a un repositorio público.

## 8. Si algo falla

| Síntoma | Causa probable / qué hacer |
|---|---|
| El build falla pronto con `colcon: command not found` o falta `catkin_pkg`/`lark` | Faltan dependencias en el entorno `.espidf-*` de PlatformIO (sección 2) |
| `AttributeError: module 'em' has no attribute 'RAW_OPT'` | `pip install "empy==3.3.4"` en ese mismo entorno |
| Errores raros de CMake/ROS durante el build | Compilaste con ROS cargado: abre una terminal nueva y no hagas `source` de ROS |
| `Kconfig`: no existe `SCARA_COMMS_UROS` o `MICRO_ROS_*` | PlatformIO no encontró `src/Kconfig.projbuild` o el componente. Confirma `components/micro_ros_espidf_component/` y `pio run -t fullclean`. Si PlatformIO no lo soporta, usa la salida de emergencia |
| El agente no muestra sesión | Baudios distintos (`-b` del agente vs `CONFIG_SCARA_UROS_BAUD`), puerto equivocado, monitor serie abierto, o el docker sin `--privileged`/`-v /dev:/dev` |
| Aparece el nodo pero `goto` no hace nada | Falta Home (`home`) o hay una falla (`clear`); mira `/scara/event` |
| Se reinicia solo | Pila de la tarea micro-ROS corta: sube `CONFIG_SCARA_UROS_STACK` (menuconfig) |
| Con WiFi el robot se detiene a ratos | Pérdida de latidos del dead-man por WiFi débil: es el comportamiento seguro; acerca el AP |

### Salida de emergencia: ESP-IDF nativo (sin PlatformIO)

Si PlatformIO no logra compilar micro-ROS, el mismo código sirve con ESP-IDF 5.3 oficial: crea un proyecto
IDF, copia `src/` como componente `main` (renombra la carpeta a `main`), pon el componente micro-ROS en
`components/` y usa `idf.py set-target esp32 && idf.py build flash`. Los `sdkconfig.defaults` y
`app-colcon.meta` que genera `tools/uros_select.py` se usan igual.

## 9. Volver al protocolo de texto (Windows, monitor serie, teleop.py)

```bash
python3 tools/uros_select.py text
pio run -t fullclean && pio run -t upload
```
