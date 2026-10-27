#!/usr/bin/env python3
"""
Teleoperación del SCARA por teclado (Windows / Linux).  Habla con el firmware por serial.

    pip install pyserial pynput
    python tools/teleop.py --port COM5            (en Ubuntu: --port /dev/ttyUSB0)

POR QUÉ ESTE PROGRAMA y no el monitor serie: un terminal no avisa cuando SUELTAS una tecla, y un
dead-man necesita saberlo. Aquí se captura el estado real de las teclas (presionada / soltada) y se
envía al ESP32 un latido "d <dead-man> <j1> <j2>" 25 veces por segundo. Si sueltas el espacio se envía
"d 0 0 0" al instante; si este programa o el cable se caen, el firmware se detiene solo (150 ms).

TECLAS
    espacio (mantener)    DEAD-MAN: sin él las flechas no mueven nada
    ← →                   articulación 1  (− / +)
    ↓ ↑                   articulación 2  (− / +)
    h  Home               g  ir a (x y z) en mm        a  ir a ángulos (q1 q2) en grados
    s / Esc  Detener      o  Apagar motores            c  Borrar falla
    t  Telemetría on/off  e  Cambiar codo              p  Pulso de prueba de sentido (j)
    :  Escribir comando crudo (gain, cfg, sens, ...)   q  Salir
"""
import argparse
import csv
import re
import sys
import threading
import time

import serial
import serial.tools.list_ports

HEARTBEAT_HZ = 25


def heartbeat_from_keys(pressed):
    """pressed: conjunto de nombres de tecla -> (dm, j1, j2). Función pura (se prueba sin teclado)."""
    dm = 1 if "space" in pressed else 0
    j1 = (1 if "right" in pressed else 0) - (1 if "left" in pressed else 0)
    j2 = (1 if "up" in pressed else 0) - (1 if "down" in pressed else 0)
    if not dm:
        return 0, 0, 0
    return dm, j1, j2


class Console:
    """Pone la consola sin eco ni búsqueda de línea para que las flechas no ensucien la pantalla."""

    def __init__(self):
        self.saved = None
        self.win = sys.platform.startswith("win")

    def quiet(self):
        try:
            if self.win:
                import ctypes
                k = ctypes.windll.kernel32
                h = k.GetStdHandle(-10)
                m = ctypes.c_uint32()
                k.GetConsoleMode(h, ctypes.byref(m))
                if self.saved is None:
                    self.saved = m.value
                k.SetConsoleMode(h, m.value & ~0x0006)       # sin ENABLE_ECHO_INPUT ni ENABLE_LINE_INPUT
            else:
                import termios
                if self.saved is None:
                    self.saved = termios.tcgetattr(sys.stdin)
                a = termios.tcgetattr(sys.stdin)
                a[3] &= ~(termios.ECHO | termios.ICANON)
                termios.tcsetattr(sys.stdin, termios.TCSANOW, a)
        except Exception:
            pass

    def normal(self):
        try:
            if self.win:
                import ctypes
                if self.saved is not None:
                    ctypes.windll.kernel32.SetConsoleMode(ctypes.windll.kernel32.GetStdHandle(-10), self.saved)
            else:
                import termios
                if self.saved is not None:
                    termios.tcsetattr(sys.stdin, termios.TCSANOW, self.saved)
        except Exception:
            pass

    def flush_input(self):
        try:
            if self.win:
                import msvcrt
                while msvcrt.kbhit():
                    msvcrt.getwch()
            else:
                import termios
                termios.tcflush(sys.stdin, termios.TCIFLUSH)
        except Exception:
            pass


class Teleop:
    def __init__(self, ser, log_path=None):
        self.ser = ser
        self.wlock = threading.Lock()
        self.pressed = set()
        self.typing = False
        self.running = True
        self.last_sent = None
        self.console = Console()
        self.status_visible = False
        self.log = None
        if log_path:
            self.log = csv.writer(open(log_path, "a", newline=""))
            self.log.writerow(["t_pc", "tx", "ty", "tz", "x_enc", "y_enc", "ex", "ey", "q1_deg", "q2_deg"])
            self.log_file = None

    # ---- envío
    def send(self, line):
        with self.wlock:
            try:
                self.ser.write((line + "\n").encode())
            except serial.SerialException:
                self.running = False

    def send_heartbeat(self, force=False):
        hb = heartbeat_from_keys(self.pressed) if not self.typing else (0, 0, 0)
        self.send("d %d %d %d" % hb)
        self.last_sent = hb

    # ---- lectura del ESP32
    def reader(self):
        buf = b""
        while self.running:
            try:
                data = self.ser.read(256)
            except serial.SerialException:
                self.running = False
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self.show(line)

    def show(self, line):
        if line.startswith("ST,"):
            f = line.split(",")
            try:
                txt = ("%-6s home=%s%s  q1=%8.2f° q2=%8.2f°  x=%7.1f y=%7.1f mm  fc=%s falla=%s" %
                       (f[1], f[2], f[3], float(f[4]), float(f[5]), float(f[6]), float(f[7]), f[9], f[8]))
            except (IndexError, ValueError):
                return
            sys.stdout.write("\r" + txt + "   ")
            sys.stdout.flush()
            self.status_visible = True
            return
        if self.status_visible:
            sys.stdout.write("\r" + " " * 110 + "\r")
            self.status_visible = False
        print(line)
        if line.startswith("#LLEGO") and self.log:
            v = re.findall(r"[-+]?\d+\.?\d*", line.split(":", 1)[1])
            if len(v) == 9:          # goto: objetivo x y z | medido x y | error ex ey | q1 q2
                self.log.writerow([time.time()] + v)

    # ---- teclado
    def prompt(self, text):
        self.typing = True
        self.send("d 0 0 0")
        self.console.normal()
        self.console.flush_input()
        try:
            return input(text)
        finally:
            self.console.quiet()
            self.console.flush_input()
            self.typing = False

    def on_press(self, key):
        from pynput import keyboard
        name = self.keyname(key, keyboard)
        if self.typing or name is None:
            return
        self.pressed.add(name)
        if name in ("space", "left", "right", "up", "down"):
            self.send_heartbeat()
            return
        if name == "esc":
            self.send("stop")
        elif name == "q":
            self.running = False
        elif name in ("h", "s", "o", "c", "t", "e", "g", "a", "p", ":"):
            threading.Thread(target=self.atajo, args=(name,), daemon=True).start()

    def on_release(self, key):
        from pynput import keyboard
        name = self.keyname(key, keyboard)
        self.pressed.discard(name)
        if name in ("space", "left", "right", "up", "down"):
            self.send_heartbeat()          # soltar = parar de inmediato

    @staticmethod
    def keyname(key, keyboard):
        special = {keyboard.Key.space: "space", keyboard.Key.left: "left", keyboard.Key.right: "right",
                   keyboard.Key.up: "up", keyboard.Key.down: "down", keyboard.Key.esc: "esc"}
        if key in special:
            return special[key]
        ch = getattr(key, "char", None)
        return ch.lower() if ch else None

    def atajo(self, k):
        self.tele_on = getattr(self, "tele_on", False)
        if k == "h":
            self.send("home")
        elif k == "s":
            self.send("stop")
        elif k == "o":
            self.send("off")
        elif k == "c":
            self.send("clear")
        elif k == "t":
            self.tele_on = not self.tele_on
            self.send("tele %d" % self.tele_on)
        elif k == "e":
            self.elbow = -getattr(self, "elbow", 1)
            self.send("elbow %+d" % self.elbow)
        elif k == "g":
            v = self.prompt("\ngoto  x y z [mm]: ")
            self.send("goto " + v)
        elif k == "a":
            v = self.prompt("\nang   q1 q2 [grados]: ")
            self.send("ang " + v)
        elif k == "p":
            v = self.prompt("\npulso de prueba en eje (1 o 2): ")
            self.send("pulse " + v)
        elif k == ":":
            v = self.prompt("\ncomando> ")
            self.send(v)

    # ---- bucle principal
    def run(self):
        from pynput import keyboard
        threading.Thread(target=self.reader, daemon=True).start()
        self.console.quiet()
        print(__doc__)
        self.send("\n")
        self.send("\x1b")                 # sale de cualquier monitor/entrada que haya quedado abierta
        time.sleep(0.1)
        self.send("fmt 1")                 # modo maquina: sin eco ni atajos
        self.send("tele 1")
        self.tele_on = True
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release):
            try:
                while self.running:
                    self.send_heartbeat()
                    time.sleep(1.0 / HEARTBEAT_HZ)
            except KeyboardInterrupt:
                pass
        self.send("d 0 0 0")
        self.send("stop")
        self.send("fmt 0")                 # devuelve el modo texto para usar el monitor serie
        self.console.normal()
        print("\nSaliendo (motores en HOLD; usa 'o' o cierra el cable para apagarlos).")


def main():
    ap = argparse.ArgumentParser(description="Teleoperación del SCARA por teclado")
    ap.add_argument("--port", help="COM5, /dev/ttyUSB0, ... (si se omite, lista los puertos)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", help="archivo CSV donde guardar los eventos #LLEGO (caracterización)")
    args = ap.parse_args()
    if not args.port:
        print("Puertos disponibles:")
        for p in serial.tools.list_ports.comports():
            print("  ", p.device, "-", p.description)
        print("Vuelve a ejecutar con --port <puerto>")
        return 1
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = args.port, args.baud, 0.05
    ser.dtr = False        # se fijan ANTES de abrir: en muchas DevKit evita reiniciar la ESP32 al conectar
    ser.rts = False
    ser.open()
    Teleop(ser, args.log).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
