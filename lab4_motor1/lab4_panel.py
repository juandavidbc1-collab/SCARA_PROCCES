#!/usr/bin/env python3
"""
Lab 4 en el robot: gráficas de los observadores de Luenberger y Kalman del MOTOR 1 y control del PWM.

Windows:  py lab4_panel.py                  (busca solo el puerto COM del ESP32)
          py lab4_panel.py --port COM5
          py lab4_panel.py --demo           (sin robot: simula el motor con el mismo modelo)
Linux:    python3 lab4_panel.py [--port /dev/ttyUSB0]

Necesita:  py -m pip install pyserial matplotlib          (Linux: sudo apt install python3-matplotlib python3-serial python3-tk)
Teclas:  ESPACIO = parar   ·   m = máximo (100 %)   ·   r = reiniciar observadores
"""
import argparse
import collections
import csv
import math
import os
import sys
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider

CAMPOS = ["t_ms", "pwm", "V", "w_med", "w_L", "i_L", "w_Kpri", "i_Kpri", "w_K", "i_K",
          "P11", "P22", "P11_pri", "P22_pri", "K1", "K2"]
VENTANA_S = 10.0
N_MAX = 3000


# ------------------------------------------------------------------ conexión con el ESP32
class Serie:
    def __init__(self, port, baud):
        import serial
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, baud, 0.05
        self.s.dtr = False                  # en las placas ESP32, DTR/RTS van a BOOT/EN: así no se queda en reset
        self.s.rts = False
        self.s.open()
        self.lock = threading.Lock()

    def escribir(self, txt):
        with self.lock:
            self.s.write(txt.encode())

    def leer_linea(self):
        return self.s.readline().decode(errors="ignore").strip()

    def cerrar(self):
        self.s.close()


class Demo:
    """Simula el firmware: motor con el modelo del laboratorio, encoder cuantizado y los mismos observadores."""
    A = ((-0.00147851, -0.00308602), (0.50976057, 0.91345392))
    B = (0.01976948, 0.07782986)
    L = (0.24358162, 0.41197541)
    RPM = math.pi / 30

    def __init__(self):
        self.cola = collections.deque()
        self.pwm_pedido = 0.0
        self.reset = False
        threading.Thread(target=self._correr, daemon=True).start()

    def escribir(self, txt):
        for l in txt.splitlines():
            if l.startswith("p "):
                self.pwm_pedido = max(-100.0, min(100.0, float(l[2:])))
            elif l.startswith("r"):
                self.reset = True

    def leer_linea(self):
        while not self.cola:
            time.sleep(0.005)
        return self.cola.popleft()

    def cerrar(self):
        pass

    def _obs_reset(self):
        self.xl = [0.0, 0.0]; self.yp = 0.0; self.xp = [0.0, 0.0]; self.xk = [0.0, 0.0]; self.K = [0.0, 0.0]
        self.Pk = [[0.01, 0.0], [0.0, self.RPM ** 2]]; self.Pm = [r[:] for r in self.Pk]; self.primero = True

    def _obs(self, u, y):
        A, B, L = self.A, self.B, self.L
        if self.primero:
            self.yp = y; self.xp = self.xk[:]; self.primero = False; return
        x0, x1 = self.xl; inn = self.yp - x1
        self.xl = [A[0][0] * x0 + A[0][1] * x1 + B[0] * u + L[0] * inn, A[1][0] * x0 + A[1][1] * x1 + B[1] * u + L[1] * inn]
        self.yp = y
        x0, x1 = self.xk
        self.xp = [A[0][0] * x0 + A[0][1] * x1 + B[0] * u, A[1][0] * x0 + A[1][1] * x1 + B[1] * u]
        P = self.Pk
        M = [[A[i][0] * P[0][j] + A[i][1] * P[1][j] for j in range(2)] for i in range(2)]
        Pm = [[M[i][0] * A[j][0] + M[i][1] * A[j][1] for j in range(2)] for i in range(2)]
        Pm[0][0] += 0.010 ** 2; Pm[1][1] += (0.30 * self.RPM) ** 2
        S = Pm[1][1] + (0.81 * self.RPM) ** 2
        self.K = [Pm[0][1] / S, Pm[1][1] / S]
        e = y - self.xp[1]
        self.xk = [self.xp[0] + self.K[0] * e, self.xp[1] + self.K[1] * e]
        self.Pk = [[Pm[0][0] - self.K[0] * Pm[1][0], Pm[0][1] - self.K[0] * Pm[1][1]],
                   [Pm[0][1] - self.K[0] * Pm[1][1], Pm[1][1] - self.K[1] * Pm[1][1]]]
        self.Pm = Pm

    def _correr(self):
        ra, la, kv, b, j = 50.0, 0.060, 0.167092, 2.79197e-3, 3.63232e-4
        i = w = th = 0.0; duty = 0.0; u_prev = 0.0; c_prev = 0; t = 0.0; rc = 2 * math.pi / 6424
        self._obs_reset()
        self.cola.append("#LAB4: modo DEMO (motor simulado)")
        t_real = time.monotonic()
        while True:
            if self.reset:
                self._obs_reset(); self.reset = False
            obj = self.pwm_pedido / 100.0
            dsum = 0.0
            for _ in range(10):                                   # 10 ms con rampa de 3/1023 por ms
                duty += max(-3 / 1023, min(3 / 1023, obj - duty)); dsum += duty
                v = 12 * duty; a = abs(v) - 0.8; V = 0.0 if a <= 0 else math.copysign(a, v)
                for _ in range(10):
                    i += (V - ra * i - kv * w) / la * 1e-4
                    w += (kv * i - b * w - 0.0015 * math.tanh(w / 0.2)) / j * 1e-4
                    th += w * 1e-4
            c = math.floor(th / rc); y = (c - c_prev) * rc / 0.01; c_prev = c
            dm = dsum / 10; v = 12 * dm; a = abs(v) - 0.8; u = 0.0 if a <= 0 else math.copysign(a, v)
            self._obs(u_prev, y); u_prev = u
            R = self.RPM
            self.cola.append("D,%d,%.1f,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.2f,%.1f,%.4g,%.4g,%.4g,%.4g,%.4g,%.4f" % (
                t * 1000, duty * 100, u, y / R, self.xl[1] / R, self.xl[0] * 1000, self.xp[1] / R, self.xp[0] * 1000,
                self.xk[1] / R, self.xk[0] * 1000, self.Pk[0][0] * 1e6, self.Pk[1][1] / R ** 2, self.Pm[0][0] * 1e6,
                self.Pm[1][1] / R ** 2, self.K[0] * 1000 * R, self.K[1]))
            t += 0.01
            t_real += 0.01
            time.sleep(max(0.0, t_real - time.monotonic()))


def buscar_puerto():
    """Puerto del ESP32 en Windows (COMx) o Linux (/dev/ttyUSBx): el primero con un chip USB-serie conocido."""
    from serial.tools import list_ports
    puertos = list(list_ports.comports())
    claves = ("CP210", "CH340", "CH910", "USB-SERIAL", "USB SERIAL", "UART", "SILICON LABS", "WCH", "FTDI", "ESP32")
    for p in puertos:
        texto = " ".join(str(x) for x in (p.description, p.manufacturer, p.hwid)).upper()
        if any(k in texto for k in claves):
            return p.device
    usb = [p.device for p in puertos if "USB" in str(p.hwid).upper()]
    return usb[0] if usb else None


# ------------------------------------------------------------------ programa
def main():
    ap = argparse.ArgumentParser(description="Lab 4: observadores del motor 1 (ESP32)")
    ap.add_argument("--port", help="puerto serie: COM5 en Windows, /dev/ttyUSB0 en Linux (por defecto lo busca solo)")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--demo", action="store_true", help="sin robot: motor simulado")
    a = ap.parse_args()

    if a.demo:
        con = Demo()
    else:
        port = a.port or buscar_puerto()
        if not port:
            print("No encontré el ESP32. Mira su puerto en el Administrador de dispositivos (Puertos COM y LPT)\n"
                  "y ejecuta:  py lab4_panel.py --port COM5   (o usa --demo)")
            return 1
        try:
            con = Serie(port, a.baud)
        except Exception as ex:                                     # noqa: BLE001
            print("No pude abrir %s: %s\n(¿Está abierto el monitor de PlatformIO u otro programa? Ciérralo.)" % (port, ex))
            return 1
        print("Conectado a", port)

    datos = collections.deque(maxlen=N_MAX)
    estado = {"pwm": 0.0, "msg": "Esperando datos del ESP32...", "csv": None, "csvw": None, "pausa": False, "t_ult": 0.0}

    def lector():
        while True:
            try:
                l = con.leer_linea()
            except Exception:                                      # noqa: BLE001
                estado["msg"] = "Se perdió la conexión con el ESP32"
                return
            if not l:
                continue
            if l.startswith("D,"):
                p = l.split(",")
                if len(p) != len(CAMPOS) + 1:
                    continue
                try:
                    v = [float(x) for x in p[1:]]
                except ValueError:
                    continue
                datos.append(v)
                estado["t_ult"] = time.monotonic()
                if estado["csvw"]:
                    estado["csvw"].writerow(v)
            elif l.startswith("#"):
                estado["msg"] = l
                print(l)

    def emisor():                                                   # el ESP32 para el motor si no recibe nada en 1,5 s
        while True:
            con.escribir("p %.1f\n" % estado["pwm"])
            time.sleep(0.2)

    threading.Thread(target=lector, daemon=True).start()
    threading.Thread(target=emisor, daemon=True).start()

    # ---------------- figura
    plt.rcParams.update({"font.size": 8.5, "axes.grid": True, "grid.alpha": 0.3})
    fig = plt.figure("Lab 4 · Observadores del motor 1" + ("  (DEMO)" if a.demo else ""), figsize=(14, 8.6))
    gs = fig.add_gridspec(3, 2, left=0.06, right=0.985, top=0.95, bottom=0.25, hspace=0.45, wspace=0.16)
    ax_w = fig.add_subplot(gs[0, 0]); ax_i = fig.add_subplot(gs[0, 1], sharex=ax_w)
    ax_pw = fig.add_subplot(gs[1, 0], sharex=ax_w); ax_pi = fig.add_subplot(gs[1, 1], sharex=ax_w)
    ax_k = fig.add_subplot(gs[2, 0], sharex=ax_w); ax_u = fig.add_subplot(gs[2, 1], sharex=ax_w)

    def curva(ax, nombre, color, lw=1.4, ls="-", marker=None):
        return ax.plot([], [], color=color, lw=lw, ls=ls, marker=marker, ms=2, label=nombre)[0]

    lin = {
        "w_med": curva(ax_w, "medida (encoder)", "#9aa3ad", 0.9, marker="."),
        "w_L": curva(ax_w, "Luenberger", "#2e9e5b"),
        "w_Kpri": curva(ax_w, "Kalman a priori", "#e8893a", 1.2, "--"),
        "w_K": curva(ax_w, "Kalman a posteriori", "#1f77b4", 1.8),
        "i_L": curva(ax_i, "Luenberger", "#2e9e5b", 1.0),
        "i_Kpri": curva(ax_i, "Kalman a priori", "#e8893a", 1.2, "--"),
        "i_K": curva(ax_i, "Kalman a posteriori", "#1f77b4", 1.8),
        "P22_pri": curva(ax_pw, "P a priori (P⁻₂₂)", "#e8893a", 1.2, "--"),
        "P22": curva(ax_pw, "P a posteriori (P₂₂)", "#6b4fc8", 1.8),
        "P11_pri": curva(ax_pi, "P a priori (P⁻₁₁)", "#e8893a", 1.2, "--"),
        "P11": curva(ax_pi, "P a posteriori (P₁₁)", "#6b4fc8", 1.8),
        "K2": curva(ax_k, "K₂ (velocidad)", "#c08a00", 1.8),
        "pwm": curva(ax_u, "PWM [%]", "#c0504d", 1.6),
        "V": curva(ax_u, "voltaje del modelo [V]", "#1f4e79", 1.2),
    }
    ax_k2 = ax_k.twinx()
    lin["K1"] = ax_k2.plot([], [], color="#7a7a7a", lw=1.2, ls=":", label="K₁ (corriente) [mA/rpm]")[0]
    for ax, tit, yl in ((ax_w, "Velocidad del motor 1", "rpm"), (ax_i, "Corriente estimada (no se mide)", "mA"),
                        (ax_pw, "Covarianza P de la velocidad", "rpm²"), (ax_pi, "Covarianza P de la corriente", "mA²"),
                        (ax_k, "Ganancia de Kalman", "K₂"), (ax_u, "Entrada", "% · V")):
        ax.set_title(tit, fontsize=10, fontweight="bold")
        ax.set_ylabel(yl)
        ax.legend(loc="upper left", fontsize=7.5)
    for ax in (ax_pw, ax_pi):
        ax.set_ylim(0.1, 1e4)
        ax.set_yscale("log")                                        # P0 es ~100 veces el valor estable
    ax_k2.set_ylabel("K₁ [mA/rpm]", fontsize=8)
    ax_k2.legend(loc="upper right", fontsize=7.5)
    for ax in (ax_k, ax_u):
        ax.set_xlabel("tiempo [s]")
    txt_estado = fig.text(0.06, 0.16, "", fontsize=9.5, family="monospace")
    txt_msg = fig.text(0.06, 0.13, "", fontsize=9, color="#1f4e79")

    # ---------------- controles
    ax_sl = fig.add_axes([0.10, 0.07, 0.42, 0.03])
    sl = Slider(ax_sl, "PWM [%]", -100, 100, valinit=0, valstep=1, color="#c0504d", valfmt="%+d %%")

    def poner(p):
        estado["pwm"] = float(p)
        if sl.val != p:
            sl.set_val(p)
        con.escribir("p %.1f\n" % estado["pwm"])

    sl.on_changed(lambda v: poner(v))
    botones = []

    def boton(x, ancho, txt, fn, color="#e9eef5"):
        b = Button(fig.add_axes([x, 0.02, ancho, 0.04]), txt, color=color, hovercolor="#cfdbea")
        b.on_clicked(lambda _e: fn())
        botones.append(b)
        return b

    boton(0.10, 0.10, "PARAR (0 %)", lambda: poner(0), "#f6c9c7")
    boton(0.21, 0.10, "MÁXIMO (100 %)", lambda: poner(100), "#cfe8d6")
    boton(0.32, 0.09, "50 %", lambda: poner(50))
    boton(0.42, 0.10, "−100 % (atrás)", lambda: poner(-100))
    boton(0.56, 0.13, "Reiniciar observadores", lambda: con.escribir("r\n"))
    b_csv = boton(0.70, 0.10, "Grabar CSV", lambda: grabar())
    b_pau = boton(0.81, 0.09, "Pausar", lambda: pausar())

    def grabar():
        if estado["csv"]:
            estado["csv"].close()
            estado["msg"] = "CSV guardado: " + estado["csv"].name
            estado["csv"] = estado["csvw"] = None
            b_csv.label.set_text("Grabar CSV")
        else:
            nombre = time.strftime("lab4_motor1_%Y%m%d_%H%M%S.csv")
            estado["csv"] = open(os.path.join(os.getcwd(), nombre), "w", newline="")
            estado["csvw"] = csv.writer(estado["csv"])
            estado["csvw"].writerow(CAMPOS)
            estado["msg"] = "Grabando en " + nombre
            b_csv.label.set_text("■ Detener CSV")

    def pausar():
        estado["pausa"] = not estado["pausa"]
        b_pau.label.set_text("Seguir" if estado["pausa"] else "Pausar")

    def tecla(ev):
        if ev.key == " ":
            poner(0)
        elif ev.key == "m":
            poner(100)
        elif ev.key == "r":
            con.escribir("r\n")

    fig.canvas.mpl_connect("key_press_event", tecla)

    # ---------------- refresco
    def refrescar(_f):
        if not datos:
            txt_msg.set_text(estado["msg"])
            return []
        d = list(datos)
        t_fin = d[-1][0] / 1000.0
        d = [r for r in d if r[0] / 1000.0 >= t_fin - VENTANA_S]
        if not estado["pausa"]:
            t = [r[0] / 1000.0 for r in d]
            for k, l in lin.items():
                idx = CAMPOS.index(k)
                l.set_data(t, [r[idx] for r in d])
            for ax in (ax_w, ax_i, ax_pw, ax_pi, ax_k, ax_k2, ax_u):
                ax.relim()
                ax.autoscale_view(scalex=False)
            ax_w.set_xlim(max(0.0, t_fin - VENTANA_S), max(VENTANA_S, t_fin))
        r = d[-1]
        g = dict(zip(CAMPOS, r))
        txt_estado.set_text(
            "PWM %+6.1f %%   V %+6.2f V   |  velocidad: medida %+7.2f  Luenberger %+7.2f  Kalman %+7.2f rpm   |  "
            "corriente: Luenberger %+7.1f  Kalman %+7.1f mA   |  K₂ %.3f" % (
                g["pwm"], g["V"], g["w_med"], g["w_L"], g["w_K"], g["i_L"], g["i_K"], g["K2"]))
        sin_datos = time.monotonic() - estado["t_ult"] > 1.0
        txt_msg.set_text(("SIN DATOS DEL ESP32 (¿se reinició o se desconectó?)   " if sin_datos else "") + estado["msg"])
        return []

    anim = FuncAnimation(fig, refrescar, interval=100, cache_frame_data=False)   # noqa: F841

    def al_cerrar(_e):
        for _ in range(3):
            try:
                con.escribir("p 0\n")
            except Exception:                                      # noqa: BLE001
                pass
            time.sleep(0.05)
        if estado["csv"]:
            estado["csv"].close()
        con.cerrar()

    fig.canvas.mpl_connect("close_event", al_cerrar)
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
