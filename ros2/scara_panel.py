#!/usr/bin/env python3
"""
PANEL DE CONTROL DEL SCARA (ROS 2 Jazzy + micro-ROS) — PyQt5 + pyqtgraph.

    python3 ros2/scara_panel.py            con el robot (agente micro-ROS corriendo)
    python3 ros2/scara_panel.py --demo     sin robot: simula el ESP32 con el modelo identificado (para practicar)

Qué tiene:
  * Encabezado: conexión, modo, falla, Home, finales de carrera y los botones HOME / DETENER / APAGAR / BORRAR FALLA.
  * Una tarjeta por motor (M1, M2, M3): habilitar/deshabilitar, velocidad y corriente estimadas, PWM y un
    deslizador de POTENCIA manual (lazo abierto, % de PWM, tope del firmware ~63 %).
  * Control manual con DEAD-MAN (botón "MANTENER" o la barra ESPACIO):
        Posición  -> flechas: J1 (← →), J2 (↓ ↑)            (publica /scara/jog)
        Potencia  -> aplica los deslizadores de los motores   (publica /scara/power)
    Al soltar, el robot frena y se queda quieto. Sin latidos durante 300 ms el ESP32 se detiene solo.
  * Gráficas de los OBSERVADORES por motor: velocidad medida, Luenberger, Kalman a priori y a posteriori;
    corriente (Luenberger, Kalman a priori y a posteriori); incertidumbre sqrt(P) y ganancia de Kalman.
  * Comparación de los 3 motores, vista XY del brazo (clic = consigna), sintonía de PID y observadores,
    registro de eventos, línea de comandos y grabación a CSV.

Requisitos (Ubuntu 24.04):  sudo apt install python3-pyqt5 python3-pyqtgraph
Atajos (con el foco fuera de los cuadros de texto): ESPACIO dead-man · flechas · h Home · s Detener ·
o Apagar · c Borrar falla · r Reiniciar observadores · Esc Detener.
"""
import argparse
import collections
import csv
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scara_common as sc  # noqa: E402

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg
except ImportError as e:  # pragma: no cover
    print("Falta PyQt5 o pyqtgraph (%s).\n  sudo apt install python3-pyqt5 python3-pyqtgraph" % e)
    sys.exit(1)

HB_HZ = 25                      # latidos de dead-man (igual que el teleop anterior)
STALE_S = 0.6                   # sin /scara/state en este tiempo -> "sin conexión"
HIST_S = 60.0                   # historia guardada para las gráficas

# ---------------------------------------------------------------- colores
C = dict(bg="#0b1220", card="#111a2e", card2="#16213a", line="#1f2b45", text="#e2e8f0", muted="#8b9ab4",
         cyan="#22d3ee", green="#34d399", orange="#fb923c", red="#f87171", violet="#a78bfa", amber="#fbbf24",
         blue="#60a5fa", gray="#cbd5e1")
MOTOR_COLORS = (C["cyan"], C["violet"], C["amber"])
MODE_COLORS = {"OFF": C["muted"], "HOMING": C["amber"], "HOLD": C["green"], "MOVE": C["blue"], "JOG": C["cyan"],
               "PULSE": C["violet"], "FAULT": C["red"], "POWER": C["orange"]}

QSS = """
* { font-family: "Inter", "Ubuntu", "Segoe UI", "DejaVu Sans", sans-serif; }
QWidget { background: %(bg)s; color: %(text)s; font-size: 10pt; }
QFrame#card { background: %(card)s; border: 1px solid %(line)s; border-radius: 12px; }
QFrame#header { background: %(card)s; border: 1px solid %(line)s; border-radius: 14px; }
QLabel { background: transparent; }
QLabel#title { font-size: 17pt; font-weight: 800; letter-spacing: 0.5px; }
QLabel#subtitle { color: %(muted)s; font-size: 9pt; }
QLabel#section { font-size: 11pt; font-weight: 700; }
QLabel#muted { color: %(muted)s; font-size: 9pt; }
QLabel#big { font-size: 15pt; font-weight: 700; }
QLabel#unit { color: %(muted)s; font-size: 8pt; }
QPushButton { background: %(card2)s; border: 1px solid #2a3a5c; border-radius: 9px; padding: 7px 12px; font-weight: 600; }
QPushButton:hover { background: #1d2b4a; border-color: #3b5180; }
QPushButton:pressed { background: #0e7490; }
QPushButton:checked { background: #0e4f63; border: 1px solid %(cyan)s; color: white; }
QPushButton:disabled { color: #56627a; border-color: %(line)s; }
QPushButton#primary { background: #0891b2; border-color: #22d3ee; color: white; }
QPushButton#primary:hover { background: #0aa5c9; }
QPushButton#danger { background: #b91c1c; border-color: #f87171; color: white; }
QPushButton#danger:hover { background: #d32222; }
QPushButton#warn { background: #92400e; border-color: #fbbf24; color: white; }
QPushButton#deadman { min-height: 52px; font-size: 12pt; font-weight: 800; background: #14361f; border: 2px solid #22c55e; color: #bbf7d0; }
QPushButton#deadman:pressed { background: #16a34a; color: white; }
QPushButton#jog { min-width: 46px; min-height: 38px; font-size: 13pt; }
QPushButton#seg { border-radius: 8px; padding: 6px 10px; }
QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox { background: #0d1628; border: 1px solid #2a3a5c; border-radius: 7px; padding: 4px 6px; selection-background-color: #0e7490; }
QLineEdit:focus, QDoubleSpinBox:focus, QSpinBox:focus { border-color: %(cyan)s; }
QPlainTextEdit { background: #0d1628; border: 1px solid %(line)s; border-radius: 10px; font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace; font-size: 9pt; }
QTabWidget::pane { border: 1px solid %(line)s; border-radius: 12px; background: %(card)s; top: -1px; }
QTabBar::tab { background: transparent; color: %(muted)s; padding: 8px 16px; margin-right: 4px; border-bottom: 2px solid transparent; font-weight: 600; }
QTabBar::tab:selected { color: %(text)s; border-bottom: 2px solid %(cyan)s; }
QTabBar::tab:hover { color: %(text)s; }
QSlider::groove:horizontal { height: 6px; background: #1d2a44; border-radius: 3px; }
QSlider::handle:horizontal { width: 18px; height: 18px; margin: -7px 0; border-radius: 9px; background: %(text)s; border: 2px solid %(cyan)s; }
QSlider::handle:horizontal:disabled { background: #334155; border-color: #334155; }
QScrollArea { border: none; }
QScrollBar:vertical { background: transparent; width: 10px; }
QScrollBar::handle:vertical { background: #2a3a5c; border-radius: 5px; min-height: 30px; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
QSplitter::handle { background: transparent; }
QToolTip { background: %(card2)s; color: %(text)s; border: 1px solid %(line)s; padding: 4px; }
""" % C


# ================================================================ puentes con el robot
class RosBridge:
    """Habla con el ESP32 por ROS 2. El ejecutor de rclpy corre en un hilo; los datos llegan a colas."""

    demo = False

    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Point, Vector3
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float32MultiArray, String

        self._rclpy, self._Point, self._Vector3 = rclpy, Point, Vector3
        self._String, self._F32 = String, Float32MultiArray
        rclpy.init()
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)
        self.node = Node("scara_panel")
        self.p_cmd = self.node.create_publisher(String, "/scara/cmd", rel)
        self.p_goto = self.node.create_publisher(Point, "/scara/goto", rel)
        self.p_jog = self.node.create_publisher(Vector3, "/scara/jog", be)
        self.p_pow = self.node.create_publisher(Float32MultiArray, "/scara/power", be)
        self.states, self.obs, self.events = collections.deque(maxlen=200), collections.deque(maxlen=200), collections.deque(maxlen=500)
        self.node.create_subscription(Float32MultiArray, "/scara/state", lambda m: self.states.append((time.monotonic(), list(m.data))), be)
        self.node.create_subscription(Float32MultiArray, "/scara/obs", lambda m: self.obs.append((time.monotonic(), list(m.data))), be)
        self.node.create_subscription(String, "/scara/event", lambda m: self.events.append(m.data), rel)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.running = True
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()

    def _spin(self):
        while self.running and self._rclpy.ok():
            try:
                self.ex.spin_once(timeout_sec=0.05)
            except Exception:  # pragma: no cover - al cerrar
                break

    def cmd(self, text):
        self.p_cmd.publish(self._String(data=text))

    def goto(self, x, y, z):
        self.p_goto.publish(self._Point(x=float(x), y=float(y), z=float(z)))

    def jog(self, dm, d1, d2):
        self.p_jog.publish(self._Vector3(x=1.0 if dm else 0.0, y=float(d1), z=float(d2)))

    def power(self, dm, pct):
        self.p_pow.publish(self._F32(data=[1.0 if dm else 0.0] + [float(p) for p in pct]))

    def close(self):
        try:
            for _ in range(3):
                self.jog(False, 0, 0)
                self.power(False, (0, 0, 0))
            self.cmd("stop")
            time.sleep(0.05)
        except Exception:
            pass
        self.running = False
        try:
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass


class DemoBridge:
    """Simula el ESP32 sin hardware: motores con el modelo identificado, PID, potencia manual, observadores
    (copia de estimator.c) y los mismos mensajes de /scara/state, /scara/obs y /scara/event."""

    demo = True
    DT = 0.001

    def __init__(self):
        self.states, self.obs, self.events = collections.deque(maxlen=200), collections.deque(maxlen=200), collections.deque(maxlen=500)
        self.A, self.B = sc.discretize(dict(sc.MOTOR, j=sc.MOTOR["j"] * 2.5), self.DT)   # con algo de eslabón
        self.x = [np.zeros(2) for _ in range(sc.NM)]
        self.th = [0.4, 0.9, 0.0]                       # posición de salida [rad] (física)
        self.off = [0.0, 0.0, 0.0]
        self.en = [True, True, False]
        self.homed = [False, False]
        self.mode = "OFF"
        self.fault = 0
        self.qref = [0.0, 0.0]
        self.qdref = [0.0, 0.0]
        self.integ = [0.0, 0.0]
        self.duty = [0.0, 0.0, 0.0]
        self.gains = [[5350, 18500, 295, 85, 70], [5000, 3000, 180, 80, 70]]
        self.vgain, self.v0 = 1.0, 0.8
        self.est = [sc.Estimator() for _ in range(sc.NM)]
        self.cnt_prev = [math.floor(t / (2 * math.pi) * 6424) for t in self.th]
        self.duty_acc = [0.0] * sc.NM
        self.n = 0
        self.seq = 0
        self.jog_hb = (False, 0, 0, -1.0)
        self.pow_hb = (False, (0, 0, 0), -1.0)
        self.pw_latch = False
        self.pstop = [False, False]
        self.traj = None
        self.await_reach = None
        self.elbow = 1
        self.t = 0.0
        self.last_obs = [None] * sc.NM
        self.events.append("#INFO: MODO DEMO: robot simulado con el modelo identificado (no hay ESP32).")
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._tick20)
        self.timer.start(20)

    # ---- interfaz igual a RosBridge
    def cmd(self, text):
        self._exec(text.strip().lower())

    def goto(self, x, y, z):
        self._exec("goto %g %g %g" % (x, y, z))

    def jog(self, dm, d1, d2):
        self.jog_hb = (bool(dm), int(d1), int(d2), self.t)

    def power(self, dm, pct):
        self.pow_hb = (bool(dm), tuple(pct), self.t)

    def close(self):
        self.timer.stop()

    # ---- comandos
    def _ev(self, s):
        self.events.append(s)

    def _q(self, j):
        return self.th[j] - self.off[j]

    def _exec(self, l):
        p = l.split()
        if not p:
            return
        try:
            if l == "home":
                if self.mode not in ("OFF", "HOLD"):
                    return self._ev("#ERROR: no se puede iniciar Home, el robot esta en modo %s." % self.mode)
                for j in range(2):
                    if self.en[j]:
                        self.homed[j] = True
                        self.off[j] = 0.0
                self._hold_here()
                self.mode = "HOLD"
                self._ev("#HOME: (demo) ejes habilitados referenciados al instante.")
                return self._ev("#LLEGO: q1=%.3f q2=%.3f deg" % (math.degrees(self._q(0)), math.degrees(self._q(1))))
            if l == "stop":
                if self.mode in ("MOVE", "JOG", "POWER"):
                    self._hold_here()
                    self.mode = "HOLD"
                self.pw_latch = True
                return self._ev("#OK: detenido (mantiene posicion)")
            if l == "off":
                self.mode = "OFF" if self.mode != "FAULT" else "FAULT"
                self.pw_latch = True
                return self._ev("#OK: motores apagados")
            if l == "clear":
                if self.mode == "FAULT":
                    self.mode, self.fault = "OFF", 0
                return self._ev("#OK: falla borrada")
            if p[0] == "motor" and len(p) == 3:
                m = int(p[1]) - 1
                on = p[2] in ("on", "1")
                self.en[m] = on
                if m < 2:
                    self.qref[m] = self._q(m)
                return self._ev("#MOTOR: M%d %s." % (m + 1, "HABILITADO" if on else "deshabilitado (libre)"))
            if p[0] == "motors" and len(p) == 4:
                for m in range(3):
                    self.en[m] = p[m + 1] != "0"
                return self._ev("#OK: motores " + " ".join("M%d %s" % (m + 1, "ON" if self.en[m] else "off") for m in range(3)))
            if l in ("motors", "motor"):
                return self._ev("#INFO: motores: " + ", ".join("M%d %s" % (m + 1, "ON" if self.en[m] else "off") for m in range(3)))
            if l == "est reset":
                for e in self.est:
                    e.reset()
                return self._ev("#INFO: observadores reiniciados (x = 0, P = P0).")
            if p[0] == "est":
                m = int(p[1]) - 1 if len(p) > 1 else 0
                e = self.est[m]
                return self._ev("#INFO: M%d polos %.3f+-%.3fj L=[%.4g, %.4g] | Kalman sig_i=%.1f mA sig_w=%.2f rpm sig_y=%.2f rpm | V=%.3f*12*duty-%.2f"
                                % (m + 1, e.p["polo_re"], e.p["polo_im"], e.L[0], e.L[1], e.p["sig_i"] * 1000,
                                   e.p["sig_w"] / sc.RPM, e.p["sig_y"] / sc.RPM, self.vgain, self.v0))
            if p[0] == "luen" and len(p) == 4:
                m, re, im = int(p[1]) - 1, float(p[2]), abs(float(p[3]))
                if math.hypot(re, im) >= 0.999:
                    return self._ev("#ERROR: luen j re im con |polo| < 1")
                e = self.est[m]
                e.design(dict(e.p, polo_re=re, polo_im=im))
                return self._ev("#EST: M%d Luenberger polos %.3f +/- %.3fj -> L = [%.5g, %.5g]" % (m + 1, re, im, e.L[0], e.L[1]))
            if p[0] == "kal" and len(p) == 5:
                m = int(p[1]) - 1
                e = self.est[m]
                e.design(dict(e.p, sig_i=float(p[2]) / 1000, sig_w=float(p[3]) * sc.RPM, sig_y=float(p[4]) * sc.RPM))
                return self._ev("#EST: M%d Kalman sig_i=%s mA, sig_w=%s rpm, sig_y=%s rpm" % (m + 1, p[2], p[3], p[4]))
            if p[0] == "vmap" and len(p) == 3:
                self.vgain, self.v0 = float(p[1]), float(p[2])
                return self._ev("#EST: voltaje del modelo V = %.4f * (12 V * duty) - %.3f V" % (self.vgain, self.v0))
            if p[0] == "gain":
                if len(p) == 7:
                    j = int(p[1]) - 1
                    self.gains[j] = [float(v) for v in p[2:]]
                    return self._ev("#OK: ganancias del eje %d actualizadas" % (j + 1))
                g = self.gains
                return self._ev("#INFO: J1 kp=%.0f ki=%.0f kd=%.0f kff=%.0f fric=%.0f | J2 kp=%.0f ki=%.0f kd=%.0f kff=%.0f fric=%.0f"
                                % tuple(g[0] + g[1]))
            if p[0] == "elbow":
                self.elbow = 1 if float(p[1]) >= 0 else -1
                return self._ev("#OK: codo preferido = %+d" % self.elbow)
            if p[0] in ("goto", "ang"):
                if not all(self.homed[j] for j in range(2) if self.en[j]):
                    return self._ev("#ERROR: primero ejecuta HOME (opcion 5).")
                if p[0] == "goto":
                    if not (self.en[0] and self.en[1]):
                        return self._ev("#ERROR: goto necesita J1 y J2 habilitados.")
                    x, y = float(p[1]), float(p[2])
                    q = sc.ik(x, y, self.elbow)
                    if q is None:
                        return self._ev("#ERROR: no se puede ir a (%.1f, %.1f): inalcanzable. Alcance: 50.0 a 190.0 mm." % (x, y))
                    self._ev("#INFO: cinematica inversa (%.1f, %.1f, 0.0) -> q1 = %.2f deg, q2 = %.2f deg (codo %+d). Moviendo..."
                             % (x, y, q[0], q[1], self.elbow))
                    tgt = (x, y)
                else:
                    q = (float(p[1]), float(p[2]))
                    tgt = None
                    self._ev("#INFO: moviendo a q1 = %.2f deg, q2 = %.2f deg..." % q)
                q0 = list(self.qref)
                q1 = [math.radians(q[j]) if self.en[j] else q0[j] for j in range(2)]
                dmax = max(abs(q1[j] - q0[j]) for j in range(2))
                T = max(dmax / math.radians(50), 0.05)
                self.traj = (self.t, T, q0, q1)
                self.await_reach = tgt if tgt else "ang"
                self.mode = "MOVE"
                return
            if l == "status":
                return self._ev("#INFO: [%s] q1=%.2f q2=%.2f deg home J1:%d J2:%d (demo)"
                                % (self.mode, math.degrees(self._q(0)), math.degrees(self._q(1)), self.homed[0], self.homed[1]))
            if l == "help":
                return self._ev("#INFO: home stop off clear status | goto x y [z] | ang q1 q2 | elbow +-1 | gain [j ...] | "
                                "motor j on/off | motors | est [j|reset] | luen j re im | kal j mA rpm rpm | vmap g v0")
            if p[0] == "pulse":
                return self._ev("#INFO: (demo) el pulso de prueba solo tiene sentido con el robot real")
        except (ValueError, IndexError):
            pass
        self._ev("#ERROR: comando desconocido: %s  (usa 'help')" % l)

    def _hold_here(self):
        for j in range(2):
            self.qref[j] = self._q(j)
            self.qdref[j] = 0.0
            self.integ[j] = 0.0

    # ---- simulación
    def _pid(self, j, e, qd_ref, qd):
        kp, ki, kd, kff, fric = self.gains[j]
        u = kff * qd_ref + kp * e + kd * (qd_ref - qd) + self.integ[j]
        if not (abs(u) >= 650 and (u > 0) == (e > 0)):
            self.integ[j] = max(-250, min(250, self.integ[j] + ki * e * self.DT))
        if abs(e) > 0.0026 and u != 0:
            u += math.copysign(fric, u)
        return max(-650, min(650, u))

    def _tick20(self):
        for _ in range(20):
            self._step()
        self._publish()

    def _step(self):
        self.t += self.DT
        jdm, d1, d2, tj = self.jog_hb
        pdm, pct, tp = self.pow_hb
        jfresh = jdm and self.t - tj < 0.3
        pfresh = pdm and self.t - tp < 0.3
        if not pfresh:
            self.pw_latch = False
        pw = [(max(-100, min(100, pct[m])) * 10.23 if (pfresh and not self.pw_latch and self.en[m]) else 0.0) for m in range(3)]
        pw = [max(-650, min(650, v)) for v in pw]
        u = [0.0, 0.0, 0.0]
        qd = [self.x[m][1] for m in range(3)]
        if self.mode in ("OFF", "HOLD", "JOG") and any(abs(pw[j]) > 0.5 for j in range(2)):
            if self.mode != "HOLD":
                self._hold_here()
            self.pstop = [False, False]
            self.mode = "POWER"
        if self.mode in ("OFF", "HOLD") and jfresh and (d1 or d2):
            if self.mode == "OFF":
                self._hold_here()
            self.mode = "JOG"
        if self.mode == "MOVE" and self.traj:
            t0, T, q0, q1 = self.traj
            s = min(1.0, (self.t - t0) / T)
            ss = s * s * (3 - 2 * s)
            for j in range(2):
                self.qref[j] = q0[j] + ss * (q1[j] - q0[j])
                self.qdref[j] = (q1[j] - q0[j]) * 6 * s * (1 - s) / T if s < 1 else 0.0
            if s >= 1:
                self.mode = "HOLD"
        if self.mode == "JOG":
            for j, d in enumerate((d1, d2)):
                vt = (d * math.radians(30)) if (jfresh and self.en[j]) else 0.0
                dv = (15 if abs(vt) < abs(self.qdref[j]) else 5) * self.DT
                self.qdref[j] = vt if abs(self.qdref[j] - vt) <= dv else self.qdref[j] + math.copysign(dv, vt - self.qdref[j])
                self.qref[j] += self.qdref[j] * self.DT
                self.qref[j] = max(self._q(j) - 0.12, min(self._q(j) + 0.12, self.qref[j]))
            if not jfresh and all(abs(v) < 1e-4 for v in self.qdref):
                self.mode = "HOLD"
        if self.mode in ("HOLD", "MOVE", "JOG"):
            for j in range(2):
                u[j] = self._pid(j, self.qref[j] - self._q(j), self.qdref[j], qd[j]) if self.en[j] else 0.0
        if self.mode == "POWER":
            alguno = False
            for j in range(2):
                if not self.en[j]:
                    self.pstop[j] = False
                    continue
                if abs(pw[j]) > 0.5:                      # lazo abierto (con Home: respeta los límites)
                    u[j] = pw[j]
                    lim = math.radians(120 if j == 0 else 140)
                    if self.homed[j] and ((self._q(j) >= lim and u[j] > 0) or (self._q(j) <= -lim and u[j] < 0)):
                        u[j] = 0.0
                    self.qref[j], self.qdref[j], self.integ[j] = self._q(j), 0.0, 0.0
                    self.pstop[j] = True
                    alguno = True
                elif self.pstop[j]:                       # frena con amortiguamiento y se queda donde para
                    self.qref[j], self.integ[j] = self._q(j), 0.0
                    u[j] = max(-650, min(650, -self.gains[j][2] * qd[j]))
                    if abs(qd[j]) < 0.03:
                        self.pstop[j] = False
                    else:
                        alguno = True
                else:
                    u[j] = self._pid(j, self.qref[j] - self._q(j), 0.0, qd[j])
            if not alguno:
                self.mode = "HOLD"
        if self.mode not in ("FAULT",):
            u[2] = pw[2]
        for m in range(3):
            if not self.en[m] or (self.mode == "OFF" and m < 2):
                u[m] = 0.0
            d = max(-3.0, min(3.0, u[m] - self.duty[m]))
            self.duty[m] += d
            if not self.en[m] or (self.mode == "OFF" and m < 2):
                self.duty[m] = 0.0
            v = sc.voltaje(self.duty[m] / 1023.0, 12.0, 1.0, 0.8)
            if self.duty[m] == 0.0 and abs(self.x[m][1]) > 0:       # motor libre: corriente 0, solo fricción viscosa
                self.x[m] = np.array([0.0, self.x[m][1] * (1 - 7.7 * self.DT)])
            else:
                self.x[m] = self.A @ self.x[m] + self.B * v
            self.th[m] += self.x[m][1] * self.DT
            self.duty_acc[m] += self.duty[m]
        if self.await_reach and self.mode == "HOLD":
            if all(abs(self.qref[j] - self._q(j)) < 0.007 for j in range(2) if self.en[j]):
                q1d, q2d = math.degrees(self._q(0)), math.degrees(self._q(1))
                x, y = sc.fk(q1d, q2d)
                if self.await_reach == "ang":
                    self._ev("#LLEGO: q1=%.3f q2=%.3f deg | medido x=%.2f y=%.2f mm" % (q1d, q2d, x, y))
                else:
                    tx, ty = self.await_reach
                    self._ev("#LLEGO: objetivo x=%.2f y=%.2f z=0.00 | medido x=%.2f y=%.2f | error ex=%.3f ey=%.3f mm | q1=%.3f q2=%.3f deg"
                             % (tx, ty, x, y, x - tx, y - ty, q1d, q2d))
                self.await_reach = None
        self.n += 1
        if self.n % 10 == 0:                                          # observadores a 100 Hz
            for m in range(3):
                cnt = math.floor(self.th[m] / (2 * math.pi) * 6424)
                y = (cnt - self.cnt_prev[m]) * 2 * math.pi / 6424 / 0.01
                self.cnt_prev[m] = cnt
                uv = sc.voltaje(self.duty_acc[m] / 10 / 1023.0, 12.0, self.vgain, self.v0)
                self.duty_acc[m] = 0.0
                e = self.est[m]
                e.step(uv, y, True)
                self.last_obs[m] = (y, uv)

    def _publish(self):
        q1, q2 = math.degrees(self._q(0)), math.degrees(self._q(1))
        x, y = sc.fk(q1, q2)
        en = sum((1 << m) for m in range(3) if self.en[m])
        st = [q1, q2, x, y, sc.MODES.index(self.mode), self.fault, self.homed[0], self.homed[1], 0,
              math.degrees(self.qref[0]), math.degrees(self.qref[1]), self.duty[0], self.duty[1],
              en, self.duty[2], 1.0 if self.pow_hb[0] and self.t - self.pow_hb[2] < 0.3 else 0.0,
              self.x[0][1] / sc.RPM, self.x[1][1] / sc.RPM, self.x[2][1] / sc.RPM]
        now = time.monotonic()
        self.states.append((now, [float(v) for v in st]))
        self.seq += 1
        ob = [float(self.seq), float(0b111111)]
        for m in range(3):
            e = self.est[m]
            y, uv = self.last_obs[m] or (0.0, 0.0)
            ob += [y / sc.RPM, e.xl[1] / sc.RPM, e.xp[1] / sc.RPM, e.xk[1] / sc.RPM, e.xl[0], e.xp[0], e.xk[0],
                   math.sqrt(max(e.Pk[0, 0], 0)), math.sqrt(max(e.Pk[1, 1], 0)) / sc.RPM, e.K[1], e.K[0], uv]
        self.obs.append((now, [float(v) for v in ob]))


# ================================================================ widgets
class Toggle(QtWidgets.QAbstractButton):
    """Interruptor tipo switch (dibujado)."""

    def __init__(self, color=C["green"], parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setFixedSize(44, 24)
        self.color = QtGui.QColor(color)
        self.setFocusPolicy(QtCore.Qt.NoFocus)

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        on = self.isChecked()
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(self.color if on else QtGui.QColor("#334155"))
        p.drawRoundedRect(0, 2, 44, 20, 10, 10)
        p.setBrush(QtGui.QColor("white"))
        p.drawEllipse(QtCore.QPointF(33 if on else 11, 12), 8, 8)


def card(parent=None):
    f = QtWidgets.QFrame(parent)
    f.setObjectName("card")
    return f


def label(text="", name=None, align=None):
    w = QtWidgets.QLabel(text)
    if name:
        w.setObjectName(name)
    if align is not None:
        w.setAlignment(align)
    return w


def button(text, name=None, tip=None, checkable=False):
    b = QtWidgets.QPushButton(text)
    if name:
        b.setObjectName(name)
    if tip:
        b.setToolTip(tip)
    b.setCheckable(checkable)
    b.setFocusPolicy(QtCore.Qt.NoFocus)      # ESPACIO nunca "pulsa" un botón: es el dead-man
    b.setCursor(QtCore.Qt.PointingHandCursor)
    return b


def rgba(hexcol, a):
    c = QtGui.QColor(hexcol)
    return "rgba(%d, %d, %d, %.2f)" % (c.red(), c.green(), c.blue(), a)


class Pill(QtWidgets.QLabel):
    def set(self, text, color):
        if text == self.text() and getattr(self, "_col", None) == color:
            return
        self._col = color
        self.setText(text)
        self.setStyleSheet("QLabel { color: %s; background: %s; border: 1px solid %s; border-radius: 11px;"
                           " padding: 4px 11px; font-weight: 700; }" % (color, rgba(color, 0.14), rgba(color, 0.55)))


class Led(QtWidgets.QLabel):
    def __init__(self, text):
        super().__init__(text)
        self.set(False)

    def set(self, on):
        col = C["red"] if on else "#334155"
        self.setStyleSheet("QLabel { color: %s; font-weight: 700; font-size: 8pt; padding: 2px 6px; border-radius: 6px;"
                           " background: %s; }" % ("white" if on else C["muted"], col))


class MotorCard(QtWidgets.QFrame):
    def __init__(self, m, panel):
        super().__init__()
        self.setObjectName("card")
        self.m, self.panel = m, panel
        col = MOTOR_COLORS[m]
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(6)
        top = QtWidgets.QHBoxLayout()
        dot = label("●")
        dot.setStyleSheet("color: %s; font-size: 14pt;" % col)
        top.addWidget(dot)
        name = label(sc.MOTOR_NAMES[m], "section")
        top.addWidget(name)
        top.addStretch(1)
        self.state_lbl = label("", "muted")
        top.addWidget(self.state_lbl)
        self.toggle = Toggle(C["green"])
        self.toggle.setToolTip("Habilitar / deshabilitar el motor (deshabilitado = libre, sin PWM)")
        self.toggle.clicked.connect(self._toggled)
        top.addWidget(self.toggle)
        lay.addLayout(top)

        g = QtWidgets.QGridLayout()
        g.setHorizontalSpacing(10)
        self.vals = []
        for i, (t, u) in enumerate((("Velocidad", "rpm · Kalman"), ("Corriente", "mA · motor (Kalman)"), ("PWM", "% real"))):
            g.addWidget(label(t, "muted"), 0, i)
            v = label("—", "big")
            g.addWidget(v, 1, i)
            g.addWidget(label(u, "unit"), 2, i)
            self.vals.append(v)
        lay.addLayout(g)

        row = QtWidgets.QHBoxLayout()
        row.setSpacing(6)
        row.addWidget(label("Potencia", "muted"))
        self.b_rev = button("◀", "seg", "Sentido negativo", checkable=True)
        self.b_fwd = button("▶", "seg", "Sentido positivo", checkable=True)
        grp = QtWidgets.QButtonGroup(self)
        grp.setExclusive(True)
        for b in (self.b_rev, self.b_fwd):
            b.setFixedWidth(34)
            grp.addButton(b)
            b.clicked.connect(self._show_pwm)
            row.addWidget(b)
        self.b_fwd.setChecked(True)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)
        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setToolTip("Potencia manual en lazo abierto, de 0 a 100 %. 1 % = apenas arranca, 100 % = el tope de "
                               "este motor (se cambia en Sintonía). Se aplica SOLO mientras mantienes el dead-man en modo Potencia.")
        self.slider.valueChanged.connect(self._show_pwm)
        row.addWidget(self.slider, 1)
        self.pct_lbl = button("0 %", "seg")
        self.pct_lbl.setMinimumWidth(58)
        self.pct_lbl.setToolTip("Clic: poner en 0")
        self.pct_lbl.clicked.connect(lambda: self.slider.setValue(0))
        row.addWidget(self.pct_lbl)
        lay.addLayout(row)
        self.pwm_lbl = label("", "unit")
        lay.addWidget(self.pwm_lbl)
        self._show_pwm()

    def _toggled(self, on):
        self.panel.send("motor %d %s" % (self.m + 1, "on" if on else "off"))

    def _signed(self):
        return float(self.slider.value()) * (-1.0 if self.b_rev.isChecked() else 1.0)

    def _pwm(self):
        cfg = self.panel.pow_cfg[self.m]
        return sc.potencia_a_pwm(self._signed(), cfg["arranque"], cfg["tope"])

    def _show_pwm(self, *_):
        v = self.slider.value()
        self.pct_lbl.setText(("%d %%" % v) if v == 0 else ("%s %d %%" % ("◀" if self.b_rev.isChecked() else "▶", v)))
        cfg = self.panel.pow_cfg[self.m]
        self.pwm_lbl.setText("= %+.0f %% de PWM   ·   escala de este motor: %.0f–%.0f %% de PWM" % (
            self._pwm(), cfg["arranque"], cfg["tope"]))

    def pct(self):
        """% de PWM real con signo que se manda por /scara/power."""
        return self._pwm() if self.toggle.isChecked() else 0.0

    def update_view(self, st, ob):
        if st is not None:
            on = bool(st["enabled"] >> self.m & 1)
            if self.toggle.isChecked() != on:
                self.toggle.setChecked(on)
            self.slider.setEnabled(on)
            self.state_lbl.setText("habilitado" if on else "libre")
            duty = st["duty"][self.m]
            self.vals[2].setText("—" if duty != duty else "%+.0f" % (duty / 10.23))
        if ob is not None:
            self.vals[0].setText("%+.1f" % ob["wK_rpm"])
            self.vals[1].setText("%+.0f" % (ob["iK_A"] * 1000))


# ================================================================ ventana principal
class Panel(QtWidgets.QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self.b = bridge
        self.setWindowTitle("SCARA · Panel de control" + ("  (DEMO)" if bridge.demo else ""))
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.resize(1560, 960)
        self.t0 = time.monotonic()
        self.st = None
        self.ob = None
        self.t_state = 0.0
        self.sel_motor = 0
        self.pressed = set()
        self.mouse_dm = False
        self.jog_btn = (0, 0)
        self.manual = "pos"
        self.paused = False
        self.window_s = 15.0
        self.csv_w = None
        self.csv_f = None
        self.gain_read = False
        self.hist = []
        self.hist_i = 0
        # historias
        n = int(HIST_S * 25)
        self.h_t = collections.deque(maxlen=n)
        self.h_obs = collections.deque(maxlen=n)          # (t, [dict x3])
        self.h_st = collections.deque(maxlen=n)           # (t, dict)
        self.trail = collections.deque(maxlen=600)
        self.target = None
        self.h_sup = collections.deque(maxlen=n)          # (t, corriente estimada de la fuente en A)
        self.i_sup = None
        # escala de la potencia manual y corriente de la parte lógica (se recuerdan entre sesiones)
        self.settings = QtCore.QSettings("SCARA", "panel")
        self.pow_cfg = [dict(d) for d in sc.POWER_DEFAULT]
        for m, cfg in enumerate(self.pow_cfg):
            for k in cfg:
                cfg[k] = self._setting("potencia/M%d_%s" % (m + 1, k), cfg[k])
        self.i_logica = self._setting("fuente/i_logica_A", sc.I_LOGICA_DEFAULT)
        self._build()
        app = QtWidgets.QApplication.instance()
        app.installEventFilter(self)
        app.applicationStateChanged.connect(self._app_state)
        self.t_hb = QtCore.QTimer(self)
        self.t_hb.timeout.connect(self._heartbeat)
        self.t_hb.start(int(1000 / HB_HZ))
        self.t_ui = QtCore.QTimer(self)
        self.t_ui.timeout.connect(self._refresh)
        self.t_ui.start(50)

    def _setting(self, key, default):
        try:
            return float(self.settings.value(key, default))
        except (TypeError, ValueError):
            return float(default)

    def _set_pow(self, m, key, value):
        self.pow_cfg[m][key] = float(value)
        self.settings.setValue("potencia/M%d_%s" % (m + 1, key), float(value))
        self.cards[m]._show_pwm()

    def _set_ilog(self, ma):
        self.i_logica = ma / 1000.0
        self.settings.setValue("fuente/i_logica_A", self.i_logica)

    # ------------------------------------------------------------ construcción
    def _build(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(14, 12, 14, 12)
        outer.setSpacing(10)
        outer.addWidget(self._header())
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split.setChildrenCollapsible(False)
        split.addWidget(self._sidebar())
        right = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        right.setChildrenCollapsible(False)
        right.addWidget(self._tabs())
        right.addWidget(self._console())
        right.setSizes([700, 210])
        split.addWidget(right)
        split.setSizes([400, 1140])
        outer.addWidget(split, 1)

    def _header(self):
        h = QtWidgets.QFrame()
        h.setObjectName("header")
        lay = QtWidgets.QHBoxLayout(h)
        lay.setContentsMargins(16, 10, 16, 10)
        lay.setSpacing(12)
        tl = QtWidgets.QVBoxLayout()
        tl.setSpacing(0)
        tl.addWidget(label("SCARA", "title"))
        tl.addWidget(label("ESP32 · micro-ROS · ROS 2 Jazzy" + ("  ·  MODO DEMO (simulado)" if self.b.demo else ""), "subtitle"))
        lay.addLayout(tl)
        lay.addSpacing(16)
        self.p_conn, self.p_mode, self.p_fault = Pill(), Pill(), Pill()
        self.p_home1, self.p_home2, self.p_pow, self.p_sup = Pill(), Pill(), Pill(), Pill()
        self.p_sup.setToolTip("Corriente que debería marcar la fuente: parte lógica (Sintonía) + Σ |i motor| · duty.\n"
                              "Compárala con el amperímetro de la fuente.")
        for w in (self.p_conn, self.p_mode, self.p_home1, self.p_home2, self.p_pow, self.p_sup, self.p_fault):
            lay.addWidget(w)
        lay.addSpacing(8)
        lay.addWidget(label("Topes", "muted"))
        self.leds = [Led(t) for t in ("J1−", "J1+", "J2−", "J2+")]
        for l_ in self.leds:
            lay.addWidget(l_)
        lay.addStretch(1)
        for txt, cmd, name, tip in (("HOME", "home", "primary", "Referencia con los finales de carrera (h)"),
                                    ("DETENER", "stop", None, "Detiene y mantiene la posición (s / Esc)"),
                                    ("BORRAR FALLA", "clear", "warn", "Borra la falla; después hay que repetir Home (c)"),
                                    ("APAGAR", "off", "danger", "Apaga todos los motores (o)")):
            b = button(txt, name, tip)
            b.setMinimumHeight(38)
            b.clicked.connect(lambda _c, c=cmd: self.send(c))
            lay.addWidget(b)
        return h

    def _sidebar(self):
        sa = QtWidgets.QScrollArea()
        sa.setWidgetResizable(True)
        sa.setMinimumWidth(380)
        w = QtWidgets.QWidget()
        sa.setWidget(w)
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 6, 0)
        lay.setSpacing(10)
        lay.addWidget(label("Motores", "section"))
        self.cards = [MotorCard(m, self) for m in range(sc.NM)]
        for c in self.cards:
            lay.addWidget(c)
        lay.addWidget(self._manual_card())
        lay.addWidget(self._move_card())
        lay.addStretch(1)
        return sa

    def _manual_card(self):
        f = card()
        lay = QtWidgets.QVBoxLayout(f)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(8)
        lay.addWidget(label("Control manual con dead-man", "section"))
        seg = QtWidgets.QHBoxLayout()
        self.b_pos = button("Posición · flechas", "seg", checkable=True)
        self.b_pow = button("Potencia · deslizadores", "seg", checkable=True)
        grp = QtWidgets.QButtonGroup(self)
        grp.setExclusive(True)
        for b in (self.b_pos, self.b_pow):
            grp.addButton(b)
            seg.addWidget(b)
        self.b_pos.setChecked(True)
        self.b_pos.clicked.connect(lambda: self._set_manual("pos"))
        self.b_pow.clicked.connect(lambda: self._set_manual("pow"))
        lay.addLayout(seg)
        self.dm_btn = button("MANTENER PARA MOVER   ·   o ESPACIO", "deadman")
        self.dm_btn.pressed.connect(lambda: self._mouse_dm(True))
        self.dm_btn.released.connect(lambda: self._mouse_dm(False))
        lay.addWidget(self.dm_btn)
        pad = QtWidgets.QGridLayout()
        pad.setSpacing(6)
        self.jog_buttons = []
        for (txt, d, r, c_) in (("◀ J1", (-1, 0), 1, 0), ("J1 ▶", (1, 0), 1, 2), ("▲ J2", (0, 1), 0, 1), ("▼ J2", (0, -1), 2, 1)):
            b = button(txt, "jog", "Mantener: mueve el eje (el botón ya hace de dead-man)")
            b.pressed.connect(lambda d=d: self._jog_btn(d))
            b.released.connect(lambda: self._jog_btn((0, 0)))
            pad.addWidget(b, r, c_)
            self.jog_buttons.append(b)
        self.dm_lbl = label("", align=QtCore.Qt.AlignCenter)
        pad.addWidget(self.dm_lbl, 1, 1)
        lay.addLayout(pad)
        zero = button("Poner todas las potencias en 0")
        zero.clicked.connect(lambda: [c.slider.setValue(0) for c in self.cards])
        lay.addWidget(zero)
        self.manual_hint = label("", "muted")
        self.manual_hint.setWordWrap(True)
        lay.addWidget(self.manual_hint)
        self._set_manual("pos")
        return f

    def _move_card(self):
        f = card()
        lay = QtWidgets.QVBoxLayout(f)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(8)
        lay.addWidget(label("Mover a una consigna", "section"))
        g = QtWidgets.QGridLayout()
        g.setSpacing(6)
        self.sp_xyz = []
        for i, (t, v) in enumerate((("x", 120.0), ("y", 80.0), ("z", 0.0))):
            g.addWidget(label(t + " [mm]", "muted"), 0, i)
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(-300, 300)
            s.setDecimals(1)
            s.setValue(v)
            g.addWidget(s, 1, i)
            self.sp_xyz.append(s)
        bx = button("Ir a (x, y, z)", "primary")
        bx.clicked.connect(self._do_goto)
        g.addWidget(bx, 1, 3)
        self.sp_q = []
        for i, (t, v) in enumerate((("q1 [°]", 20.0), ("q2 [°]", 90.0))):
            g.addWidget(label(t, "muted"), 2, i)
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(-180, 180)
            s.setDecimals(1)
            s.setValue(v)
            g.addWidget(s, 3, i)
            self.sp_q.append(s)
        ba = button("Ir a ángulos")
        ba.clicked.connect(self._do_ang)
        g.addWidget(ba, 3, 3)
        lay.addLayout(g)
        er = QtWidgets.QHBoxLayout()
        er.addWidget(label("Codo", "muted"))
        self.b_elbow = []
        eg = QtWidgets.QButtonGroup(self)
        for v in ("+1", "-1"):
            b = button("codo " + v, "seg", checkable=True)
            b.clicked.connect(lambda _c, v=v: self.send("elbow " + v))
            eg.addButton(b)
            er.addWidget(b)
            self.b_elbow.append(b)
        self.b_elbow[0].setChecked(True)
        er.addStretch(1)
        lay.addLayout(er)
        lay.addWidget(label("Tip: en la pestaña Posición, haz clic en el plano para elegir (x, y).", "muted"))
        return f

    # ---- pestañas
    def _plot(self, title, ylabel, legend=True):
        w = pg.PlotWidget()
        w.setBackground(C["card"])
        pi = w.getPlotItem()
        pi.setTitle("<span style='color:%s;font-size:10pt;font-weight:600'>%s</span>" % (C["text"], title))
        pi.showGrid(x=True, y=True, alpha=0.15)
        for ax in ("left", "bottom"):
            a = pi.getAxis(ax)
            a.setPen(pg.mkPen(C["line"]))
            a.setTextPen(pg.mkPen(C["muted"]))
            a.enableAutoSIPrefix(False)
        pi.setLabel("left", ylabel, color=C["muted"])
        pi.setMenuEnabled(False)
        w.setMouseEnabled(x=False, y=True)
        if legend:
            lg = pi.addLegend(offset=(8, 6))
            lg.setBrush(pg.mkBrush(11, 18, 32, 200))
            lg.setPen(pg.mkPen(C["line"]))
            lg.setLabelTextColor(C["text"])
        return w

    @staticmethod
    def _curve(plot, name, color, width=2.0, dash=False):
        pen = pg.mkPen(color, width=width, style=QtCore.Qt.DashLine if dash else QtCore.Qt.SolidLine)
        return plot.plot([], [], pen=pen, name=name)

    def _tabs(self):
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(False)
        self.tabs.addTab(self._tab_obs(), "Observadores")
        self.tabs.addTab(self._tab_motors(), "Motores")
        self.tabs.addTab(self._tab_pos(), "Posición")
        self.tabs.addTab(self._tab_tune(), "Sintonía")
        return self.tabs

    def _tab_obs(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(12, 10, 12, 10)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(label("Motor", "muted"))
        grp = QtWidgets.QButtonGroup(self)
        self.b_selm = []
        for m in range(sc.NM):
            b = button(sc.MOTOR_NAMES[m], "seg", checkable=True)
            b.clicked.connect(lambda _c, m=m: self._sel(m))
            grp.addButton(b)
            top.addWidget(b)
            self.b_selm.append(b)
        self.b_selm[0].setChecked(True)
        top.addStretch(1)
        self.obs_info = label("", "muted")
        top.addWidget(self.obs_info)
        rst = button("Reiniciar observadores", None, "x̂ = 0 y P = P0: sirve para ver cómo convergen (r)")
        rst.clicked.connect(lambda: self.send("est reset"))
        top.addWidget(rst)
        lay.addLayout(top)
        self.pv = self._plot("Velocidad del eje de salida", "rpm")
        self.c_v = [self._curve(self.pv, "medida (encoder)", C["gray"], 1.2),
                    self._curve(self.pv, "Luenberger", C["green"], 1.6),
                    self._curve(self.pv, "Kalman a priori", C["orange"], 1.6, True),
                    self._curve(self.pv, "Kalman a posteriori", C["cyan"], 2.4)]
        self.pi = self._plot("Corriente de armadura estimada (no se mide)", "mA")
        self.c_i = [self._curve(self.pi, "Luenberger", C["green"], 1.4),
                    self._curve(self.pi, "Kalman a priori", C["orange"], 1.6, True),
                    self._curve(self.pi, "Kalman a posteriori", C["cyan"], 2.4)]
        lay.addWidget(self.pv, 3)
        lay.addWidget(self.pi, 3)
        row = QtWidgets.QHBoxLayout()
        self.psw = self._plot("Incertidumbre √P velocidad", "rpm", False)
        self.psi = self._plot("Incertidumbre √P corriente", "mA", False)
        self.pk = self._plot("Ganancia de Kalman (velocidad)", "K_w", False)
        self.c_sw = self._curve(self.psw, "σw", C["violet"], 2)
        self.c_si = self._curve(self.psi, "σi", C["violet"], 2)
        self.c_k = self._curve(self.pk, "Kw", C["amber"], 2)
        for p in (self.psw, self.psi, self.pk):
            row.addWidget(p)
        lay.addLayout(row, 2)
        return w

    def _tab_motors(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(12, 10, 12, 10)
        self.pm_v = self._plot("Velocidad (Kalman) de los 3 motores", "rpm")
        self.pm_i = self._plot("Corriente: motores (Kalman) y fuente estimada", "mA")
        self.c_sup = self._curve(self.pm_i, "fuente = lógica + Σ|i|·duty", C["text"], 1.6, True)
        self.pm_d = self._plot("PWM aplicado", "%")
        self.pm_u = self._plot("Voltaje del modelo (entrada de los observadores)", "V")
        self.c_mv, self.c_mi, self.c_md, self.c_mu = [], [], [], []
        for m in range(sc.NM):
            n = "M%d" % (m + 1)
            self.c_mv.append(self._curve(self.pm_v, n, MOTOR_COLORS[m]))
            self.c_mi.append(self._curve(self.pm_i, n, MOTOR_COLORS[m]))
            self.c_md.append(self._curve(self.pm_d, n, MOTOR_COLORS[m]))
            self.c_mu.append(self._curve(self.pm_u, n, MOTOR_COLORS[m]))
        g.addWidget(self.pm_v, 0, 0)
        g.addWidget(self.pm_i, 0, 1)
        g.addWidget(self.pm_d, 1, 0)
        g.addWidget(self.pm_u, 1, 1)
        return w

    def _tab_pos(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(w)
        lay.setContentsMargins(12, 10, 12, 10)
        self.pxy = self._plot("Espacio de trabajo (vista superior) · clic = consigna", "y [mm]", False)
        pi = self.pxy.getPlotItem()
        pi.setLabel("bottom", "x [mm]", color=C["muted"])
        pi.setAspectLocked(True)
        self.pxy.setMouseEnabled(x=True, y=True)
        r = sc.L1_MM + sc.L2_MM
        pi.setXRange(-r - 20, r + 20)
        pi.setYRange(-r - 20, r + 20)
        ang = np.linspace(0, 2 * np.pi, 200)
        for rad in (sc.L1_MM + sc.L2_MM, abs(sc.L1_MM - sc.L2_MM)):
            self.pxy.plot(rad * np.cos(ang), rad * np.sin(ang), pen=pg.mkPen("#2f855a", width=1.5, style=QtCore.Qt.DashLine))
        self.c_trail = self.pxy.plot([], [], pen=pg.mkPen(C["violet"] + "aa", width=1.5))
        self.c_arm = self.pxy.plot([], [], pen=pg.mkPen(C["text"], width=6), symbol="o", symbolSize=12,
                                   symbolBrush=C["card2"], symbolPen=pg.mkPen(C["cyan"], width=2))
        self.c_tgt = pg.ScatterPlotItem(size=16, pen=pg.mkPen(C["green"], width=2), brush=pg.mkBrush(52, 211, 153, 80), symbol="+")
        self.pxy.addItem(self.c_tgt)
        self.pxy.scene().sigMouseClicked.connect(self._xy_click)
        lay.addWidget(self.pxy, 3)
        col = QtWidgets.QVBoxLayout()
        self.pq = self._plot("Ángulos medidos y referencia", "grados")
        self.c_q = [self._curve(self.pq, "q1", MOTOR_COLORS[0]), self._curve(self.pq, "q2", MOTOR_COLORS[1]),
                    self._curve(self.pq, "q1 ref", MOTOR_COLORS[0], 1.2, True), self._curve(self.pq, "q2 ref", MOTOR_COLORS[1], 1.2, True)]
        col.addWidget(self.pq, 1)
        self.pos_lbl = label("", "big")
        col.addWidget(self.pos_lbl)
        lay.addLayout(col, 2)
        return w

    def _tab_tune(self):
        w = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(w)
        outer.setContentsMargins(14, 12, 14, 12)
        outer.setSpacing(12)
        # PID
        f1 = card()
        l1 = QtWidgets.QGridLayout(f1)
        l1.setContentsMargins(14, 12, 14, 12)
        l1.addWidget(label("PID de posición por articulación (unidades de PWM, 1023 = 12 V)", "section"), 0, 0, 1, 8)
        names = ("kp", "ki", "kd", "kff", "fric")
        for c_, n in enumerate(names):
            l1.addWidget(label(n, "muted"), 1, c_ + 1)
        self.gain_sp = {}
        for r, j in enumerate((1, 2), start=2):
            l1.addWidget(label("J%d" % j, "section"), r, 0)
            sps = []
            for c_ in range(5):
                s = QtWidgets.QDoubleSpinBox()
                s.setRange(0, 50000)
                s.setDecimals(0)
                l1.addWidget(s, r, c_ + 1)
                sps.append(s)
            self.gain_sp[j] = sps
            b = button("Aplicar J%d" % j, "primary")
            b.clicked.connect(lambda _c, j=j: self._apply_gain(j))
            l1.addWidget(b, r, 6)
        br = button("Leer del robot")
        br.clicked.connect(lambda: self.send("gain"))
        l1.addWidget(br, 2, 7, 2, 1)
        l1.addWidget(label("Por defecto: J1 con el modelo identificado (5350 · 18500 · 295 · 85 · 70); J2 con las del Reporte 3 "
                           "(5000 · 3000 · 180 · 80 · 70), porque con las del modelo el eslabón 2 quedaba muy agresivo. "
                           "Si un eje vibra o busca la posición, baja primero ki y luego kd. Los cambios se pierden al reiniciar.", "muted"), 4, 0, 1, 8)
        outer.addWidget(f1)
        # observadores
        f2 = card()
        l2 = QtWidgets.QGridLayout(f2)
        l2.setContentsMargins(14, 12, 14, 12)
        l2.addWidget(label("Observadores por motor", "section"), 0, 0, 1, 9)
        heads = ("polo re", "polo im", "", "σ corriente [mA]", "σ modelo vel. [rpm]", "σ medida [rpm]")
        for c_, n in enumerate(heads):
            if n:
                l2.addWidget(label(n, "muted"), 1, c_ + 1)
        self.est_sp = {}
        for r, m in enumerate(range(sc.NM), start=2):
            l2.addWidget(label("M%d" % (m + 1), "section"), r, 0)
            vals = (0.25, 0.25, None, 10.0, 0.30, 0.81)
            sps = []
            for c_, v in enumerate(vals):
                if v is None:
                    b = button("Aplicar Luenberger")
                    b.clicked.connect(lambda _c, m=m: self._apply_luen(m))
                    l2.addWidget(b, r, c_ + 1)
                    sps.append(None)
                    continue
                s = QtWidgets.QDoubleSpinBox()
                s.setDecimals(3)
                s.setSingleStep(0.05)
                s.setRange(-0.99, 0.99) if c_ < 2 else s.setRange(0.001, 1000)
                s.setValue(v)
                l2.addWidget(s, r, c_ + 1)
                sps.append(s)
            b = button("Aplicar Kalman")
            b.clicked.connect(lambda _c, m=m: self._apply_kal(m))
            l2.addWidget(b, r, 7)
            b2 = button("Leer")
            b2.clicked.connect(lambda _c, m=m: self.send("est %d" % (m + 1)))
            l2.addWidget(b2, r, 8)
            self.est_sp[m] = sps
        l2.addWidget(label("Luenberger: polos re ± j·im (|polo| < 1; más cerca de 0 = más rápido y más ruidoso). "
                           "Kalman: Q = diag(σi², σw²), R = σy²; R se midió en el laboratorio (0,655 rpm²).", "muted"), 5, 0, 1, 9)
        outer.addWidget(f2)
        # voltaje del modelo
        f3 = card()
        l3 = QtWidgets.QHBoxLayout(f3)
        l3.setContentsMargins(14, 12, 14, 12)
        l3.addWidget(label("Voltaje del modelo  V = g · (12 V · duty) − v0", "section"))
        self.sp_vg = QtWidgets.QDoubleSpinBox()
        self.sp_vg.setRange(0.31, 1.49)
        self.sp_vg.setDecimals(4)
        self.sp_vg.setValue(1.0)
        self.sp_v0 = QtWidgets.QDoubleSpinBox()
        self.sp_v0.setRange(0, 5.9)
        self.sp_v0.setDecimals(3)
        self.sp_v0.setValue(0.8)
        for t, s in (("g", self.sp_vg), ("v0 [V]", self.sp_v0)):
            l3.addWidget(label(t, "muted"))
            l3.addWidget(s)
        bv = button("Aplicar", "primary")
        bv.clicked.connect(lambda: self.send("vmap %.4f %.3f" % (self.sp_vg.value(), self.sp_v0.value())))
        l3.addWidget(bv)
        bl = button("Valores del L298N (lab)")
        bl.clicked.connect(lambda: (self.sp_vg.setValue(0.9782), self.sp_v0.setValue(1.197)))
        l3.addWidget(bl)
        bt = button("TB6612 (tarjeta)")
        bt.clicked.connect(lambda: (self.sp_vg.setValue(1.0), self.sp_v0.setValue(0.8)))
        l3.addWidget(bt)
        l3.addStretch(1)
        outer.addWidget(f3)
        # potencia manual y fuente
        f4 = card()
        l4 = QtWidgets.QGridLayout(f4)
        l4.setContentsMargins(14, 12, 14, 12)
        l4.addWidget(label("Escala de los deslizadores de potencia (0–100 %)", "section"), 0, 0, 1, 6)
        l4.addWidget(label("arranque [% PWM] = 1 %", "muted"), 1, 1)
        l4.addWidget(label("tope [% PWM] = 100 %", "muted"), 1, 2)
        for r, m in enumerate(range(sc.NM), start=2):
            l4.addWidget(label(sc.MOTOR_NAMES[m], "section"), r, 0)
            for c_, key in enumerate(("arranque", "tope"), start=1):
                s = QtWidgets.QDoubleSpinBox()
                s.setDecimals(0)
                s.setRange(0, int(sc.DUTY_MAX_PCT))
                s.setSuffix(" %")
                s.setValue(self.pow_cfg[m][key])
                s.valueChanged.connect(lambda v, m=m, key=key: self._set_pow(m, key, v))
                l4.addWidget(s, r, c_)
        l4.addWidget(label("Arranque: el PWM con el que el motor apenas empieza a girar (súbelo si con 1–5 %% no se mueve). "
                           "Tope: el PWM al 100 %% del deslizador (máximo %d %%, límite del firmware). "
                           "Se guardan en este PC." % int(sc.DUTY_MAX_PCT), "muted"), 5, 0, 1, 6)
        l4.addWidget(label("Corriente de la parte lógica", "section"), 6, 0)
        self.sp_ilog = QtWidgets.QDoubleSpinBox()
        self.sp_ilog.setDecimals(0)
        self.sp_ilog.setRange(0, 1000)
        self.sp_ilog.setSuffix(" mA")
        self.sp_ilog.setValue(self.i_logica * 1000)
        self.sp_ilog.valueChanged.connect(self._set_ilog)
        l4.addWidget(self.sp_ilog, 6, 1)
        l4.addWidget(label("ESP32, reguladores, encoders y driver. Mídela en la fuente con el robot conectado y los motores "
                           "APAGADOS, y escríbela aquí: el encabezado mostrará lo que debería marcar la fuente "
                           "(lógica + Σ |i motor| · duty).", "muted"), 7, 0, 1, 6)
        l4.setColumnStretch(5, 1)
        for lb in f4.findChildren(QtWidgets.QLabel):
            if lb.objectName() == "muted":
                lb.setWordWrap(True)
        outer.addWidget(f4)
        outer.addStretch(1)
        return w

    def _console(self):
        f = card()
        lay = QtWidgets.QVBoxLayout(f)
        lay.setContentsMargins(12, 10, 12, 10)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(label("Registro de eventos", "section"))
        top.addStretch(1)
        top.addWidget(label("Ventana", "muted"))
        self.cb_win = QtWidgets.QComboBox()
        for s in ("5 s", "10 s", "15 s", "30 s", "60 s"):
            self.cb_win.addItem(s)
        self.cb_win.setCurrentIndex(2)
        self.cb_win.setFocusPolicy(QtCore.Qt.NoFocus)
        self.cb_win.currentTextChanged.connect(lambda t: setattr(self, "window_s", float(t.split()[0])))
        top.addWidget(self.cb_win)
        self.b_pause = button("Pausar gráficas", checkable=True)
        self.b_pause.toggled.connect(lambda on: setattr(self, "paused", on))
        top.addWidget(self.b_pause)
        self.b_rec = button("● Grabar CSV", checkable=True)
        self.b_rec.toggled.connect(self._rec)
        top.addWidget(self.b_rec)
        lay.addLayout(top)
        self.log_w = QtWidgets.QPlainTextEdit()
        self.log_w.setReadOnly(True)
        self.log_w.setMaximumBlockCount(800)
        self.log_w.setFocusPolicy(QtCore.Qt.ClickFocus)
        lay.addWidget(self.log_w, 1)
        row = QtWidgets.QHBoxLayout()
        self.cmd = QtWidgets.QLineEdit()
        self.cmd.setPlaceholderText("Comando (igual que el monitor serie): help, status, gain, motor 3 on, est 1, goto 120 80 …   ↑/↓ historial · Esc vuelve al teclado del robot")
        self.cmd.returnPressed.connect(self._do_cmdline)
        row.addWidget(self.cmd, 1)
        bs = button("Enviar", "primary")
        bs.clicked.connect(self._do_cmdline)
        row.addWidget(bs)
        lay.addLayout(row)
        return f

    # ------------------------------------------------------------ acciones
    def log(self, text, color=None):
        col = color or (C["red"] if text.startswith(("#ERROR", "#FALLA")) else C["green"] if text.startswith(("#LLEGO", "#OK", "#HOME: terminado"))
                        else C["amber"] if text.startswith(("#MOTOR", "#EST", "#HOME")) else C["text"])
        ts = time.strftime("%H:%M:%S")
        safe = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.log_w.appendHtml("<span style='color:%s'>%s</span>&nbsp;&nbsp;<span style='color:%s'>%s</span>" % (C["muted"], ts, col, safe))

    def send(self, text):
        self.b.cmd(text)
        self.log("> " + text, C["blue"])

    def _do_goto(self):
        x, y, z = (s.value() for s in self.sp_xyz)
        self.target = (x, y)
        self.b.goto(x, y, z)
        self.log("> goto %.1f %.1f %.1f" % (x, y, z), C["blue"])

    def _do_ang(self):
        self.target = sc.fk(self.sp_q[0].value(), self.sp_q[1].value())
        self.send("ang %.2f %.2f" % (self.sp_q[0].value(), self.sp_q[1].value()))

    def _apply_gain(self, j):
        v = [s.value() for s in self.gain_sp[j]]
        self.send("gain %d %g %g %g %g %g" % ((j,) + tuple(v)))
        QtCore.QTimer.singleShot(300, lambda: self.send("gain"))

    def _apply_luen(self, m):
        s = self.est_sp[m]
        self.send("luen %d %.4f %.4f" % (m + 1, s[0].value(), s[1].value()))

    def _apply_kal(self, m):
        s = self.est_sp[m]
        self.send("kal %d %.4g %.4g %.4g" % (m + 1, s[3].value(), s[4].value(), s[5].value()))

    def _do_cmdline(self):
        t = self.cmd.text().strip()
        if not t:
            return
        self.send(t)
        if not self.hist or self.hist[-1] != t:
            self.hist.append(t)
        self.hist_i = len(self.hist)
        self.cmd.clear()

    def _sel(self, m):
        self.sel_motor = m

    def _set_manual(self, mode):
        self.manual = mode
        pos = mode == "pos"
        for b in self.jog_buttons:
            b.setEnabled(pos)
        self.manual_hint.setText(
            "Posición: mantén ESPACIO (o el botón) y usa las flechas ← → (J1) ↓ ↑ (J2). También puedes mantener los botones de flecha."
            if pos else
            "Potencia: elige el sentido (◀ ▶) y el % (0–100) de cada motor y MANTÉN el dead-man para aplicarlos (lazo abierto). "
            "Los ejes con 0 % mantienen su posición; al soltar, todo frena y se queda quieto.")

    def _mouse_dm(self, on):
        self.mouse_dm = on

    def _jog_btn(self, d):
        self.jog_btn = d

    def _xy_click(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            return
        vb = self.pxy.getPlotItem().vb
        if not vb.sceneBoundingRect().contains(ev.scenePos()):
            return
        p = vb.mapSceneToView(ev.scenePos())
        self.sp_xyz[0].setValue(round(p.x(), 1))
        self.sp_xyz[1].setValue(round(p.y(), 1))
        self.target = (p.x(), p.y())
        self.log("(consigna elegida en el plano: x=%.1f y=%.1f · pulsa 'Ir a (x, y, z)')" % (p.x(), p.y()), C["muted"])

    def _rec(self, on):
        if on:
            name = time.strftime("scara_%Y%m%d_%H%M%S.csv")
            path = os.path.join(os.getcwd(), name)
            self.csv_f = open(path, "w", newline="")
            self.csv_w = csv.writer(self.csv_f)
            head = ["t_s", "modo", "q1_deg", "q2_deg", "qref1_deg", "qref2_deg", "x_mm", "y_mm", "habilitados", "I_fuente_est_A"]
            for m in range(sc.NM):
                head += ["M%d_duty" % (m + 1)] + ["M%d_%s" % (m + 1, f) for f in sc.OBS_FIELDS]
            self.csv_w.writerow(head)
            self.b_rec.setText("■ Detener grabación")
            self.log("Grabando en %s" % path, C["amber"])
        else:
            if self.csv_f:
                self.csv_f.close()
                self.log("Grabación guardada (%s)" % self.csv_f.name, C["amber"])
            self.csv_f = self.csv_w = None
            self.b_rec.setText("● Grabar CSV")

    # ------------------------------------------------------------ teclado y dead-man
    def _typing(self):
        w = QtWidgets.QApplication.focusWidget()
        return isinstance(w, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox))

    def eventFilter(self, obj, ev):
        t = ev.type()
        if t == QtCore.QEvent.MouseButtonPress and self._typing() and \
                not isinstance(obj, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox)):
            self.setFocus()                                   # clic fuera de un cuadro: el teclado vuelve al robot
        if t in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease):
            key = ev.key()
            if self._typing():
                if t == QtCore.QEvent.KeyPress and key == QtCore.Qt.Key_Escape:
                    self.setFocus()
                    return True
                if obj is self.cmd and t == QtCore.QEvent.KeyPress and key in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down) and self.hist:
                    self.hist_i = max(0, min(len(self.hist), self.hist_i + (-1 if key == QtCore.Qt.Key_Up else 1)))
                    self.cmd.setText(self.hist[self.hist_i] if self.hist_i < len(self.hist) else "")
                    return True
                return False
            if ev.isAutoRepeat():
                return key in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Left, QtCore.Qt.Key_Right, QtCore.Qt.Key_Up, QtCore.Qt.Key_Down)
            names = {QtCore.Qt.Key_Space: "space", QtCore.Qt.Key_Left: "left", QtCore.Qt.Key_Right: "right",
                     QtCore.Qt.Key_Up: "up", QtCore.Qt.Key_Down: "down"}
            if key in names:
                (self.pressed.add if t == QtCore.QEvent.KeyPress else self.pressed.discard)(names[key])
                self._heartbeat()
                return True
            if t == QtCore.QEvent.KeyPress:
                shortcuts = {QtCore.Qt.Key_H: "home", QtCore.Qt.Key_S: "stop", QtCore.Qt.Key_O: "off",
                             QtCore.Qt.Key_C: "clear", QtCore.Qt.Key_Escape: "stop", QtCore.Qt.Key_R: "est reset"}
                if key in shortcuts and not (ev.modifiers() & (QtCore.Qt.ControlModifier | QtCore.Qt.AltModifier)):
                    self.send(shortcuts[key])
                    return True
        return super().eventFilter(obj, ev)

    def _app_state(self, state):
        if state != QtCore.Qt.ApplicationActive:
            self.pressed.clear()
            self.mouse_dm = False
            self.jog_btn = (0, 0)
            self._heartbeat()

    def _dm_state(self):
        keys = self.pressed
        dm = "space" in keys or self.mouse_dm or self.jog_btn != (0, 0)
        d1 = (1 if "right" in keys else 0) - (1 if "left" in keys else 0) + self.jog_btn[0]
        d2 = (1 if "up" in keys else 0) - (1 if "down" in keys else 0) + self.jog_btn[1]
        return dm, max(-1, min(1, d1)), max(-1, min(1, d2))

    def _heartbeat(self):
        dm, d1, d2 = self._dm_state()
        pct = [c.pct() for c in self.cards]
        if self.manual == "pos":
            self.b.jog(dm, d1, d2)
            self.b.power(False, (0.0, 0.0, 0.0))
        else:
            self.b.jog(False, 0, 0)
            self.b.power(dm, pct)
        if dm:
            txt = ("DEAD-MAN ACTIVO\nJ1 %+d  J2 %+d" % (d1, d2)) if self.manual == "pos" else "DEAD-MAN ACTIVO\npotencia aplicada"
            self.dm_lbl.setText(txt)
            self.dm_lbl.setStyleSheet("color: %s; font-weight: 800;" % C["green"])
        else:
            self.dm_lbl.setText("dead-man\nsuelto")
            self.dm_lbl.setStyleSheet("color: %s;" % C["muted"])

    # ------------------------------------------------------------ datos y refresco
    def _drain(self):
        while self.b.states:
            t, d = self.b.states.popleft()
            st = sc.parse_state(d)
            if st:
                self.st, self.t_state = st, t
                self.h_st.append((t - self.t0, st))
                x, y = st["xy"]
                if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.8:
                    self.trail.append((x, y))
        while self.b.obs:
            t, d = self.b.obs.popleft()
            r = sc.parse_obs(d)
            if r:
                self.ob = r[1]
                self.h_obs.append((t - self.t0, r[1]))
                if self.st:
                    self.i_sup = sc.corriente_fuente(self.i_logica, [o["iK_A"] for o in r[1]],
                                                     [d / 1023.0 for d in self.st["duty"]])
                    self.h_sup.append((t - self.t0, self.i_sup))
                if self.csv_w and self.st:
                    s = self.st
                    row = [round(t - self.t0, 3), sc.mode_name(s["mode"]), s["q"][0], s["q"][1], s["qref"][0], s["qref"][1],
                           s["xy"][0], s["xy"][1], s["enabled"], self.i_sup]
                    for m in range(sc.NM):
                        row += [s["duty"][m]] + [r[1][m][f] for f in sc.OBS_FIELDS]
                    self.csv_w.writerow(row)
        while self.b.events:
            ev = self.b.events.popleft()
            self.log(ev)
            self._parse_event(ev)

    def _parse_event(self, ev):
        import re
        for m in re.finditer(r"J([12]) kp=([\d.]+) ki=([\d.]+) kd=([\d.]+) kff=([\d.]+) fric=([\d.]+)", ev):
            for s, v in zip(self.gain_sp[int(m.group(1))], m.groups()[1:]):
                s.setValue(float(v))
        m = re.match(r"#INFO: M(\d) polos ([-\d.]+)\+-([\d.]+)j .* sig_i=([\d.]+) mA sig_w=([\d.]+) rpm sig_y=([\d.]+) rpm", ev)
        if m:
            sps = self.est_sp[int(m.group(1)) - 1]
            for s, v in zip((sps[0], sps[1], sps[3], sps[4], sps[5]), m.groups()[1:]):
                s.setValue(float(v))

    def _refresh(self):
        self._drain()
        now = time.monotonic()
        conn = self.st is not None and now - self.t_state < STALE_S
        self.p_conn.set("● ESP32 conectado" if conn else "○ sin datos del ESP32", C["green"] if conn else C["red"])
        st = self.st
        if st:
            mn = sc.mode_name(st["mode"])
            self.p_mode.set("modo " + mn, MODE_COLORS.get(mn, C["muted"]))
            self.p_fault.set("falla: " + sc.fault_name(st["fault"]) if st["fault"] else "sin fallas", C["red"] if st["fault"] else C["muted"])
            for p, j in ((self.p_home1, 0), (self.p_home2, 1)):
                p.set("Home J%d %s" % (j + 1, "✓" if st["homed"][j] else "—"), C["green"] if st["homed"][j] else C["muted"])
            self.p_pow.set("potencia manual" if st["power_on"] else "potencia: no", C["orange"] if st["power_on"] else C["muted"])
            for i, l_ in enumerate(self.leds):
                l_.set(bool(st["fc"] >> i & 1))
            if not self.gain_read and conn:
                self.gain_read = True
                self.send("gain")
        else:
            self.p_mode.set("modo —", C["muted"])
            self.p_fault.set("sin datos", C["muted"])
            for p, j in ((self.p_home1, 0), (self.p_home2, 1)):
                p.set("Home J%d —" % (j + 1), C["muted"])
            self.p_pow.set("potencia: no", C["muted"])
        if conn and self.i_sup is not None:
            self.p_sup.set("fuente ≈ %.0f mA" % (self.i_sup * 1000), C["cyan"])
        else:
            self.p_sup.set("fuente —", C["muted"])
        for c in self.cards:
            c.update_view(st, self.ob[c.m] if self.ob else None)
        if self.ob:
            o = self.ob[self.sel_motor]
            self.obs_info.setText("Kw = %.3f   σw = %.2f rpm   σi = %.1f mA   V = %+.2f V%s" % (
                o["Kw"], o["sig_w_rpm"], o["sig_i_A"] * 1000, o["u_V"], "" if o["med"] else "   (sin encoder: solo modelo)"))
        if not self.paused:
            self._plots()

    def _plots(self):
        tnow = time.monotonic() - self.t0
        x0 = tnow - self.window_s
        idx = self.tabs.currentIndex()
        if idx in (0, 1) and self.h_obs:
            data = [(t, o) for t, o in self.h_obs if t >= x0 - 1]
            if data:
                t = np.array([d[0] for d in data])

                def col(m, f, k=1.0):
                    return np.array([d[1][m][f] for d in data], dtype=float) * k
                if idx == 0:
                    m = self.sel_motor
                    for c, f in zip(self.c_v, ("w_meas_rpm", "wL_rpm", "wKpri_rpm", "wK_rpm")):
                        c.setData(t, col(m, f), connect="finite")
                    for c, f in zip(self.c_i, ("iL_A", "iKpri_A", "iK_A")):
                        c.setData(t, col(m, f, 1000.0))
                    self.c_sw.setData(t, col(m, "sig_w_rpm"))
                    self.c_si.setData(t, col(m, "sig_i_A", 1000.0))
                    self.c_k.setData(t, col(m, "Kw"))
                    for p in (self.pv, self.pi, self.psw, self.psi, self.pk):
                        p.setXRange(x0, tnow, padding=0)
                else:
                    for m in range(sc.NM):
                        self.c_mv[m].setData(t, col(m, "wK_rpm"))
                        self.c_mi[m].setData(t, col(m, "iK_A", 1000.0))
                        self.c_mu[m].setData(t, col(m, "u_V"))
                    sup = [(ts, v) for ts, v in self.h_sup if ts >= x0 - 1]
                    if sup:
                        self.c_sup.setData(np.array([s_[0] for s_ in sup]), np.array([s_[1] for s_ in sup]) * 1000.0)
                    for p in (self.pm_v, self.pm_i, self.pm_d, self.pm_u):
                        p.setXRange(x0, tnow, padding=0)
        if idx == 1 and self.h_st:
            sd = [(t, s) for t, s in self.h_st if t >= x0 - 1]
            if sd:
                t = np.array([d[0] for d in sd])
                for m in range(sc.NM):
                    self.c_md[m].setData(t, np.array([d[1]["duty"][m] for d in sd], dtype=float) / 10.23)
        if idx == 2:
            if self.st:
                q1, q2 = self.st["q"]
                xe, ye = sc.L1_MM * math.cos(math.radians(q1)), sc.L1_MM * math.sin(math.radians(q1))
                x, y = self.st["xy"]
                self.c_arm.setData([0, xe, x], [0, ye, y])
                self.pos_lbl.setText("x = %.1f mm   y = %.1f mm   ·   q1 = %.2f°   q2 = %.2f°" % (x, y, q1, q2))
            if self.trail:
                tr = np.array(self.trail)
                self.c_trail.setData(tr[:, 0], tr[:, 1])
            self.c_tgt.setData([self.target[0]], [self.target[1]]) if self.target else self.c_tgt.setData([], [])
            sd = [(t, s) for t, s in self.h_st if t >= x0 - 1]
            if sd:
                t = np.array([d[0] for d in sd])
                for i, (k, j) in enumerate((("q", 0), ("q", 1), ("qref", 0), ("qref", 1))):
                    self.c_q[i].setData(t, np.array([d[1][k][j] for d in sd], dtype=float))
                self.pq.setXRange(x0, tnow, padding=0)

    def closeEvent(self, ev):
        self.t_hb.stop()
        self.t_ui.stop()
        if self.csv_f:
            self.csv_f.close()
        self.b.close()
        ev.accept()


def make_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setApplicationName("SCARA panel")
    app.setStyle("Fusion")
    app.setStyleSheet(QSS)
    pg.setConfigOptions(antialias=True, foreground=C["muted"])
    return app


def main():
    ap = argparse.ArgumentParser(description="Panel de control del SCARA (ROS 2 / micro-ROS)")
    ap.add_argument("--demo", action="store_true", help="sin robot: simula el ESP32 con el modelo identificado")
    a = ap.parse_args()
    app = make_app()
    if a.demo:
        bridge = DemoBridge()
    else:
        try:
            bridge = RosBridge()
        except ImportError:
            print("No se encontró rclpy. Carga ROS 2 (source /opt/ros/jazzy/setup.bash) o usa --demo.")
            return 1
    w = Panel(bridge)
    w.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
