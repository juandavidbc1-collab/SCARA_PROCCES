"""
Definiciones compartidas por el panel (scara_panel.py) y los demás scripts de ROS 2.

  - Formato de /scara/state y /scara/obs (debe coincidir con src/uros_comms.c).
  - Cinemática del SCARA (L1, L2 de src/config.c).
  - Modelo identificado del motor y observadores (Luenberger + Kalman), copia en Python de src/estimator.c,
    que usa el modo demo del panel para simular el robot sin hardware.

No importa rclpy: se puede usar y probar sin ROS 2.
"""
import math

import numpy as np

# ---------------------------------------------------------------- robot
L1_MM, L2_MM = 120.0, 70.0                    # mantener igual que KIN en src/config.c
NM = 3                                        # motores: M1 = J1, M2 = J2, M3 = atornillador
MOTOR_NAMES = ("M1 · Eslabón 1", "M2 · Eslabón 2", "M3 · Atornillador")
DUTY_MAX_PCT = 650 / 1023 * 100               # tope de PWM del firmware (DUTY_MAX) en %
MODES = ["OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT", "POWER"]
FAULTS = ["ninguna", "error de seguimiento", "motor trabado", "final de carrera inesperado",
          "tiempo de Home agotado", "MIN y MAX a la vez"]
RPM = math.pi / 30.0                          # rpm -> rad/s

# ---------------------------------------------------------------- /scara/state (Float32MultiArray)
(S_Q1, S_Q2, S_X, S_Y, S_MODE, S_FAULT, S_HOMED1, S_HOMED2, S_FC, S_QREF1, S_QREF2, S_DUTY1, S_DUTY2,
 S_ENABLED, S_DUTY3, S_POWER_ON, S_W1, S_W2, S_W3) = range(19)
STATE_N = 19

# ---------------------------------------------------------------- /scara/obs (Float32MultiArray)
OBS_HDR = 2
(O_W_MEAS, O_WL, O_WKPRI, O_WK, O_IL, O_IKPRI, O_IK, O_SIG_I, O_SIG_W, O_KW, O_KI, O_U) = range(12)
OBS_STRIDE = 12
OBS_N = OBS_HDR + NM * OBS_STRIDE
OBS_FIELDS = ("w_meas_rpm", "wL_rpm", "wKpri_rpm", "wK_rpm", "iL_A", "iKpri_A", "iK_A",
              "sig_i_A", "sig_w_rpm", "Kw", "Ki", "u_V")


def parse_state(data):
    """Lista de /scara/state -> dict. Acepta el formato viejo de 13 datos (sin motor 3 ni observadores)."""
    d = list(data)
    if len(d) < 13:
        return None
    d += [float("nan")] * (STATE_N - len(d))
    en = d[S_ENABLED]
    return {
        "q": (d[S_Q1], d[S_Q2]), "xy": (d[S_X], d[S_Y]), "mode": int(d[S_MODE]), "fault": int(d[S_FAULT]),
        "homed": (bool(d[S_HOMED1]), bool(d[S_HOMED2])), "fc": int(d[S_FC]), "qref": (d[S_QREF1], d[S_QREF2]),
        "duty": (d[S_DUTY1], d[S_DUTY2], d[S_DUTY3]),
        "enabled": (0x3 if math.isnan(en) else int(en)), "power_on": d[S_POWER_ON] == 1.0,
        "w": (d[S_W1], d[S_W2], d[S_W3]), "nuevo": len(data) >= STATE_N,
    }


def parse_obs(data):
    """Lista de /scara/obs -> (seq, [dict por motor]) o None."""
    d = list(data)
    if len(d) < OBS_N:
        return None
    flags = int(d[1])
    motors = []
    for m in range(NM):
        b = OBS_HDR + m * OBS_STRIDE
        rec = dict(zip(OBS_FIELDS, d[b:b + OBS_STRIDE]))
        rec["ok"] = bool(flags >> m & 1)
        rec["med"] = bool(flags >> (m + 3) & 1)
        motors.append(rec)
    return int(d[0]), motors


def mode_name(m):
    return MODES[m] if 0 <= m < len(MODES) else "?"


def fault_name(f):
    return FAULTS[f] if 0 <= f < len(FAULTS) else str(f)


# ---------------------------------------------------------------- cinemática
def fk(q1_deg, q2_deg):
    a, b = math.radians(q1_deg), math.radians(q1_deg + q2_deg)
    return (L1_MM * math.cos(a) + L2_MM * math.cos(b), L1_MM * math.sin(a) + L2_MM * math.sin(b))


def ik(x, y, elbow=+1):
    """(x, y) mm -> (q1, q2) grados o None si es inalcanzable."""
    c2 = (x * x + y * y - L1_MM ** 2 - L2_MM ** 2) / (2 * L1_MM * L2_MM)
    if abs(c2) > 1:
        return None
    q2 = elbow * math.acos(c2)
    q1 = math.atan2(y, x) - math.atan2(L2_MM * math.sin(q2), L1_MM + L2_MM * math.cos(q2))
    return math.degrees(q1), math.degrees(q2)


# ---------------------------------------------------------------- modelo identificado y observadores
# Eje de salida (src/config.h, MOT_*): La di/dt = V - Ra i - Kv w ;  J dw/dt = Kv i - B w
MOTOR = dict(ra=50.0, la=0.060, kv=0.167092, b=2.79197e-3, j=3.63232e-4)
OBS_DEFAULT = dict(polo_re=0.25, polo_im=0.25, sig_i=0.010, sig_w=0.30 * RPM, sig_y=0.81 * RPM,
                   p0_i=0.10, p0_w=1.0 * RPM)


def expm(m):
    """Exponencial de matriz por escalado y elevación al cuadrado + Taylor (sin scipy)."""
    m = np.asarray(m, dtype=float)
    norm = np.abs(m).sum(axis=1).max()
    sq = 0
    while norm > 0.25 and sq < 30:
        norm *= 0.5
        sq += 1
    a = m / (2 ** sq)
    term = np.eye(len(m))
    out = np.eye(len(m))
    for n in range(1, 15):
        term = term @ a / n
        out = out + term
    for _ in range(sq):
        out = out @ out
    return out


def discretize(model, ts):
    ra, la, kv, b, j = (model[k] for k in ("ra", "la", "kv", "b", "j"))
    ac = np.array([[-ra / la, -kv / la], [kv / j, -b / j]])
    bc = np.array([1.0 / la, 0.0])
    mm = np.zeros((3, 3))
    mm[:2, :2] = ac * ts
    mm[:2, 2] = bc * ts
    e = expm(mm)
    return e[:2, :2], e[:2, 2]


class Estimator:
    """Copia de src/estimator.c (mismas ecuaciones y el mismo orden de cálculo)."""

    def __init__(self, model=MOTOR, params=OBS_DEFAULT, ts=0.01):
        self.model, self.ts = dict(model), ts
        self.design(params)

    def design(self, params):
        self.p = dict(params)
        self.A, self.B = discretize(self.model, self.ts)
        re, im = self.p["polo_re"], self.p["polo_im"]
        a1, a0 = -2 * re, re * re + im * im
        A = self.A
        phi = A @ A + a1 * A + a0 * np.eye(2)
        self.ok = math.sqrt(a0) < 1 and abs(A[1, 0]) > 1e-9
        self.L = phi[:, 0] / A[1, 0] if abs(A[1, 0]) > 1e-9 else np.zeros(2)
        self.reset()

    def reset(self):
        p0 = np.diag([self.p["p0_i"] ** 2, self.p["p0_w"] ** 2])
        self.xl = np.zeros(2)
        self.xp = np.zeros(2)
        self.xk = np.zeros(2)
        self.Pm = p0.copy()
        self.Pk = p0.copy()
        self.K = np.zeros(2)
        self.y_prev = 0.0
        self.first = True

    def step(self, u_prev, y, med=True):
        A, B = self.A, self.B
        if self.first:
            self.y_prev = y if med else 0.0
            self.xp = self.xk.copy()
            self.K = np.zeros(2)
            self.first = False
            return
        inn = (self.y_prev - self.xl[1]) if med else 0.0
        self.xl = A @ self.xl + B * u_prev + self.L * inn
        self.y_prev = y if med else 0.0
        self.xp = A @ self.xk + B * u_prev
        Q = np.diag([self.p["sig_i"] ** 2, self.p["sig_w"] ** 2])
        self.Pm = A @ self.Pk @ A.T + Q
        if med:
            S = self.Pm[1, 1] + self.p["sig_y"] ** 2
            self.K = self.Pm[:, 1] / S
            self.xk = self.xp + self.K * (y - self.xp[1])
            self.Pk = self.Pm - np.outer(self.K, self.Pm[1, :])
        else:
            self.K = np.zeros(2)
            self.xk = self.xp.copy()
            self.Pk = self.Pm.copy()


def voltaje(duty_frac, vbus=12.0, vgain=1.0, v0=0.8):
    v = vgain * vbus * duty_frac
    a = abs(v) - v0
    return 0.0 if a <= 0 else math.copysign(a, v)


# ---------------------------------------------------------------- potencia manual (escala 0-100 %)
# El deslizador del panel va de 0 a 100 % y el sentido se elige aparte. 0 % = parado,
# 1 % = el PWM con el que el motor apenas arranca, 100 % = el tope elegido para ese motor.
# Lo que viaja por /scara/power sigue siendo % de PWM real (0-100 % de 1023, el firmware lo limita a ~63 %).
POWER_DEFAULT = (dict(arranque=7.0, tope=60.0),       # M1 · eslabón 1
                 dict(arranque=7.0, tope=30.0),       # M2 · eslabón 2 (eslabón liviano: con menos le basta)
                 dict(arranque=7.0, tope=60.0))       # M3 · atornillador


def potencia_a_pwm(pct, arranque, tope):
    """% del deslizador (-100..100, el signo es el sentido) -> % de PWM real con signo."""
    p = max(-100.0, min(100.0, float(pct)))
    if abs(p) < 0.5:
        return 0.0
    tope = max(0.0, min(DUTY_MAX_PCT, tope))
    arranque = max(0.0, min(tope, arranque))
    return math.copysign(arranque + (tope - arranque) * abs(p) / 100.0, p)


# ---------------------------------------------------------------- corriente de la fuente
# La fuente no ve la corriente de armadura: con PWM, cada motor le pide en promedio |i| * duty.
# Además alimenta la parte lógica (ESP32, reguladores, encoders, driver), que se mide en la fuente
# con los motores apagados y se escribe en el panel.
I_LOGICA_DEFAULT = 0.060                              # A, valor de arranque: medirlo en la fuente


def corriente_fuente(i_logica, corrientes_A, duties_frac):
    """Corriente que debería marcar la fuente: lógica + suma de |i_motor| * |duty|."""
    tot = float(i_logica)
    for i, d in zip(corrientes_A, duties_frac):
        if i == i and d == d:                          # ignora NaN
            tot += abs(i) * min(1.0, abs(d))
    return tot
