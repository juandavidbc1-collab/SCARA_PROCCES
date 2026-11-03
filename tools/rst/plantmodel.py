"""Lazo completo de una articulación en Python: motor, reductor, fricción, encoder, PWM, rampa y perfil trapezoidal.
Es el mismo modelo de planta que tools/host_tests/sim_control.c y tools/sim/sim_hal.c."""
import math
from dataclasses import dataclass

import rst_design as rd

N = 100.37
CPR = 6424.0
DUTY_MAX, SLEW_MS, FRIC, E_ZONE = 650.0, 3.0, 70.0, 0.0026
DT_PLANT, TICKS_MS, RST_TICKS = 1e-4, 10, 1


@dataclass
class Motor:
    """Hoja de datos del Pololu 37D, lado del motor. J_load: inercia del eslabón [kg m²] (estimada, sin medir)."""
    J_load: float = 0.04
    R: float = 2.4
    L: float = 1.5e-3
    Ke: float = 0.01056
    J_rotor: float = 3e-6
    B: float = 2.97e-6
    Tc: float = 3e-3

    def J(self):
        return self.J_rotor + self.J_load / (N * N)

    def K(self):
        """Ganancia de velocidad en el eje de salida [rad/s por V]."""
        return self.Ke / (self.Ke ** 2 + self.R * self.B) / N

    def tau(self):
        return self.J() * self.R / (self.Ke ** 2 + self.R * self.B)


class Traj:
    """Trapecio igual al de src/traj.c para un eje."""

    def __init__(self, q0, q1, vmax=math.radians(60), amax=math.radians(120)):
        self.q0, self.dq = q0, q1 - q0
        a = abs(self.dq)
        self.T = 0.0
        if a < 1e-6:
            return
        self.vs, self.as_ = vmax / a, amax / a
        self.ta = self.vs / self.as_
        if self.vs * self.ta > 1.0:
            self.ta = math.sqrt(1.0 / self.as_)
            self.vs = self.as_ * self.ta
            self.T = 2 * self.ta
        else:
            self.T = 1.0 / self.vs + self.ta

    def __call__(self, t):
        if self.T == 0.0 or t >= self.T:
            return self.q0 + self.dq
        if t < 0:
            return self.q0
        if t < self.ta:
            s = 0.5 * self.as_ * t * t
        elif t < self.T - self.ta:
            s = self.vs * (t - 0.5 * self.ta)
        else:
            s = 1.0 - 0.5 * self.as_ * (self.T - t) ** 2
        return self.q0 + self.dq * s


class PidController:
    """src/pid.c, para comparar con el controlador anterior."""

    def __init__(self, kp, ki, kd, kff, fric=FRIC, i_max=250.0):
        self.kp, self.ki, self.kd, self.kff, self.fric, self.i_max = kp, ki, kd, kff, fric, i_max
        self.i = 0.0

    def step(self, e, qd_ref, qd, dt, umax):
        base = self.kff * qd_ref + self.kp * e + self.kd * (qd_ref - qd)
        u = base + self.i
        if not (abs(u) >= umax and (u > 0) == (e > 0)):
            self.i = max(-self.i_max, min(self.i_max, self.i + self.ki * e * dt))
            u = base + self.i
        if abs(e) > E_ZONE and u != 0.0:
            u += math.copysign(self.fric, u)
        return max(-umax, min(umax, u))


def run(motor, gains, form, ref, tend, y0=0.0, disturb=None, ff=True):
    """Simula el firmware de un eje. gains = (kp, ki, kd, s1, kff); ref(t) en rad; disturb(t) en N·m en el eje de salida; ff=False quita la prealimentación de velocidad.
    Devuelve listas (t, r, y, u) a 1 kHz."""
    is_pid = form == "pid"
    if is_pid:
        ctl = PidController(*gains)
    else:
        b, c = rd.FORMS[form]
        ctl = rd.RstController(*gains[:4], b, c, RST_TICKS / 1000.0, y0=y0, kff=gains[4] if ff else 0.0)
    y_prev, qd, r_prev = y0, 0.0, ref(0.0)
    i = w = 0.0
    th = y0 * N
    J = motor.J()
    duty = u_hold = 0.0
    out = ([], [], [], [])
    for k in range(int(tend * 1000)):
        t = k / 1000.0
        r = ref(t)
        y = math.floor(th / N / (2 * math.pi) * CPR) * 2 * math.pi / CPR    # encoder cuantizado
        if is_pid:
            qd += 0.001 / (0.010 + 0.001) * ((y - y_prev) / 0.001 - qd)
            u_hold = ctl.step(r - y, ((r - r_prev) / 0.001) if ff else 0.0, qd, 0.001, DUTY_MAX)
            y_prev, r_prev = y, r
        elif k % RST_TICKS == 0:
            u = ctl.step(r, y, DUTY_MAX)
            if abs(r - y) > E_ZONE and u != 0.0:                            # como en pid.c y control.c
                u += math.copysign(FRIC, u)
            u_hold = max(-DUTY_MAX, min(DUTY_MAX, u))
        d = max(-SLEW_MS, min(SLEW_MS, u_hold - duty))                      # rampa de PWM en el lazo de 1 kHz
        duty += d
        V = duty / 1023.0 * 12.0
        Tl_ext = (disturb(t) / N) if disturb else 0.0
        for _ in range(TICKS_MS):
            i += (V - motor.R * i - motor.Ke * w) / motor.L * DT_PLANT
            Tc = motor.Tc * (1.0 if w > 0.5 else (-1.0 if w < -0.5 else w / 0.5))
            w += (motor.Ke * i - motor.B * w - Tc - Tl_ext) / J * DT_PLANT
            th += w * DT_PLANT
        out[0].append(t); out[1].append(r); out[2].append(y); out[3].append(duty)
    return out


def metrics(out, T_mov, final, tol=math.radians(0.4)):
    """Medidas de una respuesta: errores en grados, tiempos en s, esfuerzo en PWM."""
    t, r, y, u = out
    err = [abs(a - b) for a, b in zip(r, y)]
    last_out = 0.0
    for tk, yk in zip(t, y):
        if tk >= T_mov and abs(yk - final) > tol:
            last_out = tk - T_mov
    sign = 1.0 if final >= r[0] else -1.0
    over = max(0.0, max(sign * (yk - final) for yk in y))
    return {"e_seg": math.degrees(max(err)), "sobre": math.degrees(over), "t_asent": last_out,
            "u_max": max(abs(v) for v in u), "e_final": math.degrees(abs(y[-1] - final))}
