#!/usr/bin/env python3
"""
Diseño de los controladores RST del SCARA por asignación de polos.

Ley de control (convención de Landau):   S(q^-1) u(k) = T(q^-1) r(k) - R(q^-1) y(k)
  u = PWM [-1023, 1023], r e y = posición articular [rad].

Planta de posición (ZOH):   q/V = K / (s (tau s + 1)),   V = u * 12 / 1023
    A(q^-1) = (1 - q^-1)(1 - a q^-1),   a = exp(-Ts/tau)
    B(q^-1) = b1 q^-1 + b2 q^-2

Estructura PID con filtro derivativo:
    S = (1 - q^-1)(1 + s1 q^-1)
    R = Kp (1 - q^-1)(1 + s1 q^-1) + Ki Ts (1 + s1 q^-1) + (Kd/Ts)(1 - q^-1)^2

La ecuación diofántica A S + B R = P fija R y S (los polos). Las tres variantes comparten R y S, así
que tienen los mismos polos y el mismo rechazo de perturbaciones; solo cambia T, es decir, cómo entra
la consigna (ceros de lazo cerrado), con los pesos b (proporcional) y c (derivativo):
    T = b Kp (1 - q^-1)(1 + s1 q^-1) + Ki Ts (1 + s1 q^-1) + c (Kd/Ts)(1 - q^-1)^2
        + (Kff/Ts) S (1 - q^-1)

El último término es la prealimentación de velocidad, dentro de T: a la salida suma u = Kff (r(k) - r(k-1))/Ts.
No toca los polos y, como S(1) = 0, tampoco la ganancia estática. Sin ella toda la velocidad de crucero la
sostendría el integrador, y al frenar en seco el brazo se pasaría mientras se descarga.
"""
import numpy as np

PWM_PER_VOLT = 1023.0 / 12.0

# (b, c): pesos de la consigna en P y en D.
# "pi_d" sale de la asignación de polos, con la consigna fuera del término derivativo.
# "mask" es el RST enmascarado: se ingresan valores de un PID y corre un RST con los polinomios equivalentes (ver pid_as_rst).
# Esta definición de "enmascarado" la escogió el equipo; falta confirmarla con el profesor.
FORMS = {"pos": (1.0, 1.0), "inc": (0.0, 0.0), "pi_d": (1.0, 0.0), "mask": (1.0, 0.0)}
FORM_NAMES = {"pos": "PID posicional", "inc": "PID incremental (I-PD)", "pi_d": "PI-D por polos",
              "mask": "RST enmascarado (PID en forma RST)"}


def plant(K, tau, Ts):
    """K [rad/s por V], tau [s], Ts [s] -> (A, B) en potencias crecientes de q^-1; B en rad por cuenta de PWM."""
    a = np.exp(-Ts / tau)
    k = K / PWM_PER_VOLT
    b1 = k * (Ts - tau * (1.0 - a))
    b2 = k * (tau * (1.0 - a) - Ts * a)
    return np.array([1.0, -(1.0 + a), a]), np.array([0.0, b1, b2])


def closed_loop(A, B, R, S):
    """Polinomio característico P = A S + B R."""
    n = max(len(A) + len(S), len(B) + len(R)) - 1
    P = np.zeros(n)
    AS = np.convolve(A, S)
    BR = np.convolve(B, R)
    P[:len(AS)] += AS
    P[:len(BR)] += BR
    return np.trim_zeros(P, "b")


def poles_from_spec(wn, zeta, aux, Ts):
    """Par dominante continuo (wn, zeta) y polos reales auxiliares [rad/s] -> polos en z."""
    s = complex(-zeta * wn, wn * np.sqrt(max(0.0, 1.0 - zeta * zeta)))
    return [np.exp(s * Ts), np.exp(s.conjugate() * Ts)] + [np.exp(-p * Ts) for p in aux]


def diophantine(A, B, P):
    """Resuelve A (1 - q^-1)(1 + s1 q^-1) + B (r0 + r1 q^-1 + r2 q^-2) = P con P mónico de orden 4."""
    A1 = np.convolve(A, [1.0, -1.0])                  # [1, a1, a2, a3]
    b1, b2 = B[1], B[2]
    # Incógnitas x = [s1, r0, r1, r2]; ecuaciones de q^-1 a q^-4.
    M = np.array([[1.0, b1, 0.0, 0.0],
                  [A1[1], b2, b1, 0.0],
                  [A1[2], 0.0, b2, b1],
                  [A1[3], 0.0, 0.0, b2]])
    rhs = np.array([P[1] - A1[1], P[2] - A1[2], P[3] - A1[3], P[4]])
    s1, r0, r1, r2 = np.linalg.solve(M, rhs)
    return np.array([r0, r1, r2]), np.convolve([1.0, -1.0], [1.0, s1])


def pid_from_rs(R, S, Ts):
    """(R, S) -> (kp, ki, kd, s1) de la forma PID con filtro derivativo."""
    s1 = S[1] + 1.0
    M = np.array([[1.0, 1.0, 1.0], [s1 - 1.0, s1, -2.0], [-s1, 0.0, 1.0]])
    kp, ki_ts, kd_ts = np.linalg.solve(M, R)
    return kp, ki_ts / Ts, kd_ts * Ts, s1


def polys(kp, ki, kd, s1, b, c, Ts, kff=0.0):
    """Ganancias PID + pesos de consigna -> (R, S, T). Es la misma cuenta que hace rst_set_pid en src/rst.c."""
    p = np.array([1.0, s1 - 1.0, -s1])                # (1 - q^-1)(1 + s1 q^-1)
    i = np.array([1.0, s1, 0.0])                      # (1 + s1 q^-1)
    d = np.array([1.0, -2.0, 1.0])                    # (1 - q^-1)^2
    R = kp * p + ki * Ts * i + kd / Ts * d
    T = np.zeros(4)
    T[:3] = b * kp * p + ki * Ts * i + c * kd / Ts * d
    T += kff / Ts * np.convolve(p, [1.0, -1.0])
    return R, p, T


def pid_as_rst(kp, ki, kd, kff, tau_vel, Ts):
    """El PID de src/pid.c como RST exacto -> (kp, ki, kd, s1, kff) para usar con los pesos de FORMS["mask"].

    pid.c deriva la medición con un filtro de primer orden tau_vel y toma la velocidad de la consigna sin filtrar:
        u = Kff vr + Kp e + Kd (vr - v_filtrada) + I
    Con alfa = tau_vel / (tau_vel + Ts), el filtro es (1 - alfa)/(1 - alfa q^-1), de donde s1 = -alfa y la Kd del RST es
    Kd (1 - alfa). La velocidad sin filtrar de la consigna no cabe en el término D de T, así que Kd se suma a Kff y c = 0.
    """
    alfa = tau_vel / (tau_vel + Ts)
    return kp, ki, kd * (1.0 - alfa), -alfa, kff + kd


def design_joint(K, tau, Ts, wn, zeta, aux):
    """Diseño completo de un eje. Devuelve un diccionario con planta, polos, R, S y ganancias PID."""
    A, B = plant(K, tau, Ts)
    poles = poles_from_spec(wn, zeta, aux, Ts)
    P = np.real(np.poly(poles))
    R, S = diophantine(A, B, P)
    kp, ki, kd, s1 = pid_from_rs(R, S, Ts)
    return {"A": A, "B": B, "P": P, "poles": poles, "R": R, "S": S, "s1": s1, "kp": kp, "ki": ki, "kd": kd,
            "K": K, "tau": tau, "Ts": Ts, "wn": wn, "zeta": zeta, "aux": tuple(aux)}


class RstController:
    """Referencia en Python de src/rst.c: misma ecuación, misma saturación y misma historia."""

    def __init__(self, kp, ki, kd, s1, b, c, Ts, y0=0.0, kff=0.0):
        self.R, self.S, self.T = polys(kp, ki, kd, s1, b, c, Ts, kff)
        self.reset(y0)

    def reset(self, y0=0.0):
        self.y = [y0, y0]
        self.r = [y0, y0, y0]
        self.u = [0.0, 0.0]

    def step(self, r, y, umax):
        """Devuelve u. La historia guarda u saturada: el integrador de S no se enrolla."""
        v = (self.T[0] * r + self.T[1] * self.r[0] + self.T[2] * self.r[1] + self.T[3] * self.r[2]
             - self.R[0] * y - self.R[1] * self.y[0] - self.R[2] * self.y[1])
        u = v - self.S[1] * self.u[0] - self.S[2] * self.u[1]
        u = min(umax, max(-umax, u))
        self.y = [y, self.y[0]]
        self.r = [r, self.r[0], self.r[1]]
        self.u = [u, self.u[0]]
        return u
