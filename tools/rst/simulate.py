#!/usr/bin/env python3
"""
Diseña los tres RST del SCARA, simula su respuesta de posición y elige el mejor con un criterio escrito.

    python3 tools/rst/simulate.py

Genera docs/rst/REPORTE_RST.md, docs/rst/*.png y src/rst_defaults.h (lo que usa el firmware).
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plantmodel as pm  # noqa: E402
import rst_design as rd  # noqa: E402

ROOT = os.path.join(HERE, "..", "..")
DOCS = os.path.join(ROOT, "docs", "rst")
TS = 0.001                                      # el RST corre en el lazo de 1 kHz, igual que el PID original
VEL_TAU = 0.010                                 # filtro de velocidad del firmware (VEL_FILTRO_TAU_S)
K_DESIGN = 0.9974                               # identificado en el laboratorio de Control Digital [rad/s por V]
KFF = rd.PWM_PER_VOLT / K_DESIGN                # prealimentación de velocidad = inversa de K [PWM por rad/s]
AXES = {"J1": {"tau": 0.14, "J_load": 0.04}, "J2": {"tau": 0.10, "J_load": 0.01}}
# Polos de cada forma: (wn [rad/s], zeta, polos auxiliares [rad/s]).
SPECS = {"pos": (12.0, 0.9, (8.0, 60.0)), "mask": (12.0, 0.9, (8.0, 60.0)), "inc": (15.0, 0.9, (40.0, 60.0))}
PID_OLD = (5000.0, 3000.0, 180.0, 80.0)         # el PID original del firmware: su PWM es el que funcionaba en el robot
ORIG = rd.pid_as_rst(*PID_OLD, VEL_TAU, TS)
FOLLOW_ERR_DEG = math.degrees(0.20)             # sobre esto el firmware declara falla
HOLD_TOL_DEG = 0.4
AMP = math.radians(45.0)
STEP = math.radians(10.0)
DIST_NM = 0.8
SERIES = {"pos": "#2a78d6", "inc": "#eb6834", "mask": "#1baf7a", "orig": "#8a3ffc", "pid": "#eda100"}
LABEL = dict(rd.FORM_NAMES, pid="PID original (src/pid.c)")
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
ROBUST = [(jl, tc) for jl in (0.25, 1.0, 2.5) for tc in (0.5, 1.0, 1.5)]   # inercia del eslabón y fricción seca


def design_orig(ax):
    """El RST del PID original: no se piden polos, se calculan los que resultan con el modelo de diseño."""
    kp, ki, kd, s1, kff = ORIG
    A, B = rd.plant(K_DESIGN, AXES[ax]["tau"], TS)
    R, S, _ = rd.polys(kp, ki, kd, s1, 1.0, 1.0, TS)
    return {"A": A, "B": B, "R": R, "S": S, "s1": s1, "kp": kp, "ki": ki, "kd": kd, "kff": kff, "K": K_DESIGN,
            "tau": AXES[ax]["tau"], "Ts": TS, "poles": list(np.roots(rd.closed_loop(A, B, R, S))), "wn": None}


def design_all():
    out = {}
    for ax in AXES:
        out[ax] = {f: dict(rd.design_joint(K_DESIGN, AXES[ax]["tau"], TS, *SPECS[f]), kff=KFF) for f in SPECS}
        out[ax]["orig"] = design_orig(ax)
    return out


def gains(d):
    return (d["kp"], d["ki"], d["kd"], d["s1"], d["kff"])


def pwm_diff(out, ref):
    """Diferencia de PWM frente al PID original: (rms, máxima)."""
    d = np.array(out[3]) - np.array(ref[3])
    return float(np.sqrt(np.mean(d ** 2))), float(np.max(np.abs(d)))


def pwm_metrics(res):
    return {ax: {f: pwm_diff(res[ax][f]["traj"], res[ax]["pid"]["traj"]) for f in rd.FORMS} for ax in AXES}


def responses(ax, f, g):
    m = pm.Motor(AXES[ax]["J_load"])
    tr = pm.Traj(0.0, AMP)
    res = {"traj": pm.run(m, g, f, tr, tr.T + 1.5), "T": tr.T}
    res["step"] = pm.run(m, g, f, lambda t: STEP if t >= 0.1 else 0.0, 2.5, ff=False)
    res["dist"] = pm.run(m, g, f, lambda t: 0.0, 2.5, disturb=lambda t: DIST_NM if t >= 0.5 else 0.0)
    return res


def step_metrics(out):
    t, _, y, u = out
    y = np.degrees(y)
    final = math.degrees(STEP)
    outside = [tk for tk, yk in zip(t, y) if abs(yk - final) > HOLD_TOL_DEG and tk >= 0.1]
    return {"sobre": max(0.0, max(y) - final), "t_asent": (outside[-1] - 0.1) if outside else 0.0,
            "u_max": max(abs(v) for v in u), "e_final": abs(y[-1] - final)}


def dist_metrics(out):
    t, _, y, _ = out
    y = np.abs(np.degrees(y))
    outside = [tk for tk, yk in zip(t, y) if yk > HOLD_TOL_DEG]
    return {"pico": max(y), "t_rec": (outside[-1] - 0.5) if outside else 0.0, "e_final": y[-1]}


def robust_worst(ax, f, g):
    """Peor caso del movimiento de 45 grados sobre la rejilla de inercia y fricción."""
    w = {"e_seg": 0.0, "sobre": 0.0, "t_asent": 0.0, "e_final": 0.0, "u_max": 0.0}
    for jl, tc in ROBUST:
        m = pm.Motor(AXES[ax]["J_load"] * jl, Tc=3e-3 * tc)
        tr = pm.Traj(0.0, AMP)
        mt = pm.metrics(pm.run(m, g, f, tr, tr.T + 1.5), tr.T, AMP)
        for k in w:
            w[k] = max(w[k], mt[k])
    return w


def linear_margin(d):
    """Mayor |polo| de lazo cerrado con K +-20 % y tau +-50 % respecto al modelo de diseño."""
    worst = 0.0
    for kf in (0.8, 1.0, 1.2):
        for tf in (0.5, 1.0, 1.5):
            A, B = rd.plant(d["K"] * kf, d["tau"] * tf, TS)
            worst = max(worst, max(abs(np.roots(rd.closed_loop(A, B, d["R"], d["S"])))))
    return worst


def plot_small_multiples(fname, title, key, res, ylabel, tmax, ref=True, xform=None):
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True, constrained_layout=True)
    for axn, ax in zip(res, axs):
        first = next(iter(res[axn].values()))
        t = np.array(first[key][0])
        if ref:
            ax.plot(t, np.degrees(first[key][1]), color=MUTED, lw=1.2, ls="--", label="Consigna")
        for f, r in res[axn].items():
            y = np.degrees(r[key][2]) if xform is None else xform(r[key])
            ax.plot(t, y, color=SERIES[f], lw=2, label=LABEL[f])
        ax.set_title(axn, loc="left", color=INK, fontsize=11)
        ax.set_xlim(0, tmax)
        ax.set_xlabel("Tiempo [s]", color=MUTED)
        ax.grid(color=GRID, lw=0.8)
        ax.tick_params(colors=MUTED, length=0)
        for sp in ax.spines.values():
            sp.set_visible(False)
    axs[0].set_ylabel(ylabel, color=MUTED)
    axs[1].legend(frameon=False, fontsize=9, loc="best")
    fig.suptitle(title, x=0.01, ha="left", color=INK, fontsize=12)
    fig.savefig(os.path.join(DOCS, fname), dpi=140, facecolor="#fcfcfb")
    plt.close(fig)


def pick_winner(robust, pwm):
    """Criterio: sin falla ni error en reposo en la rejilla; entre esas, el PWM más parecido al del PID original."""
    ok = {f: all(robust[ax][f]["e_seg"] < 0.8 * FOLLOW_ERR_DEG and robust[ax][f]["e_final"] < HOLD_TOL_DEG
                 for ax in AXES) for f in rd.FORMS}
    cand = [f for f in rd.FORMS if ok[f]] or list(rd.FORMS)
    return min(cand, key=lambda f: max(pwm[ax][f][0] for ax in AXES)), ok


def write_header(des, winner):
    names = {"pos": "RST_FORM_POS", "inc": "RST_FORM_INC", "mask": "RST_FORM_MASK", "orig": "RST_FORM_ORIG"}
    with open(os.path.join(ROOT, "src", "rst_defaults.h"), "w") as f:
        f.write("/* Generado por tools/rst/simulate.py: no editar a mano. Ver docs/rst/REPORTE_RST.md. */\n#pragma once\n")
        f.write("/* Forma con mejor resultado en la simulación */\n#define RST_FORM_DEFAULT %s\n\n" % names[winner])
        f.write("/* [forma: pos, inc, mask, orig][eje] = { kp, ki, kd, s1, kff }   (PWM/rad, Ts = 1 ms) */\n")
        f.write("static const float RST_GAINS[4][2][5] = {\n")
        for fm in ("pos", "inc", "mask", "orig"):
            f.write("    { ")
            f.write(", ".join("{ %.3ff, %.3ff, %.5ff, %.7ff, %.3ff }" % gains(des[ax][fm]) for ax in AXES))
            f.write(" },\n")
        f.write("};\n")


def table(rows):
    head, *body = rows
    out = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    out += ["| " + " | ".join(r) + " |" for r in body]
    return "\n".join(out)


def poly(c):
    return "[" + ", ".join("%.4f" % v for v in c) + "]"


def write_report(des, res, robust, margins, winner, ok, pwm):
    A = {ax: rd.plant(K_DESIGN, AXES[ax]["tau"], TS) for ax in AXES}
    L = []
    L.append("""# Controladores RST del SCARA

Generado por `python3 tools/rst/simulate.py`. Todos los números salen de ese programa.

**Forma elegida para el robot: %s.** Todo es simulación: falta medir en el robot la inercia de los eslabones y repetir estas pruebas.

## 1. Planta

Cada articulación se modela como motor con reductor, de PWM a posición:

```
q(s)/V(s) = K / (s (tau s + 1)),   V = u * 12 / 1023
```

| Dato | Valor | Origen |
|---|---|---|
| K | %.4f rad/s por V | Identificado en el laboratorio de Control Digital, sin eslabón |
| tau J1 | %.2f s | Estimado: motor + eslabones. **Sin medir** |
| tau J2 | %.2f s | Estimado: motor + eslabón 2. **Sin medir** |
| Ts | 1 ms | El mismo lazo del PID original. Es lo que permite reproducir su PWM (sección 6) |

La planta de las simulaciones usa otro juego de datos a propósito: el motor de la hoja de datos (K = %.3f rad/s por V), con fricción seca, encoder cuantizado, saturación y rampa de PWM. El controlador no se prueba contra el mismo modelo con el que se diseñó.

Discretización ZOH:

```
A(q^-1) = (1 - q^-1)(1 - a q^-1),  a = exp(-Ts/tau)
B(q^-1) = b1 q^-1 + b2 q^-2
```

| Eje | A | B |
|---|---|---|
""" % (LABEL[winner], K_DESIGN, AXES["J1"]["tau"], AXES["J2"]["tau"], pm.Motor().K()))
    for ax in AXES:
        L.append("| %s | `%s` | `[0, %.4e, %.4e]` |\n" % (ax, poly(A[ax][0]), A[ax][1][1], A[ax][1][2]))
    L.append(("""
## 2. Estructura RST

```
S(q^-1) u(k) = T(q^-1) r(k) - R(q^-1) y(k)
S = (1 - q^-1)(1 + s1 q^-1)
R = Kp (1 - q^-1)(1 + s1 q^-1) + Ki Ts (1 + s1 q^-1) + (Kd/Ts)(1 - q^-1)^2
T = b Kp (1 - q^-1)(1 + s1 q^-1) + Ki Ts (1 + s1 q^-1) + c (Kd/Ts)(1 - q^-1)^2 + (Kff/Ts) S (1 - q^-1)
```

El último término de T es la prealimentación de velocidad (Kff = 1/K en PWM, %.1f PWM por rad/s). Sin ella, toda la velocidad de crucero la sostiene el integrador, y al frenar en seco el brazo se pasa mientras se descarga. Como S(1) = 0, no cambia la ganancia estática; como está en T, tampoco los polos.

R y S salen de la ecuación diofántica `A S + B R = P`, con P dado por los polos deseados. T no mueve los polos: decide cómo entra la consigna, con los pesos b y c. En los tres casos `T(1) = R(1)`, así que la ganancia estática es 1.

| Forma | b | c | Ley equivalente |
|---|---|---|---|
| PID posicional | 1 | 1 | P, I y D actúan sobre el error |
| PID incremental (I-PD) | 0 | 0 | Solo I actúa sobre el error; P y D sobre la medición |
| PID enmascarado (PI-D) | 1 | 0 | La consigna queda fuera del término D |

> **Por confirmar con el profesor:** "masked PID" no aparece en los apuntes del curso ni en la búsqueda web. PI-D es una interpretación propia. Si la definición es otra, se cambia `FORMS` en `rst_design.py` y se vuelve a generar este reporte.

**Convención:** este reporte usa la de Landau. Algunos libros intercambian los nombres de R y S.

## 3. Diseños

""") % KFF)
    rows = [["Eje", "Forma", "Polos: wn, zeta, auxiliares", "Kp", "Ki", "Kd", "s1", "Kff", "max |polo| con K ±20 %, tau ±50 %"]]
    for ax in AXES:
        for f in rd.FORMS:
            d = des[ax][f]
            spec = ("%.0f rad/s, %.1f, %s rad/s" % (d["wn"], d["zeta"], d["aux"])) if d["wn"] else "Resultan del PID original"
            rows.append([ax, LABEL[f], spec, "%.0f" % d["kp"], "%.0f" % d["ki"],
                         "%.2f" % d["kd"], "%.4f" % d["s1"], "%.1f" % d["kff"], "%.5f" % margins[ax][f]])
    L.append(table(rows))
    L.append("\n\nLas tres primeras formas salen de la asignación de polos. La incremental lleva polos más rápidos a propósito: su error en rampa es Kp/Ki por la velocidad, y con los polos de las otras dos pasaría el límite de falla del firmware en un movimiento normal.\n\nLa cuarta no se diseñó con polos: es `src/pid.c` escrito como RST, con `s1 = -tau_v/(tau_v + Ts)`, `Kd_RST = Kd (1 + s1)` y `Kff_RST = Kff + Kd`, donde tau_v = 10 ms es el filtro de velocidad del firmware. Sus polos nominales de J1 son %s.\n\nTodos los valores de la última columna deben ser menores que 1 para que el lazo siga estable.\n\nCoeficientes (potencias crecientes de q^-1):\n\n" % ", ".join("%.5f" % abs(p) for p in sorted(des["J1"]["orig"]["poles"], key=abs)))
    rows = [["Eje", "Forma", "R", "S", "T"]]
    for ax in AXES:
        for f, (b, c) in rd.FORMS.items():
            d = des[ax][f]
            R, Sp, T = rd.polys(d["kp"], d["ki"], d["kd"], d["s1"], b, c, TS, KFF)
            rows.append([ax, LABEL[f], "`%s`" % poly(R), "`%s`" % poly(Sp), "`%s`" % poly(T)])
    L.append(table(rows))
    L.append("""

Ecuación que ejecuta el firmware (`src/rst.c`), cada 1 ms:

```
u(k) = t0 r(k) + t1 r(k-1) + t2 r(k-2) + t3 r(k-3) - r0 y(k) - r1 y(k-1) - r2 y(k-2) + (1 - s1) u(k-1) + s1 u(k-2)
```

La historia guarda el PWM ya saturado: así el integrador no se enrolla. La rampa de PWM se aplica afuera; dentro de la historia borraría el término proporcional. La compensación de fricción seca (70 PWM) se suma afuera con la misma regla de `pid.c`: sentido de u y solo si el error supera 0,15 grados.

## 4. Respuesta de posición

### Escalón de %.0f grados

Sin prealimentación: es la respuesta del RST por sí solo, donde más se nota la diferencia entre formas. En el robot las consignas nunca son escalones, siempre pasan por la trayectoria.

![Escalón](step.png)

""" % math.degrees(STEP))
    L.append("El PID anterior no está pensado para escalones: el firmware siempre le da una trayectoria. Con el escalón, la rampa de PWM desestabiliza su lazo, sobre todo en J1. Ese resultado no aplica al uso normal.\n\n")
    rows = [["Eje", "Controlador", "Sobreimpulso [grados]", "Asentamiento ±%.1f grados [s]" % HOLD_TOL_DEG, "PWM máximo", "Error final [grados]"]]
    for ax in AXES:
        for f in res[ax]:
            m = step_metrics(res[ax][f]["step"])
            rows.append([ax, LABEL[f], "%.2f" % m["sobre"], "%.2f" % m["t_asent"], "%.0f" % m["u_max"], "%.3f" % m["e_final"]])
    L.append(table(rows))
    L.append("\n\n### Trayectoria trapezoidal de 45 grados (el caso real del robot)\n\n![Trayectoria](traj.png)\n\n")
    rows = [["Eje", "Controlador", "Error de seguimiento máx. [grados]", "Sobreimpulso [grados]", "Asentamiento [s]", "PWM máximo", "Error final [grados]"]]
    for ax in AXES:
        for f in res[ax]:
            m = pm.metrics(res[ax][f]["traj"], res[ax][f]["T"], AMP)
            rows.append([ax, LABEL[f], "%.2f" % m["e_seg"], "%.2f" % m["sobre"], "%.2f" % m["t_asent"], "%.0f" % m["u_max"], "%.3f" % m["e_final"]])
    L.append(table(rows))
    L.append("\n\n![Error de seguimiento](error.png)\n\n### Perturbación de %.1f N·m en el eje de salida\n\n![Perturbación](dist.png)\n\n" % DIST_NM)
    rows = [["Eje", "Controlador", "Desviación máxima [grados]", "Recuperación ±%.1f grados [s]" % HOLD_TOL_DEG]]
    for ax in AXES:
        for f in res[ax]:
            m = dist_metrics(res[ax][f]["dist"])
            rows.append([ax, LABEL[f], "%.2f" % m["pico"], "%.2f" % m["t_rec"]])
    L.append(table(rows))
    L.append("\n\n## 5. Robustez\n\nPeor caso del movimiento de 45 grados con la inercia del eslabón ×0,25, ×1 y ×2,5, y la fricción seca ×0,5, ×1 y ×1,5 (9 casos por eje).\n\n")
    rows = [["Eje", "Forma", "Error de seguimiento [grados]", "Sobreimpulso [grados]", "Asentamiento [s]", "Error final [grados]"]]
    for ax in AXES:
        for f in rd.FORMS:
            w = robust[ax][f]
            rows.append([ax, LABEL[f], "%.2f" % w["e_seg"], "%.2f" % w["sobre"], "%.2f" % w["t_asent"], "%.3f" % w["e_final"]])
    L.append(table(rows))
    L.append("""

## 6. Decisión

Criterio, en este orden:

1. En todos los casos de robustez, el error de seguimiento queda por debajo del 80 %% del límite de falla del firmware (%.1f grados) y el error final dentro de ±%.1f grados.
2. Entre las que cumplen, gana la de PWM más parecido al del PID original, que es el que funcionó en el robot.

""" % (0.8 * FOLLOW_ERR_DEG, HOLD_TOL_DEG))
    rows = [["Forma", "Cumple el criterio 1", "Peor error de seguimiento [grados]", "Diferencia de PWM frente al original, peor eje: rms / máxima"]]
    for f in rd.FORMS:
        rows.append([LABEL[f], "Sí" if ok[f] else "No", "%.2f" % max(robust[ax][f]["e_seg"] for ax in AXES),
                     "%.1f / %.1f" % (max(pwm[ax][f][0] for ax in AXES), max(pwm[ax][f][1] for ax in AXES))])
    L.append(table(rows))
    L.append("""

**Resultado: %s.** Es el valor por defecto en el firmware. Las otras formas se eligen con `rstform`, y `pid.c` sigue disponible con `ctrl pid`. El segundo criterio lo pidió el dueño del robot: el PWM del firmware original funcionaba bien. Sin él, ganaría la forma de menor error de seguimiento en la tabla.

## 7. Límites de este resultado

- La inercia de los eslabones y por tanto tau son estimaciones. Medirlos en el robot y volver a correr este programa con los valores reales.
- El modelo de fricción es suave cerca de velocidad cero: no reproduce la adherencia real del reductor, que puede causar una pequeña oscilación en reposo si Ki es alto.
- Los ejes se simulan desacoplados; con reductor 100:1 el acoplamiento debe ser pequeño, pero no se midió.
- Nada de esto se ha ejecutado aún en el robot.
""" % LABEL[winner])
    L.append("- La igualdad de PWM con `pid.c` deja de valer si el PWM satura o el integrador pasa de 250 PWM: `pid.c` lo limita a ese valor y el RST no tiene ese límite aparte.\n")
    with open(os.path.join(DOCS, "REPORTE_RST.md"), "w") as f:
        f.write("".join(L))


def main():
    os.makedirs(DOCS, exist_ok=True)
    des = design_all()
    res, robust, margins = {}, {}, {}
    for ax in AXES:
        res[ax] = {f: responses(ax, f, gains(des[ax][f])) for f in rd.FORMS}
        res[ax]["pid"] = responses(ax, "pid", PID_OLD)
        robust[ax] = {f: robust_worst(ax, f, gains(des[ax][f])) for f in rd.FORMS}
        margins[ax] = {f: linear_margin(des[ax][f]) for f in rd.FORMS}
    pwm = pwm_metrics(res)
    winner, ok = pick_winner(robust, pwm)
    plot_small_multiples("step.png", "Escalón de 10 grados", "step", res, "Posición [grados]", 1.2)
    plot_small_multiples("traj.png", "Trayectoria trapezoidal de 45 grados", "traj", res, "Posición [grados]", 2.6)
    plot_small_multiples("error.png", "Error de seguimiento en la trayectoria de 45 grados", "traj", res, "Error [grados]", 2.6, ref=False,
                         xform=lambda o: np.degrees(np.array(o[1]) - np.array(o[2])))
    plot_small_multiples("dist.png", "Perturbación de 0,8 N·m a los 0,5 s", "dist", res, "Desviación [grados]", 2.5, ref=False)
    write_header(des, winner)
    write_report(des, res, robust, margins, winner, ok, pwm)
    print("Forma elegida:", winner, ok)
    for ax in AXES:
        for f in rd.FORMS:
            print(ax, f, "e_seg peor %.2f  margen %.5f  dPWM rms %.2f max %.1f" % (robust[ax][f]["e_seg"], margins[ax][f], pwm[ax][f][0], pwm[ax][f][1]))


if __name__ == "__main__":
    main()
