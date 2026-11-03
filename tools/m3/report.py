#!/usr/bin/env python3
"""
Reporte del motor 3 y del atornillado: corre el firmware real (tools/sim) y genera docs/m3/REPORTE_M3.md y sus gráficas.

    python3 tools/m3/report.py
"""
import ctypes
import math
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2"))
import scara_sequence as sq  # noqa: E402

DOCS = os.path.join(ROOT, "docs", "m3")
LIB = os.path.join(ROOT, "tools", "sim", "build", "libscara_sim.so")
NORMAL, ATASCO, BARRIDO, NO_CAE = 0, 1, 2, 3    # sim_tornillo_t
I, TORQUE, FUERZA, PRESION, VUELTAS, PROF, SERVO, FASE = range(16, 24)
BLUE, INK, MUTED, GRID = "#2a78d6", "#0b0b0b", "#52514e", "#e4e3df"
# Hoja de datos del Pololu 37D 50:1, 12 V
W0 = 200.0 * 2 * math.pi / 60.0
I0, I_CAL, T_CAL = 0.2, 5.5, 21.0 * 0.0980665
R = 12.0 / I_CAL
KE = (12.0 - R * I0) / W0
KT = T_CAL / (I_CAL - I0)


class Sim:
    def __init__(self):
        subprocess.run(["sh", os.path.join(ROOT, "tools", "sim", "build.sh")], check=True)
        self.lib = ctypes.CDLL(LIB)
        self.lib.sim_cmd.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        self.lib.sim_event_pop.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self.lib.sim_state.argtypes = [ctypes.POINTER(ctypes.c_float)]
        self.lib.sim_m3_factors.argtypes = [ctypes.c_double] * 3
        self.lib.sim_m3_real.argtypes = [ctypes.POINTER(ctypes.c_double)]
        self.buf = ctypes.create_string_buffer(240)
        self.st = (ctypes.c_float * 24)()
        self.real = (ctypes.c_double * 3)()

    def start(self, mode=NORMAL, factors=(1.0, 1.0, 1.0)):
        self.lib.sim_m3_factors(*factors)
        self.lib.sim_init()
        self.lib.sim_screw_mode(mode)
        self.events = []
        self.cmd("motor 3 on")
        self.run(20)

    def cmd(self, text):
        self.lib.sim_cmd(text.encode(), self.buf, len(self.buf))

    def run(self, ms):
        self.lib.sim_step_ms(ms)
        while self.lib.sim_event_pop(self.buf, len(self.buf)):
            self.events.append(self.buf.value.decode())

    def state(self):
        self.lib.sim_state(self.st)
        return list(self.st)

    def m3_real(self):
        self.lib.sim_m3_real(self.real)
        return list(self.real)

    def result(self):
        return next((sq.parse_tornillo(e) for e in reversed(self.events) if e.startswith("#TORNILLO")), None)


def drop(sim):
    sim.cmd("servo subir"); sim.run(400)
    sim.cmd("servo bajar"); sim.run(600)


def screw(sim, mode=NORMAL, factors=(1.0, 1.0, 1.0)):
    """Un tornillo completo. Devuelve (tiempo, serie medida con el modelo, serie real de la planta, resultado)."""
    sim.start(mode, factors)
    drop(sim)
    sim.cmd("screw")
    t, est, real = [], [], []
    for k in range(1300):             # hasta 26 s, cada 20 ms
        sim.run(20)
        s = sim.state()
        t.append((k + 1) * 0.02); est.append(s); real.append(sim.m3_real())
        if s[FASE] == 4 and abs(s[15]) < 0.5:
            break
    return np.array(t), np.array(est), np.array(real), sim.result()


def run_process(sim, seq):
    """Todo el proceso sin ROS, con el mismo programa de secuencia del panel. Devuelve (runner, errores al soltar [mm])."""
    sim.start()
    sim.cmd("home")
    for _ in range(4000):
        sim.run(20)
        s = sim.state()
        if s[6] and s[7] and s[4] == 2:
            break
    now = [0.0]
    r = sq.SequenceRunner(seq.steps, 1, sim.cmd, lambda: now[0])
    r.start()
    errors, last, seen = {}, None, 0
    while r.running and now[0] < 600:
        sim.run(10)
        now[0] += 0.01
        for e in sim.events[seen:]:
            r.on_event(e)
        seen = len(sim.events)
        r.tick()
        step = r.steps[r.index] if r.running else None
        if step is not last and step and step.get("text") == "servo subir":
            g = r.steps[r.index - 2]
            s = sim.state()
            errors[r.steps[r.index + 4]["label"]] = math.hypot(s[2] - g["x"], s[3] - g["y"])
        last = step
    return r, errors, now[0]


def table(rows):
    head, *body = rows
    return "\n".join(["| " + " | ".join(head) + " |", "|" + "---|" * len(head)] + ["| " + " | ".join(r) + " |" for r in body])


def style(ax, title, xlabel=True):
    ax.set_title(title, loc="left", color=INK, fontsize=11)
    if xlabel:
        ax.set_xlabel("Tiempo [s]", color=MUTED)
    ax.grid(color=GRID, lw=0.8)
    ax.tick_params(colors=MUTED, length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)


def plot_normal(t, est, real):
    fig, axs = plt.subplots(2, 2, figsize=(10, 5.6), sharex=True, constrained_layout=True)
    for ax, y, title, lab in ((axs[0, 0], est[:, I], "Corriente estimada [A]", "A"), (axs[0, 1], est[:, TORQUE], "Torque estimado [N·m]", "N·m"),
                              (axs[1, 0], est[:, FUERZA], "Fuerza en el tornillo [N]", "N"), (axs[1, 1], est[:, PROF], "Profundidad [mm]", "mm")):
        ax.plot(t, y, color=BLUE, lw=2)
        style(ax, title, xlabel=ax in axs[1])
    fig.suptitle("Un tornillo normal: rápido, lento y apriete", x=0.01, ha="left", color=INK, fontsize=12)
    fig.savefig(os.path.join(DOCS, "tornillo.png"), dpi=140, facecolor="#fcfcfb")
    plt.close(fig)


def plot_cases(cases):
    fig, axs = plt.subplots(1, 4, figsize=(11, 3.2), sharey=True, constrained_layout=True)
    for ax, (name, (t, est, _, res)) in zip(axs, cases.items()):
        ax.plot(t, est[:, I], color=BLUE, lw=2)
        style(ax, "%s: %s" % (name, res["resultado"]))
    axs[0].set_ylabel("Corriente estimada [A]", color=MUTED)
    fig.savefig(os.path.join(DOCS, "casos.png"), dpi=140, facecolor="#fcfcfb")
    plt.close(fig)


def write(cases, sens, seq, runner, errors, t_total):
    t_n, est_n, real_n, r_n = cases["Normal"]
    rap = (est_n[:, FASE] == 1) & (t_n > 1.0)
    err_i = np.abs(est_n[rap, I] - real_n[rap, 0])
    L = ["""# Motor 3 y atornillado

Generado por `python3 tools/m3/report.py`, con el firmware real contra la planta simulada. **Nada de esto se ha medido en el robot.**

## 1. Modelo del motor

Dato del usuario: M3 es de 200 rpm. Supongo un Pololu 37D 50:1 de 12 V, de la misma familia que J1 y J2, porque en esa familia 200 rpm corresponde a esa relación. **Hay que confirmarlo.**

| Dato | J1 y J2 | M3 |
|---|---|---|
| Reductor | 100:1 | 50:1 |
| Velocidad en vacío a 12 V | 100 rpm | 200 rpm |
| Corriente en vacío | 0,2 A | %.1f A |
| Corriente calado a 12 V | 5,5 A | %.1f A |
| Torque calado a 12 V | | %.2f N·m (21 kg·cm) |
| Cuentas por vuelta de salida | 6424 | 3200 |

La hoja de datos extrapola los valores de calado: no los mide. Referido al eje de salida:

```
R  = 12 / I_cal                 = %.3f ohm
Ke = (12 - R I0) / w0           = %.4f V/(rad/s)
Kt = T_cal / (I_cal - I0)       = %.4f N·m/A      (menor que Ke: pérdidas del reductor)
```

## 2. Predicción de la corriente

El ACS712 sale de J13 para dejar el sitio al servo, así que la corriente no se mide: se calcula a 1 kHz con el PWM aplicado y la velocidad del encoder de M3. Lo que el tornillo opone se nota porque frena el motor.

```
V      = PWM * 12 / 1023
i      = (V - Ke w) / R
torque = Kt (i - I0)
fuerza = (torque - T_rosca) / (K d)      K = 0,2; d = 3 mm; T_rosca = 0,03 N·m
presión = fuerza / A                     A = 15,7 mm², bajo una cabeza de 5,5 mm con hueco de 3,2 mm
profundidad = vueltas * paso             paso = 0,5 mm
```

K, el área, el paso y T_rosca son valores típicos de un tornillo M3 de acero, **no datos de tu tornillo**. K cambia con la lubricación y el material, así que la fuerza es la cifra menos segura. Este modelo no incluye la inductancia del motor.

**Límite de esta prueba:** la planta simulada usa las mismas ecuaciones que el modelo, sin inductancia. Que coincidan muestra que el firmware calcula bien, no que el modelo describa tu motor. Solo una medición en el robot puede confirmarlo.

## 3. Un tornillo

El PWM rápido es 650, el tope de PWM del firmware original. Al asentar la cabeza, el PWM baja al valor cuyo torque de calado es la meta: así el motor no puede pasarse aunque siga empujando.

Cómo leer las gráficas:

- **Primeros 0,4 s:** la corriente sube porque el motor acelera. Por eso torque, fuerza y presión se muestran en 0 durante ese tiempo.
- **Hacia los 5 s:** el PWM baja de 650 a 300 y la corriente estimada se vuelve negativa, porque el motor aún gira más rápido de lo que pide el nuevo voltaje. No es un error del tornillo.

![Tornillo normal](tornillo.png)

""" % (I0, I_CAL, T_CAL, R, KE, KT)]
    L.append(table([["Resultado", "Vueltas", "Profundidad", "Avance", "Torque", "Fuerza", "Presión", "Corriente", "Tiempo"],
                    [r_n["resultado"], "%.2f" % r_n["vueltas"], "%.2f mm" % r_n["prof"], "%.0f %%" % r_n["avance"], "%.3f N·m" % r_n["torque"],
                     "%.0f N" % r_n["fuerza"], "%.1f MPa" % r_n["presion"], "%.2f A" % r_n["corriente"], "%.1f s" % r_n["t"]]]))
    L.append("\n\nEn la planta simulada, al terminar, el torque real es de %.3f N·m. Mientras gira en la fase rápida, la corriente estimada se aparta de la real en %.3f A como máximo, %.1f %%.\n\n" % (
        np.max(real_n[:, 1]), err_i.max(), 100 * err_i.max() / np.mean(real_n[rap, 0])))
    L.append("""## 4. Tornillos que salen mal

![Casos](casos.png)

""")
    rows = [["Caso", "Qué pasa", "Resultado del firmware", "Vueltas", "Torque"]]
    what = {"Normal": "Asienta a las 12 vueltas", "Sin tornillo": "El servo no soltó nada", "Atasco": "Se traba a las 3 vueltas", "Rosca barrida": "Se barre a 0,25 N·m"}
    for name, (_, _, _, res) in cases.items():
        rows.append([name, what[name], "`%s`" % res["resultado"], "%.2f" % res["vueltas"], "%.3f N·m" % res["torque"]])
    L.append(table(rows))
    L.append("\n\nSi el resultado no es `ok`, la secuencia aborta y no pasa al siguiente hueco. Sin señal del encoder, el modelo ve el motor quieto y el firmware informa `trabado` antes de 1 s: nunca da por bueno un tornillo. El mismo resultado aparece con un atasco real, así que primero revisa el cable de J8. En ese caso el torque y la fuerza del informe no valen, porque el modelo cree que el motor está calado.\n\n")
    L.append("""## 5. Cuánto se equivoca si el motor no es igual a la hoja de datos

El firmware sigue usando los valores de la sección 1; solo cambia el motor simulado.

""")
    rows = [["Motor real", "Resultado", "Torque estimado", "Torque real", "Error"]]
    for name, res, tr in sens:
        rows.append([name, "`%s`" % res["resultado"], "%.3f N·m" % res["torque"], "%.3f N·m" % tr, "%+.0f %%" % (100 * (res["torque"] - tr) / tr)])
    L.append(table(rows))
    L.append("""

Por eso los N·m son una estimación hasta calibrar. La profundidad no tiene este problema: sale del encoder, sin usar el modelo. Para calibrar:

1. **R:** con M3 sin conectar a la placa y el eje frenado, mide la resistencia con un multímetro en varias posiciones del eje.
2. **Ke:** gira M3 en vacío con `tool 50`, lee la velocidad y calcula `Ke = (V - R I0) / w`.
3. **Kt:** mide el torque con una llave dinamométrica o un peso en un brazo conocido.
4. Cambia `M3_MODEL` en `src/config.c`.

## 6. Proceso completo

Secuencia: `ros2/secuencia.yaml`. Base del brazo en (15; 0) cm, placas de 38 mm sin girar, con centro en A, B, C y D, y huecos a 3,5 mm de cada borde.

""")
    if seq.notes:
        L.append("> " + " ".join(seq.notes) + "\n\n")
    rows = [["Hueco", "x [mm]", "y [mm]", "Error al soltar [mm]", "Resultado", "Profundidad [mm]", "Torque [N·m]", "Fuerza [N]", "Corriente [A]"]]
    res_by = {r["hueco"]: r for r in runner.results}
    for h in seq.process.holes:
        r = res_by.get(h["label"])
        rows.append([h["label"], "%.1f" % h["x"], "%.1f" % h["y"], "%.2f" % errors.get(h["label"], float("nan"))] +
                    (["`%s`" % r["resultado"], "%.2f" % r["prof"], "%.3f" % r["torque"], "%.0f" % r["fuerza"], "%.2f" % r["corriente"]] if r else ["-"] * 5))
    L.append(table(rows))
    L.append("\n\nEstado final: **%s**. Duración simulada del ciclo: %.0f s. Error máximo de posición al soltar: %.2f mm. La simulación no incluye juego mecánico ni error de montaje. Suponiendo un hueco de 3,2 mm y un tornillo de 3 mm, la holgura es de 0,1 mm por lado, menor que ese error: hará falta una punta o un chaflán que guíe el tornillo. **Revísalo en el robot.**\n\n" % (
        runner.state, t_total, max(errors.values())))
    L.append("""## 7. Lo que falta medir o confirmar

- Modelo y relación de M3, y sus valores R, Ke y Kt.
- Tornillo real: paso, largo, material y torque de apriete. Valores actuales: 12 vueltas, 0,4 N·m.
- Sentido de M3 y del encoder (`TOOL_MOTOR_SIGN`, `TOOL_ENC_SIGN`) y cuentas por vuelta (`TOOL_ENC_CUENTAS`).
- Ángulo de subida del servo (`SERVO_ARRIBA_DEG`, ángulo y signo). Siempre arranca en 0 (`SERVO_ABAJO_DEG`, destornillador abajo).
- **Señal del servo:** en la placa, cambia R40 por 330 Ω y retira C14. R41 se queda.
- **Alimentación del servo:** el MG90S puede pedir unos 700 mA calado, de los mismos 5 V del LM2596 que alimentan encoders y lógica. Pon 470 µF entre los pines 1 (+) y 3 (-) de J13 y comprueba que el ESP32 no se reinicie cuando el servo se mueve.
- **Altura:** el brazo no tiene eje z. Comprueba que la punta pase sobre las placas sin chocar en los traslados, y que M3 baje con el tornillo mientras este entra (6 mm).
- Posición de los huecos: se midió en la imagen, con unos ±0,1 mm.
- Origen: (14,5; −20) cm queda fuera de alcance por 1 cm. Si en el robot real debe llegar ahí, hay que mover la base o alargar el brazo.
""")
    with open(os.path.join(DOCS, "REPORTE_M3.md"), "w") as f:
        f.write("".join(L))


def main():
    os.makedirs(DOCS, exist_ok=True)
    sim = Sim()
    cases = {"Normal": screw(sim), "Sin tornillo": screw(sim, NO_CAE), "Atasco": screw(sim, ATASCO), "Rosca barrida": screw(sim, BARRIDO)}
    sens = []
    for name, f in (("Igual a la hoja de datos", (1, 1, 1)), ("R +20 %", (1.2, 1, 1)), ("R −20 %", (0.8, 1, 1)), ("Ke +10 %", (1, 1.1, 1)), ("Ke −10 %", (1, 0.9, 1)),
                    ("Kt +20 %", (1, 1, 1.2)), ("Kt −20 %", (1, 1, 0.8))):
        _, _, real, res = screw(sim, NORMAL, f)
        sens.append((name, res, float(np.max(real[:, 1]))))
    sim.lib.sim_m3_factors(1.0, 1.0, 1.0)
    seq = sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml"))
    runner, errors, t_total = run_process(sim, seq)
    plot_normal(*cases["Normal"][:3])
    plot_cases(cases)
    write(cases, sens, seq, runner, errors, t_total)
    for n, (_, _, _, r) in cases.items():
        print(n, r)
    for s in sens:
        print(s[0], s[1]["resultado"], s[1]["torque"], round(s[2], 3))
    print(runner.state, runner.reason, len(runner.results), "t=%.0f" % t_total, "err max %.2f" % max(errors.values()))


if __name__ == "__main__":
    main()
