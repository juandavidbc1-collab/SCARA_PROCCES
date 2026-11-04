#!/usr/bin/env python3
"""
Proceso repetitivo: lee una secuencia de un archivo YAML y la ejecuta orden por orden, esperando a que el robot
confirme cada punto. No usa ROS: quien la usa le pasa una función para enviar comandos y le entrega los eventos.

Dos formatos (ver ros2/secuencia.yaml):

    repeticiones: 1
    proceso:                   # recorrer todos los huecos de unas placas; se expande a pasos
      base_cm, codo, placas_cm, lado_mm, hueco_borde_mm, asentar_s, servo, m3

    repeticiones: 3            # 0 = hasta que se detenga a mano
    pasos:
      - ir: [120, 80]          # x, y [mm] desde la base
      - angulos: [20, 90]      # q1, q2 [grados]
      - esperar: 0.5           # segundos
      - codo: -1
      - servo: {accion: subir, tiempo: 0.4}     # subir, bajar o grados (-180 a 180)
      - m3_ir: 18.6                             # M3 a esas vueltas desde su tope de arriba
      - atornillar: {vueltas: 12, torque: 0.4}  # del diseño anterior, en el que M3 hacía girar la punta
      - m3: {potencia: 40, tiempo: 1.5}         # motor 3 sin control: -100..100 %, durante un tiempo [s]

En cada hueco: sube el servo, baja M3, baja el servo y M3 sube 1/4 de la barra. El servo no para en el medio. Al final, el brazo va a 0 0.
Antes hay que hacer "m3 home" y el Home del brazo. Si el robot informa una falla o rechaza un punto, la secuencia aborta.

load_sequence_full(path, pcb=2) hace solo esa placa; load_sequence_full(path, ensayo=True) no mueve el servo.
"""
import math
import re
from dataclasses import dataclass, field

import yaml

L1_MM, L2_MM = 125.0, 105.0           # mantener igual que KIN en src/config.c; L2 llega hasta la punta
SING_MARGIN_RAD = math.radians(5.0)
Q1_LIM_DEG = (-75.7, 75.9)            # lo que midió el Home en el robot
Q2_LIM_DEG = (-113.4, 109.0)
M3_ABAJO_VUELTAS, M3_TRABAJO_VUELTAS = 24.855, 18.641   # M3_VUELTAS_ATORNILLAR y M3_VUELTAS_TRABAJO de src/config.h
M3_TIMEOUT_S = 60.0                   # PROC_LIMITE_M3_S de src/config.h
STEP_TIMEOUT_S = 25.0                 # un movimiento o un tornillo tardan menos; si no, algo anda mal
TOOL_RENEW_S = 5.0                    # debe ser menor que TOOL_MAX_ON_MS del firmware
RESULTADOS_BUENOS = ("ok", "ensayado", "hecho")
SCREW_VUELTAS_EXTRA = 2.0             # el firmware corta el tornillo a las vueltas pedidas + 2 (src/control.c)

_TORNILLO = re.compile(r"#TORNILLO: (\w+) vueltas=(-?[\d.]+) prof=(-?[\d.]+)mm avance=(-?[\d.]+)% torque=(-?[\d.]+)Nm "
                       r"fuerza=(-?[\d.]+)N presion=(-?[\d.]+)MPa corriente=(-?[\d.]+)A t=(-?[\d.]+)s")
_TORNILLO_CAMPOS = ("vueltas", "prof", "avance", "torque", "fuerza", "presion", "corriente", "t")


class SequenceError(Exception):
    """Archivo de secuencia no válido. El mensaje está pensado para mostrarlo al usuario."""


@dataclass
class Process:
    """Geometría del proceso, en mm desde la base. Sirve para dibujar las placas."""
    boards: dict
    holes: list
    origin: tuple
    elbow: int
    side: float


@dataclass
class Sequence:
    steps: list
    repeat: int
    process: Process = None
    notes: list = field(default_factory=list)


def parse_tornillo(text):
    """Evento "#TORNILLO: ..." -> diccionario con "resultado" y los números; None si es otro evento."""
    m = _TORNILLO.search(text)
    if not m:
        return None
    return dict(zip(_TORNILLO_CAMPOS, (float(v) for v in m.groups()[1:])), resultado=m.group(1))


_LLEGO = re.compile(r"#LLEGO: objetivo x=(-?[\d.]+) y=(-?[\d.]+) .*medido x=(-?[\d.]+) y=(-?[\d.]+) .*error ex=(-?[\d.]+) ey=(-?[\d.]+)")


def parse_llego(text):
    """Evento "#LLEGO: objetivo ..." -> diccionario con objetivo, medido y error en mm; None si es otro evento."""
    m = _LLEGO.search(text)
    if not m:
        return None
    return dict(zip(("x", "y", "mx", "my", "ex", "ey"), (float(v) for v in m.groups())))


def point_problem(x, y, elbow=None):
    """None si el brazo llega a (x, y) [mm]; si no, el motivo. Con elbow también revisa los límites de ese codo."""
    r = math.hypot(x, y)
    if r > L1_MM + L2_MM or r < abs(L1_MM - L2_MM):
        return "(%g, %g) está fuera de alcance: %.0f a %.0f mm" % (x, y, abs(L1_MM - L2_MM), L1_MM + L2_MM)
    c2 = (r * r - L1_MM ** 2 - L2_MM ** 2) / (2.0 * L1_MM * L2_MM)
    q2 = math.acos(max(-1.0, min(1.0, c2)))
    if q2 < SING_MARGIN_RAD or q2 > math.pi - SING_MARGIN_RAD:
        return "(%g, %g) está en una postura singular del brazo" % (x, y)
    if elbow is not None:
        q2 = elbow * q2
        q1 = math.atan2(y, x) - math.atan2(L2_MM * math.sin(q2), L1_MM + L2_MM * math.cos(q2))
        q1 = math.degrees((q1 + math.pi) % (2 * math.pi) - math.pi)
        q2 = math.degrees(q2)
        if not (Q1_LIM_DEG[0] <= q1 <= Q1_LIM_DEG[1] and Q2_LIM_DEG[0] <= q2 <= Q2_LIM_DEG[1]):
            return "(%.1f, %.1f) se sale de los límites con codo %+d: q1 = %.0f, q2 = %.0f grados" % (x, y, elbow, q1, q2)
    return None


def _numbers(value, n, name):
    if not isinstance(value, (list, tuple)) or len(value) != n or not all(isinstance(v, (int, float)) for v in value):
        raise ValueError("%s necesita %d numeros" % (name, n))
    return [float(v) for v in value]


def _parse_step(raw):
    if not isinstance(raw, dict) or len(raw) != 1:
        raise ValueError("cada paso es una sola orden, por ejemplo  - ir: [120, 80]")
    key, val = next(iter(raw.items()))
    if key == "ir":
        x, y = _numbers(val, 2, "ir")
        problem = point_problem(x, y)
        if problem:
            raise ValueError(problem)
        return _goto(x, y)
    if key == "angulos":
        a, b = _numbers(val, 2, "angulos")
        return {"kind": "ang", "a": a, "b": b, "desc": "Ángulos q1 = %g, q2 = %g grados" % (a, b)}
    if key == "esperar":
        if not isinstance(val, (int, float)) or val < 0:
            raise ValueError("esperar necesita un tiempo positivo en segundos")
        return _wait(float(val))
    if key == "m3":
        if not isinstance(val, dict) or "potencia" not in val or "tiempo" not in val:
            raise ValueError("m3 necesita {potencia: %, tiempo: s}")
        pct, secs = val["potencia"], val["tiempo"]
        if not isinstance(pct, (int, float)) or abs(pct) > 100:
            raise ValueError("la potencia de m3 va de -100 a 100 %")
        if not isinstance(secs, (int, float)) or secs <= 0:
            raise ValueError("el tiempo de m3 debe ser positivo")
        return {"kind": "tool", "pct": float(pct), "secs": float(secs),
                "desc": "Motor 3 al %g %% durante %g s" % (pct, secs)}
    if key == "m3_ir":
        if not isinstance(val, (int, float)) or not 0 <= val <= M3_ABAJO_VUELTAS:
            raise ValueError("m3_ir va de 0 a %g vueltas desde el tope de arriba" % M3_ABAJO_VUELTAS)
        return _m3_ir(float(val), "M3 a %g vueltas" % val)
    if key == "codo":
        if val not in (1, -1):
            raise ValueError("codo es 1 o -1")
        return _cmd("elbow %+d" % val, "Codo %+d" % val)
    if key == "servo":
        if not isinstance(val, dict) or "accion" not in val:
            raise ValueError("servo necesita {accion: subir | bajar | grados, tiempo: s}")
        acc, secs = val["accion"], val.get("tiempo", 0.0)
        if not (acc in ("subir", "bajar") or (isinstance(acc, (int, float)) and -180 <= acc <= 180)):
            raise ValueError("la acción del servo es subir, bajar o de -180 a 180 grados")
        if not isinstance(secs, (int, float)) or secs < 0:
            raise ValueError("el tiempo del servo debe ser positivo")
        return _cmd("servo %s" % acc, "Servo: %s" % acc, float(secs))
    if key == "atornillar":
        if not isinstance(val, dict) or "vueltas" not in val or "torque" not in val:
            raise ValueError("atornillar necesita {vueltas: n, torque: N·m}")
        v, t = val["vueltas"], val["torque"]
        if not isinstance(v, (int, float)) or not 0 < v <= 40 or not isinstance(t, (int, float)) or not 0 < t <= 1.5:
            raise ValueError("atornillar: vueltas de 0 a 40 y torque de 0 a 1.5 N·m")
        return _screw(float(v), float(t))
    raise ValueError("orden desconocida: %s (usa ir, angulos, esperar, codo, servo, m3_ir, atornillar o m3)" % key)


def _goto(x, y, desc=None):
    return {"kind": "goto", "x": x, "y": y, "desc": desc or "Ir a x = %g, y = %g mm" % (x, y)}


def _m3_ir(vueltas, desc, label=None):
    s = {"kind": "m3", "vueltas": vueltas, "desc": desc}
    if label:
        s["label"] = label
    return s


def _wait(secs, desc=None):
    return {"kind": "wait", "secs": secs, "desc": desc or "Esperar %g s" % secs}


def _cmd(text, desc, secs=0.0):
    return {"kind": "cmd", "text": text, "secs": secs, "desc": desc}


def _screw(vueltas, torque, label=None):
    s = {"kind": "screw", "vueltas": vueltas, "torque": torque,
         "desc": "Atornillar %s(%g vueltas, %g N·m)" % (label + " " if label else "", vueltas, torque)}
    if label:
        s["label"] = label
    return s


def _pair(val, name):
    return _numbers(val, 2, name)


def _positive(raw, key, name, default=None):
    val = raw.get(key, default)
    if not isinstance(val, (int, float)) or val < 0:
        raise ValueError("%s debe ser un número positivo" % name)
    return float(val)


def _m3_turns(pct, secs):
    """Vueltas de M3 a ese % en lazo abierto y sin carga: modelo de src/config.c con TOOL_DUTY_MAX = 650."""
    volts = pct / 100.0 * 650.0 / 1023.0 * 12.0
    return max(0.0, volts - 0.2 * 2.1818) / 0.5522 / (2.0 * math.pi) * secs


def _expand_process(raw, ensayo=False, pcb=None):
    """Sección "proceso" -> (pasos, Process, notas). pcb: 1, 2... para hacer solo esa placa."""
    if not isinstance(raw, dict):
        raise ValueError("proceso debe ser una sección con sus datos")
    bx, by = _pair(raw.get("base_cm"), "base_cm")
    elbow = raw.get("codo", -1)
    if elbow not in (1, -1):
        raise ValueError("codo es 1 o -1")
    side, edge = _positive(raw, "lado_mm", "lado_mm"), _positive(raw, "hueco_borde_mm", "hueco_borde_mm")
    if 2 * edge >= side:
        raise ValueError("los huecos no caben: hueco_borde_mm debe ser menor que la mitad de lado_mm")
    placas = raw.get("placas_cm")
    if not isinstance(placas, dict) or not placas:
        raise ValueError("placas_cm necesita al menos una placa, por ejemplo A: [17, 3.4]")
    if pcb is not None and not (isinstance(pcb, int) and 1 <= pcb <= len(placas)):
        raise ValueError("la PCB va de 1 a %d" % len(placas))
    servo, m3 = raw.get("servo") or {}, raw.get("m3") or {}
    t_up, t_down = _positive(servo, "arriba_s", "servo.arriba_s", 1.0), _positive(servo, "bajar_s", "servo.bajar_s", 1.0)
    settle = _positive(raw, "asentar_s", "asentar_s", 0.3)
    abajo, arriba = _positive(m3, "abajo_vueltas", "m3.abajo_vueltas", M3_ABAJO_VUELTAS), _positive(m3, "trabajo_vueltas", "m3.trabajo_vueltas", M3_TRABAJO_VUELTAS)
    if not 0 < arriba < abajo <= M3_ABAJO_VUELTAS:
        raise ValueError("m3: trabajo_vueltas debe ser menor que abajo_vueltas, y esta no puede pasar de %g" % M3_ABAJO_VUELTAS)

    half = side / 2.0 - edge
    boards, holes = {}, []
    steps = [_cmd("elbow %+d" % elbow, "Codo %+d" % elbow)]
    for n, (name, center) in enumerate(placas.items(), start=1):
        cx, cy = _pair(center, "placa %s" % name)
        cx, cy = (cx - bx) * 10.0, (cy - by) * 10.0
        boards[str(name)] = (cx, cy)
        # Primero los dos huecos de arriba y luego los dos de abajo, sin cruzar la placa.
        for hole, sx, sy in (("H2", -1, 1), ("H4", 1, 1), ("H3", 1, -1), ("H1", -1, -1)):
            label = "%s-%s" % (name, hole)
            x, y = cx + sx * half, cy + sy * half
            problem = point_problem(x, y, elbow)
            if problem:
                raise ValueError("hueco %s: %s" % (label, problem))
            if pcb is not None and n != pcb:
                continue
            holes.append({"label": label, "board": str(name), "x": x, "y": y})
            steps += [_goto(x, y, "Ir a %s (%.1f, %.1f) mm" % (label, x, y)), _wait(settle, "Asentar")]
            if ensayo:
                steps += [_m3_ir(abajo, "Bajar M3 en %s" % label, label), dict(_wait(settle, "Pausa abajo"), label=label)]
            else:
                steps += [_cmd("servo subir", "Servo: subir"), _wait(t_up, "Servo subiendo"),
                          _m3_ir(abajo, "Bajar M3 en %s" % label, label), dict(_wait(settle, "Pausa abajo"), label=label),
                          dict(_cmd("servo bajar", "Servo: bajar"), label=label), dict(_wait(t_down, "Servo bajando"), label=label)]
            steps.append(dict(_m3_ir(arriba, "Subir M3 1/4 de la barra en %s" % label, label), fin="ensayado" if ensayo else "hecho"))
    steps.append({"kind": "ang", "a": 0.0, "b": 0.0, "desc": "Volver a 0 0"})
    return steps, Process(boards, holes, (L1_MM + L2_MM, 0.0), elbow, side), []


def load_sequence_full(path, ensayo=False, pcb=None):
    """Devuelve un Sequence. pcb: 1, 2... para hacer solo esa placa. Lanza SequenceError con un mensaje claro si el archivo no sirve."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise SequenceError("no se pudo leer %s: %s" % (path, e)) from e
    if not isinstance(data, dict) or not (data.get("pasos") or data.get("proceso")):
        raise SequenceError("el archivo debe tener una sección proceso o una lista de pasos")
    repeat = data.get("repeticiones", 1)
    if not isinstance(repeat, int) or repeat < 0:
        raise SequenceError("repeticiones debe ser un entero mayor o igual a 0")
    if data.get("proceso"):
        try:
            steps, process, notes = _expand_process(data["proceso"], ensayo, pcb)
        except ValueError as e:
            raise SequenceError("proceso: %s" % e) from e
        return Sequence(steps, repeat, process, notes)
    if not isinstance(data["pasos"], list):
        raise SequenceError("pasos debe ser una lista")
    steps = []
    for i, raw in enumerate(data["pasos"], start=1):
        try:
            steps.append(_parse_step(raw))
        except ValueError as e:
            raise SequenceError("paso %d: %s" % (i, e)) from e
    return Sequence(steps, repeat)


def load_sequence(path):
    """Devuelve (pasos, repeticiones)."""
    seq = load_sequence_full(path)
    return seq.steps, seq.repeat


def _fmt(v):
    return "%g" % v


class SequenceRunner:
    """Máquina de estados: idle -> running -> done | stopped | aborted. Se llama tick() con frecuencia."""

    def __init__(self, steps, repeat, send, clock, step_timeout=STEP_TIMEOUT_S):
        self.steps, self.repeat, self.send, self.clock = steps, repeat, send, clock
        self.step_timeout = step_timeout
        self.state = "idle"
        self.reason = ""
        self.index = 0
        self.cycles_done = 0
        self.cycle_times = []
        self.results = []                 # un diccionario por tornillo terminado
        self._arrived = False
        self._m3_done = False
        self._screw_result = None
        self._servo_up = False
        self._t_step = self._t_cycle = self._t_renew = 0.0

    @property
    def running(self):
        return self.state == "running"

    @property
    def tool_left_down(self):
        """Se cortó con M3 abajo: no se devuelve solo, y el brazo no debería moverse así."""
        if self.state not in ("stopped", "aborted"):
            return False
        return "label" in self.steps[self.index]

    def start(self):
        self.state, self.reason = "running", ""
        self.index = self.cycles_done = 0
        self.cycle_times = []
        self.results = []
        self._servo_up = False
        self._t_cycle = self.clock()
        self._begin_step()

    def stop(self):
        if not self.running:
            return
        self._tool_off_if_needed()
        self.send("stop")
        self.state = "stopped"

    def abort(self, reason):
        """Corta la secuencia sin enviar nada más (por ejemplo, tras un paro de emergencia)."""
        if self.running:
            self.state, self.reason = "aborted", reason

    def on_event(self, text):
        if not self.running:
            return
        if text.startswith("#LLEGO"):
            self._arrived = True
        elif text.startswith("#M3"):
            self._m3_done = True
        elif text.startswith("#TORNILLO"):
            self._screw_result = parse_tornillo(text)
        elif text.startswith(("#FALLA", "#ERROR")):
            self._abort(text.split(":", 1)[-1].strip(), send_stop=False)

    def tick(self):
        if not self.running:
            return
        step = self.steps[self.index]
        now = self.clock()
        kind = step["kind"]
        if kind in ("goto", "ang"):
            if self._arrived:
                self._next_step()
            elif now - self._t_step > self.step_timeout:
                self._abort("se agotó el tiempo esperando llegar al paso %d" % (self.index + 1), send_stop=True)
        elif kind == "wait":
            if now - self._t_step >= step["secs"]:
                self._next_step()
        elif kind == "cmd":
            if now - self._t_step >= step["secs"]:
                self._next_step()
        elif kind == "m3":
            if self._m3_done:
                if step.get("fin"):
                    self.results.append({"hueco": step["label"], "resultado": step["fin"], "ciclo": self.cycles_done + 1})
                self._next_step()
            elif now - self._t_step > M3_TIMEOUT_S:
                self._abort("M3 no llegó a tiempo en el paso %d" % (self.index + 1), send_stop=True)
        elif kind == "screw":
            if self._screw_result is not None:
                self._finish_screw(step, self._screw_result)
            elif now - self._t_step > self.step_timeout:
                self._abort("se agotó el tiempo atornillando en el paso %d" % (self.index + 1), send_stop=True)
        elif kind == "tool":
            if now - self._t_step >= step["secs"]:
                self.send("tool 0")
                if step.get("fin"):
                    self.results.append({"hueco": step["label"], "resultado": "ensayado", "ciclo": self.cycles_done + 1})
                self._next_step()
            elif now - self._t_renew >= TOOL_RENEW_S:
                self._t_renew = now
                self.send("tool " + _fmt(step["pct"]))

    def _finish_screw(self, step, res):
        where = step.get("label") or "paso %d" % (self.index + 1)
        self.results.append(dict(res, hueco=where, ciclo=self.cycles_done + 1))
        if res["resultado"] == "ok":
            self._next_step()
        else:
            self._abort("tornillo %s: %s (%.1f vueltas, %.2f N·m)" % (where, res["resultado"], res["vueltas"], res["torque"]),
                        send_stop=False)

    def _begin_step(self):
        step = self.steps[self.index]
        self._arrived = False
        self._m3_done = False
        self._screw_result = None
        self._t_step = self._t_renew = self.clock()
        if step["kind"] == "goto":
            self.send("goto %s %s" % (_fmt(step["x"]), _fmt(step["y"])))
        elif step["kind"] == "ang":
            self.send("ang %s %s" % (_fmt(step["a"]), _fmt(step["b"])))
        elif step["kind"] == "cmd":
            self.send(step["text"])
            if step["text"].startswith("servo "):
                self._servo_up = step["text"] not in ("servo bajar", "servo 0")
        elif step["kind"] == "screw":
            self.send("screw %s %s" % (_fmt(step["vueltas"]), _fmt(step["torque"])))
        elif step["kind"] == "m3":
            self.send("m3 ir " + _fmt(step["vueltas"]))
        elif step["kind"] == "tool":
            self.send("tool " + _fmt(step["pct"]))

    def _next_step(self):
        self.index += 1
        if self.index >= len(self.steps):
            now = self.clock()
            self.cycles_done += 1
            self.cycle_times.append(now - self._t_cycle)
            self._t_cycle = now
            self.index = 0
            if self.repeat and self.cycles_done >= self.repeat:
                self.state = "done"
                return
        self._begin_step()

    def _tool_off_if_needed(self):
        step = self.steps[self.index]
        if step["kind"] in ("tool", "m3"):
            self.send("tool 0")
        if self._servo_up:                   # subir dura 0 s: casi siempre se llega aquí en la espera de después
            self.send("servo bajar")
            self._servo_up = False

    def _abort(self, reason, send_stop):
        self._tool_off_if_needed()
        if send_stop:
            self.send("stop")
        self.state, self.reason = "aborted", reason
