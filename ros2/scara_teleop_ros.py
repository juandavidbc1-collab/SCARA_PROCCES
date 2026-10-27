#!/usr/bin/env python3
"""
Panel de control del SCARA sobre ROS 2 (micro-ROS): teclado con dead-man + todo lo del monitor serie.

TECLADO (con el foco en la ventana, no en un cuadro de texto):
    ESPACIO (mantener)  DEAD-MAN: sin él las flechas no mueven nada
    ← →  articulación 1 (− / +)      ↓ ↑  articulación 2 (− / +)
    h Home    s Detener    o Apagar motores    c Borrar falla    q Salir    Esc Detener

PANEL (ventana):  botones Home / Detener / Apagar / Borrar falla / Estado,
    mover en x, y, z [mm] (con codo +1/-1),  mover en ángulos q1, q2 [grados],
    ganancias PID por eje (Leer / Aplicar), pulsos de prueba, línea de comandos libre y registro de eventos.

Publica en /scara/jog (Vector3, 25 Hz: x = dead-man, y = dir J1, z = dir J2), /scara/goto y /scara/cmd.
El ESP32 se detiene solo si dejan de llegar latidos (DEADMAN_TIMEOUT_MS = 300 ms con ROS 2).

Tres formas de leer el teclado (se elige sola; fuerza una con --modo):
  ventana (tk)   Panel completo. Funciona en Wayland y en X11.  Necesita:  sudo apt install python3-tk
  pynput         Solo teclado, en sesión X11:  pip install pynput
  terminal       Último recurso, solo teclado: un terminal solo repite la ÚLTIMA tecla pulsada, así que aquí
                 la flecha misma hace de dead-man (al soltarla, se detiene en ~0.2 s). Espacio no se usa.
"""
import argparse
import collections
import os
import re
import select
import sys
import time

HZ = 25
ARROW_NAMES = {"Left": "left", "Right": "right", "Up": "up", "Down": "down", "space": "space"}
MOVE_KEYS = ("space", "left", "right", "up", "down")
SHORTCUTS = {"h": "home", "s": "stop", "o": "off", "c": "clear"}


def jog_from_keys(pressed, space_required=True):
    """pressed: conjunto de nombres de tecla -> (dm, d1, d2). Función pura (se prueba sin teclado)."""
    d1 = (1 if "right" in pressed else 0) - (1 if "left" in pressed else 0)
    d2 = (1 if "up" in pressed else 0) - (1 if "down" in pressed else 0)
    if space_required:
        if "space" not in pressed:
            return 0.0, 0.0, 0.0
        return 1.0, float(d1), float(d2)
    # modo terminal: la propia flecha es el dead-man
    return (1.0 if (d1 or d2) else 0.0), float(d1), float(d2)


_SEQ = re.compile(r"\x1b\[([ABCD])|\x1b|(.)", re.S)
_ARROW = {"A": "up", "B": "down", "C": "right", "D": "left"}


def parse_terminal_bytes(chunk):
    """Convierte lo leído del terminal en eventos: ('key', 'left'|...|'space') | ('char', 'h') | ('esc', None).
    Una flecha llega como 'ESC [ C' (3 caracteres): NO debe confundirse con Esc ni con la letra 'c'."""
    ev = []
    for m in _SEQ.finditer(chunk):
        if m.group(1):
            ev.append(("key", _ARROW[m.group(1)]))
        elif m.group(2) is None:
            ev.append(("esc", None))
        else:
            ch = m.group(2)
            ev.append(("key", "space") if ch == " " else ("char", ch.lower()))
    return ev


# ---------------------------------------------------------------- nodo ROS 2
def make_node(rclpy_mod):
    from geometry_msgs.msg import Point, Vector3
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Float32MultiArray, String

    best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

    class Teleop(Node):
        def __init__(self):
            super().__init__("scara_teleop")
            self.pub_jog = self.create_publisher(Vector3, "/scara/jog", best_effort)
            self.pub_cmd = self.create_publisher(String, "/scara/cmd", reliable)
            self.pub_goto = self.create_publisher(Point, "/scara/goto", reliable)
            self.create_subscription(String, "/scara/event", self._on_event, reliable)
            self.create_subscription(Float32MultiArray, "/scara/state", self._on_state, best_effort)
            self.events = collections.deque(maxlen=500)
            self.state = None
            self.echo = True                      # imprimir eventos en la terminal (el panel los muestra él)
            self.pressed = set()
            self.space_required = True
            self.running = True

        def _on_event(self, m):
            self.events.append(m.data)
            if self.echo:
                print(m.data)

        def _on_state(self, m):
            self.state = list(m.data)

        def publish_jog(self):
            dm, d1, d2 = jog_from_keys(self.pressed, self.space_required)
            self.pub_jog.publish(Vector3(x=dm, y=d1, z=d2))

        def command(self, text):
            self.pub_cmd.publish(String(data=text))

        def goto(self, x, y, z):
            self.pub_goto.publish(Point(x=float(x), y=float(y), z=float(z)))

        def shortcut(self, k):
            cmd = SHORTCUTS.get(k)
            if cmd:
                self.command(cmd)
            elif k == "q":
                self.running = False

    return Teleop()


# ---------------------------------------------------------------- 1) panel Tk
MODES = ["OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT"]
GAIN_RE = re.compile(r"J([12]) kp=(-?[\d.]+) ki=(-?[\d.]+) kd=(-?[\d.]+) kff=(-?[\d.]+) fric=(-?[\d.]+)")
GAIN_NAMES = ("kp", "ki", "kd", "kff", "fric")


def fmt_topes(bits):
    """bit (2*j + lado): J1 min=bit0, J1 max=bit1, J2 min=bit2, J2 max=bit3. Devuelve p. ej. 'J1[min -  max -]  J2[min -  max ON]'."""
    b = int(bits)
    on = lambda i: "ON" if b >> i & 1 else "-"
    return "J1[min %s max %s]  J2[min %s max %s]" % (on(0), on(1), on(2), on(3))


def fmt_state(d):
    """13 datos de /scara/state -> dos líneas legibles."""
    if not d or len(d) < 13:
        return "sin datos del ESP32 (¿agente micro-ROS corriendo?)"
    m = int(d[4])
    return ("[%s]  q1=%7.2f  q2=%7.2f deg   x=%6.1f  y=%6.1f mm   home J1:%s J2:%s   falla:%s\n"
            "finales de carrera presionados: %s") % (
        MODES[m] if 0 <= m < len(MODES) else "?", d[0], d[1], d[2], d[3],
        "ok" if d[6] else "NO", "ok" if d[7] else "NO", "ninguna" if int(d[5]) == 0 else int(d[5]), fmt_topes(d[8]))


class ControlPanel:
    """Ventana Tk: teclado con estado REAL de las teclas + panel de control. X11 genera 'soltar+pulsar' en cada
    repetición automática: se ignora una suelta si en menos de RELEASE_DEBOUNCE_MS vuelve la misma tecla.
    Con el foco en un cuadro de texto el teclado del robot se ignora (para poder escribir números)."""
    RELEASE_DEBOUNCE_MS = 45
    TEXT_CLASSES = ("Entry", "TEntry", "Text", "TCombobox", "TSpinbox", "Spinbox")

    def __init__(self, node, tk, ttk):
        self.node, self.tk, self.ttk = node, tk, ttk
        self.root = tk.Tk()
        self.root.title("SCARA - panel de control (clic fuera de los cuadros para usar el teclado)")
        self.root.geometry("900x850")
        self.timers = {}
        self.history, self.hist_i = [], 0
        self.gain_read = False
        self._build()
        self.root.bind("<KeyPress>", self.on_press)
        self.root.bind("<KeyRelease>", self.on_release)
        self.root.bind("<FocusOut>", self.on_focus_out)
        self.root.bind("<Button-1>", self.on_click, add="+")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.focus_force()

    # ---- construcción
    def _build(self):
        tk, ttk, root = self.tk, self.ttk, self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(5, weight=1)
        mono = ("monospace", 10)

        top = ttk.Frame(root); top.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 2))
        self.state_lbl = tk.Label(top, text="", font=("monospace", 10, "bold"), anchor="w", justify="left")
        self.state_lbl.pack(fill="x")
        self.key_lbl = tk.Label(top, text="", font=mono, anchor="w")
        self.key_lbl.pack(fill="x")

        acc = ttk.LabelFrame(root, text="Acciones"); acc.grid(row=1, column=0, sticky="ew", padx=8, pady=3)
        for i, (txt, cmd) in enumerate((("Home", "home"), ("Detener", "stop"), ("Apagar motores", "off"),
                                         ("Borrar falla", "clear"), ("Estado", "status"), ("Ayuda", "help"))):
            self._btn(acc, txt, lambda c=cmd: self.send(c)).grid(row=0, column=i, padx=4, pady=4)
        self._btn(acc, "Pulso J1", lambda: self.send("pulse 1")).grid(row=0, column=6, padx=(20, 4))
        self._btn(acc, "Pulso J2", lambda: self.send("pulse 2")).grid(row=0, column=7, padx=4)

        mid = ttk.Frame(root); mid.grid(row=2, column=0, sticky="ew", padx=8, pady=3)
        mid.columnconfigure(0, weight=1); mid.columnconfigure(1, weight=1)

        xyz = ttk.LabelFrame(mid, text="Mover en x, y, z (mm)"); xyz.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        self.xyz = [tk.StringVar(value=v) for v in ("120", "80", "0")]
        for i, (lbl, var) in enumerate(zip(("x", "y", "z"), self.xyz)):
            ttk.Label(xyz, text=lbl).grid(row=0, column=2 * i, padx=(6, 1), pady=6)
            e = self._entry(xyz, var, 7); e.grid(row=0, column=2 * i + 1)
            e.bind("<Return>", lambda _e: self.do_goto())
        self._btn(xyz, "Ir", self.do_goto).grid(row=0, column=6, padx=8)
        self.elbow = tk.StringVar(value="+1")
        ttk.Label(xyz, text="Codo").grid(row=1, column=0, columnspan=2, sticky="e")
        for i, v in enumerate(("+1", "-1")):
            ttk.Radiobutton(xyz, text=v, value=v, variable=self.elbow, takefocus=False,
                            command=lambda: self.send("elbow " + self.elbow.get())).grid(row=1, column=2 + i, pady=(0, 6))

        ang = ttk.LabelFrame(mid, text="Mover en ángulos (grados)"); ang.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        self.ang = [tk.StringVar(value=v) for v in ("20", "90")]
        for i, (lbl, var) in enumerate(zip(("q1", "q2"), self.ang)):
            ttk.Label(ang, text=lbl).grid(row=0, column=2 * i, padx=(6, 1), pady=6)
            e = self._entry(ang, var, 8); e.grid(row=0, column=2 * i + 1)
            e.bind("<Return>", lambda _e: self.do_ang())
        self._btn(ang, "Ir", self.do_ang).grid(row=0, column=4, padx=8)

        g = ttk.LabelFrame(root, text="Ganancias PID por eje  (los cambios se pierden al reiniciar: para fijarlos edita config.c)")
        g.grid(row=3, column=0, sticky="ew", padx=8, pady=3)
        self.gain_vars = {1: [tk.StringVar() for _ in GAIN_NAMES], 2: [tk.StringVar() for _ in GAIN_NAMES]}
        for c, name in enumerate(GAIN_NAMES):
            ttk.Label(g, text=name).grid(row=0, column=c + 1)
        for r, j in enumerate((1, 2), start=1):
            ttk.Label(g, text="J%d" % j).grid(row=r, column=0, padx=6)
            for c in range(5):
                self._entry(g, self.gain_vars[j][c], 8).grid(row=r, column=c + 1, padx=2, pady=2)
            self._btn(g, "Aplicar J%d" % j, lambda j=j: self.apply_gain(j)).grid(row=r, column=6, padx=8)
        self._btn(g, "Leer del robot", lambda: self.send("gain")).grid(row=1, column=7, rowspan=2, padx=8)

        cmdf = ttk.LabelFrame(root, text="Línea de comandos (igual que el monitor serie; 'help' lista los comandos)")
        cmdf.grid(row=4, column=0, sticky="ew", padx=8, pady=3)
        cmdf.columnconfigure(0, weight=1)
        self.cmd_var = tk.StringVar()
        self.cmd_entry = self._entry(cmdf, self.cmd_var, 60); self.cmd_entry.grid(row=0, column=0, sticky="ew", padx=6, pady=6)
        self.cmd_entry.bind("<Return>", self.do_cmdline)
        self.cmd_entry.bind("<Up>", lambda e: self.hist_move(-1))
        self.cmd_entry.bind("<Down>", lambda e: self.hist_move(+1))
        self._btn(cmdf, "Enviar", self.do_cmdline).grid(row=0, column=1, padx=6)

        logf = ttk.LabelFrame(root, text="Registro de eventos"); logf.grid(row=5, column=0, sticky="nsew", padx=8, pady=3)
        logf.columnconfigure(0, weight=1); logf.rowconfigure(0, weight=1)
        self.log_w = tk.Text(logf, height=9, font=mono, state="disabled", wrap="word", takefocus=0)
        sb = ttk.Scrollbar(logf, command=self.log_w.yview); self.log_w.config(yscrollcommand=sb.set)
        self.log_w.grid(row=0, column=0, sticky="nsew"); sb.grid(row=0, column=1, sticky="ns")
        for tag, col in (("err", "#c62828"), ("ok", "#2e7d32"), ("tx", "#1565c0")):
            self.log_w.tag_config(tag, foreground=col)

        tk.Label(root, font=mono, anchor="w", justify="left", wraplength=870, text=(
            "Teclado: ESPACIO (mantener) = dead-man + flechas: J1 (<- ->) J2 (abajo/arriba) | h Home  s Detener  "
            "o Apagar  c Borrar falla  Esc Detener  q Salir\nSi la ventana pierde el foco o escribes en un cuadro, "
            "el robot se detiene. Clic fuera de los cuadros (o Esc en la línea de comandos) devuelve el teclado al robot."
        )).grid(row=6, column=0, sticky="ew", padx=8, pady=(2, 8))

    def _btn(self, parent, text, command):
        def run():
            command()
            self.root.focus_set()                 # que ESPACIO no vuelva a pulsar este botón
        return self.ttk.Button(parent, text=text, command=run, takefocus=False)

    def _entry(self, parent, var, width):
        e = self.ttk.Entry(parent, textvariable=var, width=width)
        e.bind("<Escape>", lambda _e: (self.root.focus_set(), "break")[1])
        return e

    # ---- registro y envío
    def log(self, text, tag=None):
        w = self.log_w
        w.config(state="normal")
        w.insert("end", text + "\n", tag or ())
        if int(w.index("end-1c").split(".")[0]) > 500:
            w.delete("1.0", "50.0")
        w.see("end")
        w.config(state="disabled")

    def send(self, text):
        self.node.command(text)
        self.log("> " + text, "tx")

    def _nums(self, vars_, names):
        try:
            return [float(v.get().replace(",", ".")) for v in vars_]
        except ValueError:
            self.log("#ERROR (panel): escribe números válidos en %s" % ", ".join(names), "err")
            return None

    def do_goto(self):
        v = self._nums(self.xyz, "x, y, z")
        if v:
            self.node.goto(*v)
            self.log("> goto %g %g %g" % tuple(v), "tx")

    def do_ang(self):
        v = self._nums(self.ang, "q1, q2")
        if v:
            self.send("ang %g %g" % tuple(v))

    def apply_gain(self, j):
        v = self._nums(self.gain_vars[j], GAIN_NAMES)
        if v:
            self.send("gain %d %g %g %g %g %g" % ((j,) + tuple(v)))
            self.root.after(300, lambda: self.send("gain"))      # relee para confirmar lo que quedó

    def do_cmdline(self, _ev=None):
        text = self.cmd_var.get().strip()
        if text:
            self.send(text)
            if not self.history or self.history[-1] != text:
                self.history.append(text)
            self.hist_i = len(self.history)
            self.cmd_var.set("")

    def hist_move(self, d):
        if not self.history:
            return "break"
        self.hist_i = max(0, min(len(self.history), self.hist_i + d))
        self.cmd_var.set(self.history[self.hist_i] if self.hist_i < len(self.history) else "")
        self.cmd_entry.icursor("end")
        return "break"

    # ---- teclado
    def _is_text(self, w):
        try:
            return w.winfo_class() in self.TEXT_CLASSES
        except Exception:
            return False

    def name(self, ev):
        return ARROW_NAMES.get(ev.keysym) or (ev.keysym.lower() if len(ev.keysym) == 1 else ev.keysym)

    def on_click(self, ev):
        if not self._is_text(ev.widget):
            self.root.after(1, self.root.focus_set)

    def on_press(self, ev):
        if self._is_text(ev.widget):
            return                                # escribiendo en un cuadro: el robot no recibe teclas
        k = self.name(ev)
        if k in self.timers:                      # repetición automática: cancela la 'suelta' falsa
            self.root.after_cancel(self.timers.pop(k))
            return
        if k in MOVE_KEYS:
            self.node.pressed.add(k)
            self.node.publish_jog()
        elif k == "Escape":
            self.node.command("stop")
        elif len(k) == 1:
            self.node.shortcut(k)

    def on_release(self, ev):
        if self._is_text(ev.widget):
            return
        k = self.name(ev)
        if k in MOVE_KEYS:
            self.timers[k] = self.root.after(self.RELEASE_DEBOUNCE_MS, lambda k=k: self.really_released(k))

    def really_released(self, k):
        self.timers.pop(k, None)
        self.node.pressed.discard(k)
        self.node.publish_jog()                   # soltar = parar de inmediato

    def on_focus_out(self, _ev):
        for t in self.timers.values():
            self.root.after_cancel(t)
        self.timers.clear()
        self.node.pressed.clear()
        self.node.publish_jog()

    def close(self):
        self.node.running = False

    # ---- bucle
    def handle_event(self, ev):
        tag = "err" if ev.startswith(("#ERROR", "#FALLA", "#err")) else "ok" if ev.startswith(("#LLEGO", "#HOME: terminado")) else None
        self.log(ev, tag)
        for m in GAIN_RE.finditer(ev):             # respuesta de 'gain': rellena los cuadros
            j = int(m.group(1))
            for var, val in zip(self.gain_vars[j], m.groups()[1:]):
                var.set(val)

    def tick(self):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node.publish_jog()
        while self.node.events:
            self.handle_event(self.node.events.popleft())
        st = self.node.state
        self.state_lbl.config(text=fmt_state(st))
        if st is not None and not self.gain_read:
            self.gain_read = True
            self.send("gain")                      # al conectar, trae las ganancias actuales
        dm, d1, d2 = jog_from_keys(self.node.pressed)
        typing = self._is_text(self.root.focus_get())
        self.key_lbl.config(text=("TECLADO DEL ROBOT DESACTIVADO (escribiendo en un cuadro)" if typing else
                                  "DEAD-MAN %s   J1 %+d   J2 %+d" % ("ON " if dm else "off", d1, d2)))
        if self.node.running:
            self.root.after(int(1000 / HZ), self.tick)
        else:
            self.root.destroy()

    def run(self):
        self.root.after(int(1000 / HZ), self.tick)
        self.root.mainloop()


def run_tk(node):
    import tkinter
    from tkinter import ttk
    node.echo = False
    ControlPanel(node, tkinter, ttk).run()


# ---------------------------------------------------------------- 2) pynput
def run_pynput(node):
    import rclpy
    from pynput import keyboard

    special = {keyboard.Key.space: "space", keyboard.Key.left: "left", keyboard.Key.right: "right",
               keyboard.Key.up: "up", keyboard.Key.down: "down", keyboard.Key.esc: "esc"}

    def name(key):
        if key in special:
            return special[key]
        ch = getattr(key, "char", None)
        return ch.lower() if ch else None

    def on_press(key):
        n = name(key)
        if n in MOVE_KEYS:
            node.pressed.add(n)
            node.publish_jog()
        elif n == "esc":
            node.command("stop")
        elif n:
            node.shortcut(n)

    def on_release(key):
        n = name(key)
        node.pressed.discard(n)
        if n in MOVE_KEYS:
            node.publish_jog()

    with keyboard.Listener(on_press=on_press, on_release=on_release):
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish_jog()
            time.sleep(1.0 / HZ)


# ---------------------------------------------------------------- 3) terminal (último recurso)
def run_terminal(node):
    import termios
    import tty
    import rclpy

    HOLD = 0.2                                     # una flecha cuenta como "mantenida" HOLD s tras su último carácter
    node.space_required = False
    last = {}
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while node.running and rclpy.ok():
            r, _, _ = select.select([fd], [], [], 1.0 / HZ)
            if r:
                chunk = os.read(fd, 64).decode("utf-8", "ignore")      # os.read: sin el búfer de Python
                for kind, val in parse_terminal_bytes(chunk):
                    if kind == "key":
                        last[val] = time.time()
                    elif kind == "esc":
                        node.command("stop")
                    else:
                        node.shortcut(val)
            now = time.time()
            node.pressed = {k for k, t in last.items() if now - t < HOLD}
            node.publish_jog()
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def pick_mode(requested):
    if requested != "auto":
        return requested
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        try:
            import tkinter  # noqa: F401
            return "tk"
        except Exception:
            print("[tkinter no está instalado: sudo apt install python3-tk]")
    if os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        try:
            import pynput  # noqa: F401
            from pynput import keyboard  # noqa: F401
            return "pynput"
        except Exception:
            pass
    return "terminal"


def main():
    import rclpy
    ap = argparse.ArgumentParser(description="Teleoperación por teclado del SCARA (ROS 2 / micro-ROS)")
    ap.add_argument("--modo", choices=["auto", "tk", "pynput", "terminal"], default="auto")
    ap.add_argument("--terminal", action="store_true", help="atajo de --modo terminal")
    a = ap.parse_args()
    modo = "terminal" if a.terminal else pick_mode(a.modo)

    rclpy.init()
    node = make_node(rclpy)
    print(__doc__)
    print("Modo de teclado:", modo)
    try:
        {"tk": run_tk, "pynput": run_pynput, "terminal": run_terminal}[modo](node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C: rclpy ya cerró su contexto y publicar daría error. El ESP32 igual se detiene solo
        # (dead-man por falta de latidos), así que aquí solo se intenta soltar el dead-man si aún se puede.
        try:
            if rclpy.ok():
                for _ in range(3):
                    node.pressed.clear()
                    node.publish_jog()
                    time.sleep(0.02)
                node.command("stop")
        except Exception:
            pass
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        print("\nSaliendo (motores en HOLD).")


if __name__ == "__main__":
    main()
