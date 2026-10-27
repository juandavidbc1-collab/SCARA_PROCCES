#!/usr/bin/env python3
"""
CARACTERIZACION PUNTO A PUNTO (Reporte 3): N puntos x M repeticiones.

Para cada repeticion: va a un punto de "aproximacion" fijo (para llegar siempre desde el mismo
lado y no mezclar histeresis del reductor con la repetibilidad) y luego al punto objetivo.
Espera "#LLEGO" y guarda lo que el robot cree (encoders -> cinematica directa).

TIERRA DE REFERENCIA (ground truth): los encoders miden lo mismo que usa el control, asi que
NO sirven para validar la exactitud. Use una referencia externa: papel milimetrado/regla bajo
la punta, o plantilla impresa. Con --medir el script pide x,y reales tras la ultima
repeticion de cada punto (o tras cada una con --medir-cada). Sin --medir solo reporta
repetibilidad (dispersion), no exactitud.

Uso:
  python characterize.py --port COM5 --reps 10 --medir
  python characterize.py --port COM5 --points "120,80;100,-60;150,0;80,60;60,-90"
Requisito: haber hecho Home (la bandera la verifica el script con ST).
"""
import argparse, csv, math, re, statistics as st, sys, time

try:
    import serial
except ImportError:
    serial = None

LLEGO_RE = re.compile(
    r"#LLEGO: objetivo x=(?P<tx>-?[\d.]+) y=(?P<ty>-?[\d.]+) z=(?P<tz>-?[\d.]+) "
    r"\| medido x=(?P<mx>-?[\d.]+) y=(?P<my>-?[\d.]+) "
    r"\| error ex=(?P<ex>-?[\d.]+) ey=(?P<ey>-?[\d.]+) mm "
    r"\| q1=(?P<q1>-?[\d.]+) q2=(?P<q2>-?[\d.]+) deg")

DEFAULT_POINTS = "120,80;100,-60;150,0;80,60;60,-90"
DEFAULT_APPROACH = "100,0"


def parse_points(s):
    pts = []
    for item in s.split(";"):
        x, y = item.split(",")
        pts.append((float(x), float(y)))
    return pts


def parse_llego(line):
    m = LLEGO_RE.search(line)
    return {k: float(v) for k, v in m.groupdict().items()} if m else None


def parse_st(line):
    """ST,modo,home1,home2,q1,q2,x,y,falla,fc,duty1,duty2,qref1,qref2"""
    p = line.strip().split(",")
    if len(p) < 9 or p[0] != "ST":
        return None
    return {"modo": p[1], "home1": int(p[2]), "home2": int(p[3]), "falla": p[8]}


def summarize(rows):
    """rows: dict punto -> lista de dicts con ex, ey (y opcional real_x, real_y)."""
    out = []
    for (tx, ty), reps in rows.items():
        mx = [r["mx"] for r in reps]; my = [r["my"] for r in reps]
        cx, cy = st.mean(mx), st.mean(my)
        # repetibilidad: radio medio de dispersion respecto al centroide (estilo ISO 9283 RP)
        d = [math.hypot(a - cx, b - cy) for a, b in zip(mx, my)]
        rp = st.mean(d) + (st.stdev(d) if len(d) > 1 else 0.0) * 3.0
        row = {"tx": tx, "ty": ty, "n": len(reps), "cx_enc": cx, "cy_enc": cy,
               "RP_mm": rp, "sx": st.stdev(mx) if len(mx) > 1 else 0.0,
               "sy": st.stdev(my) if len(my) > 1 else 0.0}
        real = [(r["rx"], r["ry"]) for r in reps if r.get("rx") is not None]
        if real:
            rx, ry = st.mean(a for a, _ in real), st.mean(b for _, b in real)
            row.update(rx=rx, ry=ry, AP_mm=math.hypot(rx - tx, ry - ty),  # exactitud vs objetivo
                       sesgo_enc_mm=math.hypot(cx - rx, cy - ry))        # error del modelo/encoder
        out.append(row)
    return out


class Robot:
    def __init__(self, ser):
        self.ser = ser
        self.last_st = None

    def send(self, line):
        self.ser.write((line + "\n").encode())

    def lines(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            raw = self.ser.readline()
            if raw:
                yield raw.decode(errors="replace").strip()

    def wait(self, pred, timeout):
        for ln in self.lines(timeout):
            if ln.startswith("ST,"):
                self.last_st = parse_st(ln)
            if ln.startswith(("#ERROR", "#err", "#FALLA")):
                raise RuntimeError(ln)
            r = pred(ln)
            if r is not None:
                return r
        raise TimeoutError("sin respuesta")

    def goto(self, x, y, timeout=20):
        self.send(f"goto {x:.2f} {y:.2f}")
        return self.wait(parse_llego, timeout)


def main():
    ap = argparse.ArgumentParser(description="Caracterizacion punto a punto del SCARA")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--points", default=DEFAULT_POINTS, help='"x,y;x,y;..." (>=5 puntos)')
    ap.add_argument("--approach", default=DEFAULT_APPROACH, help='punto previo comun "x,y"')
    ap.add_argument("--reps", type=int, default=10)
    ap.add_argument("--medir", action="store_true", help="pedir x,y reales tras la ultima repeticion de cada punto")
    ap.add_argument("--medir-cada", action="store_true", help="pedir x,y reales en cada repeticion")
    ap.add_argument("--out", default="caracterizacion.csv")
    a = ap.parse_args()

    pts = parse_points(a.points)
    ax, ay = parse_points(a.approach)[0]
    if len(pts) < 5 or a.reps < 10:
        print("AVISO: el reporte pide >=5 puntos y >=10 repeticiones.", file=sys.stderr)
    if serial is None:
        sys.exit("Falta pyserial: pip install pyserial")

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = a.port, a.baud, 0.2
    ser.dtr = False        # antes de abrir: evita que la ESP32 se reinicie (y pierda el Home)
    ser.rts = False
    ser.open()
    rb = Robot(ser)
    rb.send("\n")
    rb.send("\x1b")           # sale de cualquier monitor/entrada que haya quedado abierta
    time.sleep(0.2)
    rb.send("fmt 1")          # modo maquina
    rb.send("tele 1")         # sin esto el firmware no manda lineas ST (estado) y no se puede verificar el Home
    time.sleep(0.5)
    ser.reset_input_buffer()
    # verificar Home
    rb.wait(lambda l: (parse_st(l) if l.startswith("ST,") else None), 5)
    s = rb.last_st
    if not (s and s["home1"] and s["home2"]):
        sys.exit("Haga Home primero (comando 'home').")

    rows, f = {}, open(a.out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["punto", "rep", "tx", "ty", "mx_enc", "my_enc", "ex", "ey", "q1", "q2", "rx_real", "ry_real"])
    try:
        for pi, (tx, ty) in enumerate(pts, 1):
            rows[(tx, ty)] = []
            for k in range(1, a.reps + 1):
                try:
                    rb.goto(ax, ay)
                    r = rb.goto(tx, ty)
                except RuntimeError as e:
                    print(f"Punto {pi} ({tx},{ty}) rechazado/falla: {e}. Se omite.")
                    rows.pop((tx, ty), None)
                    break
                r["rx"] = r["ry"] = None
                last = k == a.reps
                if a.medir_cada or (a.medir and last):
                    while True:
                        v = input(f"  P{pi} rep {k}: x,y REALES medidos (mm; Enter = omitir; q = terminar): ").strip()
                        if v.lower() == "q":
                            raise KeyboardInterrupt
                        if not v:
                            break
                        try:
                            r["rx"], r["ry"] = (float(t) for t in v.replace(" ", "").split(","))
                            break
                        except ValueError:
                            print("    Formato: x,y  (ej: 120.5,79.8)")
                rows[(tx, ty)].append(r)
                w.writerow([pi, k, tx, ty, r["mx"], r["my"], r["ex"], r["ey"], r["q1"], r["q2"], r["rx"], r["ry"]])
                f.flush()
                print(f"P{pi} rep {k:2d}: err enc = ({r['ex']:+.3f}, {r['ey']:+.3f}) mm")
    except KeyboardInterrupt:
        print("\nInterrumpido: se resume lo medido hasta aqui.")
    finally:
        rb.send("stop")
        rb.send("tele 0")
        rb.send("fmt 0")      # devuelve el modo texto para usar el monitor serie
        f.close()
        ser.close()

    print("\n=== RESUMEN ===")
    for s in summarize({k: v for k, v in rows.items() if v}):
        line = f"({s['tx']:.0f},{s['ty']:.0f}) n={s['n']} RP={s['RP_mm']:.3f} mm  sx={s['sx']:.3f} sy={s['sy']:.3f}"
        if "AP_mm" in s:
            line += f"  exactitud(real vs objetivo)={s['AP_mm']:.2f} mm  sesgo enc/real={s['sesgo_enc_mm']:.2f} mm"
        print(line)
    print(f"Datos crudos: {a.out}")


if __name__ == "__main__":
    main()
