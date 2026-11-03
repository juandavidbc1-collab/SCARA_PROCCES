#!/usr/bin/env python3
"""Genera tools/host_tests/rst_vectors.h: entradas y salidas de RstController para comprobar src/rst.c contra Python."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rst_design as rd  # noqa: E402

GAINS = (5000.0, 8000.0, 200.0, -0.2)
KFF = 85.0
TS, UMAX = 0.01, 580.0
N = 120


def main():
    rows = []
    for name, (b, c) in rd.FORMS.items():
        ctl = rd.RstController(*GAINS, b, c, TS, y0=0.5, kff=KFF)
        for k in range(N):
            r = 0.5 + 0.3 * (k >= 5) + 0.05 * math.sin(k * 0.2)
            y = 0.5 + 0.2 * (1.0 - math.exp(-k * 0.05)) + 0.001 * ((k * 7) % 3)
            rows.append((b, c, k, r, y, ctl.step(r, y, UMAX)))
    path = os.path.join(HERE, "..", "host_tests", "rst_vectors.h")
    with open(path, "w") as f:
        f.write("/* Generado por tools/rst/gen_vectors.py: no editar. */\n")
        f.write("#define RV_KP %.1ff\n#define RV_KI %.1ff\n#define RV_KD %.1ff\n#define RV_S1 %.3ff\n" % GAINS)
        f.write("#define RV_KFF %.1ff\n" % KFF)
        f.write("#define RV_TS %.3ff\n#define RV_UMAX %.1ff\n#define RV_N %d\n" % (TS, UMAX, N))
        f.write("static const struct { float b, c; int k; float r, y, u; } RV[] = {\n")
        for b, c, k, r, y, u in rows:
            f.write("  {%.1ff, %.1ff, %d, %.7ff, %.7ff, %.6ff},\n" % (b, c, k, r, y, u))
        f.write("};\n")


if __name__ == "__main__":
    main()
