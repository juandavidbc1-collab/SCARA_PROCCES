"""Prueba en el PC de ros2/scara_common.py (formato de tópicos, cinemática y copia en Python del estimador).
   python3 tools/host_tests/test_common.py      (solo necesita numpy)"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2"))
import scara_common as sc  # noqa: E402

fallos = 0


def check(c, msg):
    global fallos
    print(("  [ok] " if c else "  >>> FALLO: ") + msg)
    fallos += 0 if c else 1


N, k = 102.08, sc.RPM
m = dict(ra=50, la=0.06, kv=1.6368711e-3 * N, b=2.679347e-7 * N * N, j=3.4858e-8 * N * N)
p = dict(polo_re=.25, polo_im=.25, sig_i=.01, sig_w=3.20694 / N, sig_y=.80956 * k, p0_i=.1, p0_w=k)
e = sc.Estimator(m, p, 0.01)
check(abs(e.L[0] - 0.24358173) < 1e-6 and abs(e.L[1] - 0.41197546) < 1e-6, "L igual a la del laboratorio y a estimator.c")
for _ in range(300):
    e.step(0.0, 0.0)
check(abs(e.K[1] - 0.26597) < 5e-4, "ganancia de Kalman estable = 0,266")
check(abs(sc.voltaje(1.0) - 11.2) < 1e-9 and sc.voltaje(0.05) == 0.0, "voltaje con zona muerta")
st = sc.parse_state([1] * 13)
check(st is not None and st["enabled"] == 0x3 and not st["nuevo"], "estado viejo de 13 datos se acepta")
st = sc.parse_state(list(range(19)))
check(st["enabled"] == 13 and st["nuevo"] and st["w"] == (16, 17, 18), "estado nuevo de 19 datos")
r = sc.parse_obs([5, 0b101011] + [float(i) for i in range(36)])
check(r[0] == 5 and r[1][1]["wL_rpm"] == 13 and r[1][0]["ok"] and not r[1][2]["ok"] and r[1][2]["med"], "formato de /scara/obs")
q = sc.ik(*sc.fk(20, 90))
check(abs(q[0] - 20) < 1e-9 and abs(q[1] - 90) < 1e-9 and sc.ik(400, 0) is None, "cinematica directa/inversa")
print("\nTODO OK" if not fallos else "\n%d FALLOS" % fallos)
sys.exit(1 if fallos else 0)
