"""Genera las figuras de la justificación (figs/f_observadores.pdf y figs/f_pid.pdf).
   python3 docs/justificacion/figuras.py      (numpy y matplotlib)"""
import os, sys, math, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "..", "ros2"))
import scara_common as sc
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": .3, "figure.dpi": 150, "axes.spines.top": False, "axes.spines.right": False})
m = sc.MOTOR; ra, la, kv, b, j = (m[k] for k in ("ra","la","kv","b","j"))
CPS = 6424.0; RAD_CNT = 2*math.pi/CPS
V0 = 0.8
def vef(duty):
    v = 12*duty; a = abs(v)-V0
    return 0.0 if a <= 0 else math.copysign(a, v)

# ---------- F2: observadores (planta = modelo identificado + zona muerta, encoder cuantizado) ----------
dt = 1e-4; T = 6.0
e = sc.Estimator(m, sc.OBS_DEFAULT, 0.01)
i = w = th = 0.0
def duty_t(t):
    if t < 0.5: return 0
    if t < 2.0: return 0.40
    if t < 3.5: return 0.70
    if t < 5.0: return -0.30
    return 0.0
rec = []
cnt_prev = 0; u_prev = 0.0
nsub = int(0.01/dt)
for k in range(int(T/0.01)):
    t = k*0.01
    # medición en t: diferencia de cuentas en los últimos 10 ms
    cnt = math.floor(th/RAD_CNT)
    y = (cnt-cnt_prev)*RAD_CNT/0.01; cnt_prev = cnt
    e.step(u_prev, y, True)
    rec.append((t, w, i, y, e.xl[1], e.xl[0], e.xp[1], e.xp[0], e.xk[1], e.xk[0], e.K[1], math.sqrt(e.Pk[1,1]), math.sqrt(e.Pk[0,0])))
    u = vef(duty_t(t))
    for _ in range(nsub):
        i += (u - ra*i - kv*w)/la*dt
        w += (kv*i - b*w)/j*dt
        th += w*dt
    u_prev = u
r = np.array(rec); R = sc.RPM
fig, ax = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True)
ax[0].plot(r[:,0], r[:,3]/R, color="#9aa3ad", lw=.8, label="medida (encoder, 10 ms)")
ax[0].plot(r[:,0], r[:,4]/R, color="#2e9e5b", lw=1.1, label="Luenberger")
ax[0].plot(r[:,0], r[:,8]/R, color="#1f77b4", lw=1.6, label="Kalman a posteriori")
ax[0].plot(r[:,0], r[:,1]/R, "k--", lw=1, label="real (simulación)")
ax[0].set_ylabel("velocidad [rpm]"); ax[0].legend(ncol=2, fontsize=7.5, loc="lower left")
ax[1].plot(r[:,0], r[:,5]*1000, color="#2e9e5b", lw=1.0, label="Luenberger")
ax[1].plot(r[:,0], r[:,9]*1000, color="#1f77b4", lw=1.6, label="Kalman a posteriori")
ax[1].plot(r[:,0], r[:,2]*1000, "k--", lw=1, label="real (simulación)")
ax[1].set_ylabel("corriente [mA]"); ax[1].set_xlabel("tiempo [s]"); ax[1].legend(fontsize=7.5, loc="lower left")
for a_, (t0, d) in zip([ax[0]]*4, ((0.5, "40 %"), (2.0, "70 %"), (3.5, "−30 %"), (5.0, "0 %"))):
    a_.annotate("PWM " + d, (t0, 0), xytext=(t0+0.05, 64), fontsize=7, color="#555")
fig.tight_layout(); fig.savefig(os.path.join(AQUI, "figs", "f_observadores.pdf")); plt.close(fig)
sel = (r[:,0] > 1.0) & (np.abs(r[:,0]-2.0) > .3) & (np.abs(r[:,0]-3.5) > .3) & (np.abs(r[:,0]-5.0) > .3)
def rms(x): return math.sqrt(np.mean(x*x))
print("RMS vel medida %.3f rpm, L %.3f, K %.3f" % (rms((r[sel,3]-r[sel,1])/R), rms((r[sel,4]-r[sel,1])/R), rms((r[sel,8]-r[sel,1])/R)))
print("RMS corr L %.2f mA, K %.2f mA" % (rms((r[sel,5]-r[sel,2])*1000), rms((r[sel,9]-r[sel,2])*1000)))
print("i real a 70%%: %.1f mA, w %.1f rpm" % (r[(r[:,0]>3.3)&(r[:,0]<3.5),2].mean()*1000, r[(r[:,0]>3.3)&(r[:,0]<3.5),1].mean()/R))

# ---------- F3: PID de posición, análisis lineal (sin saturación, rampa ni zona muerta) ----------
def sim_lin(g, step_deg=2.0, T=1.2):
    kp, ki, kd, kff, fric = g
    f = 1023/12.0
    Kp, Ki, Kd = kp/f, ki/f, kd/f
    dt = 1e-5; i = w = th = 0.0; I = 0.0; out = []; u = 0.0
    ref = math.radians(step_deg)
    for k in range(int(T/dt)):
        t = k*dt
        if k % 100 == 0:                     # lazo de 1 kHz
            e = ref - th
            I += Ki*e*1e-3
            u = Kp*e - Kd*w + I
            out.append((t, math.degrees(th), u))
        i += (u - ra*i - kv*w)/la*dt
        w += (kv*i - b*w)/j*dt
        th += w*dt
    return np.array(out)
fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.7))
for g, name, col in (((5350,18500,295,85,70), "modelo: 5350 · 18500 · 295", "#c0504d"), ((5000,3000,180,80,70), "Reporte 3: 5000 · 3000 · 180", "#1f4e79")):
    o = sim_lin(g)
    ax[0].plot(o[:,0], o[:,1], color=col, label=name)
    ax[1].plot(o[:,0], o[:,2], color=col, label=name)
    os_ = (o[:,1].max()-2)/2*100
    ts_ = o[np.where(np.abs(o[:,1]-2) > 0.04)[0][-1], 0]
    print(name, "sobrepaso %.1f %%, establecimiento 2%% %.3f s, u max %.2f V, error en 0.3 s %.3f deg" % (os_, ts_, o[:,2].max(), 2-o[300,1]))
ax[0].axhline(2, color="k", lw=.6, ls=":"); ax[0].set_ylabel("ángulo [°]"); ax[0].set_xlabel("tiempo [s]"); ax[0].legend(fontsize=7, loc="lower right")
ax[1].set_ylabel("voltaje pedido [V]"); ax[1].set_xlabel("tiempo [s]")
fig.tight_layout(); fig.savefig(os.path.join(AQUI, "figs", "f_pid.pdf")); plt.close(fig)
