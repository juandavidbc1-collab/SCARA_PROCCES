"""Figura figs/f_convergencia.pdf: reinicio de los observadores con el motor a 70 %."""
from estudio_observadores import *
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams.update({"font.size": 8.5, "axes.grid": True, "grid.alpha": .3, "axes.spines.top": False, "axes.spines.right": False})
r = run(reset_at=3.0, T=3.6)
s = (r[:,0] >= 2.9) & (r[:,0] <= 3.5)
t = (r[s,0]-3.0)*1000
fig, ax = plt.subplots(2, 2, figsize=(7.2, 4.6))
a = ax[0,0]
a.plot(t, r[s,3]/R, "o", ms=2.5, color="#9aa3ad", label="medida")
a.plot(t, r[s,4]/R, color="#2e9e5b", lw=1.2, label="Luenberger")
a.plot(t, r[s,6]/R, color="#e8893a", lw=1.2, ls="--", label="Kalman a priori")
a.plot(t, r[s,8]/R, color="#1f77b4", lw=1.6, label="Kalman a posteriori")
a.plot(t, r[s,1]/R, "k:", lw=1, label="real")
a.set_ylabel("velocidad [rpm]"); a.legend(fontsize=6.5, loc="lower right"); a.set_title("Velocidad", fontsize=9)
a = ax[0,1]
a.plot(t, r[s,5]*1000, color="#2e9e5b", lw=1.2, label="Luenberger")
a.plot(t, r[s,7]*1000, color="#e8893a", lw=1.2, ls="--", label="Kalman a priori")
a.plot(t, r[s,9]*1000, color="#1f77b4", lw=1.6, label="Kalman a posteriori")
a.plot(t, r[s,2]*1000, "k:", lw=1, label="real")
a.set_ylabel("corriente [mA]"); a.set_title("Corriente", fontsize=9); a.set_ylim(-150, 700)
a.annotate("Luenberger: pico de %.0f mA" % (r[s,5].max()*1000), (20, 690), xytext=(120, 560), fontsize=7, color="#2e9e5b", arrowprops=dict(arrowstyle="->", color="#2e9e5b", lw=.8))
a = ax[1,0]
a.plot(t, r[s,10], color="#c08a00", lw=1.6, drawstyle="steps-post")
a.set_ylabel("$K_w$"); a.set_xlabel("tiempo desde el reinicio [ms]"); a.set_title("Ganancia de Kalman (velocidad)", fontsize=9)
a = ax[1,1]
a.plot(t, r[s,12]/R, color="#6b4fc8", lw=1.6, drawstyle="steps-post", label="$\\sqrt{P_\\omega}$ [rpm]")
a2 = a.twinx(); a2.plot(t, r[s,13]*1000, color="#6b4fc8", lw=1.0, ls="--", drawstyle="steps-post"); a2.set_ylabel("$\\sqrt{P_i}$ [mA] (línea punteada)", fontsize=7.5)
a2.spines.top.set_visible(False)
a.set_ylabel("$\\sqrt{P_\\omega}$ [rpm]"); a.set_xlabel("tiempo desde el reinicio [ms]"); a.set_title("Incertidumbre del Kalman", fontsize=9)
for a in ax.flat: a.axvline(0, color="#c0504d", lw=.8, ls=":")
fig.tight_layout(); fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs", "f_convergencia.pdf"))
# números de convergencia
k0 = np.argmin(np.abs(r[:,0]-3.0))
for name, col in (("L", 4), ("K", 8)):
    err = np.abs(r[k0:,col]-r[k0:,1])/R
    n = next(k for k in range(len(err)) if all(err[k:k+15] < 1.0))
    print(name, "vel dentro de 1 rpm tras", n, "muestras")
for name, col in (("L", 5), ("K", 9)):
    err = np.abs(r[k0:,col]-r[k0:,2])*1000
    n = next(k for k in range(len(err)) if all(err[k:k+15] < 20))
    print(name, "corriente dentro de 20 mA tras", n, "muestras")
print("real a 70%%: %.1f rpm %.1f mA" % (r[k0,1]/R, r[k0,2]*1000))
