"""Estudio de los observadores (polos, Q y R, planta distinta al modelo). Lo usan estudio2 y figura_convergencia.
   python3 docs/justificacion/estudio_observadores.py"""
import os, sys, math, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2"))
import scara_common as sc
m = sc.MOTOR; R = sc.RPM
CPS = 6424.0; RC = 2*math.pi/CPS
def vef(d, v0=0.8):
    v = 12*d; a = abs(v)-v0
    return 0.0 if a <= 0 else math.copysign(a, v)
def duty_t(t):
    if t < 0.5: return 0
    if t < 2.0: return 0.40
    if t < 3.5: return 0.70
    if t < 5.0: return -0.30
    return 0.0
def run(params=sc.OBS_DEFAULT, plant=None, T=6.0, reset_at=None, duty=duty_t, tc=0.0):
    p = dict(m); p.update(plant or {})
    ra, la, kv, b, j = (p[k] for k in ("ra","la","kv","b","j"))
    e = sc.Estimator(m, params, 0.01)
    i = w = th = 0.0; cnt_prev = 0; u_prev = 0.0; dt = 1e-4; out = []
    for k in range(int(T/0.01)):
        t = k*0.01
        if reset_at is not None and abs(t-reset_at) < 1e-9: e.reset()
        cnt = math.floor(th/RC); y = (cnt-cnt_prev)*RC/0.01; cnt_prev = cnt
        e.step(u_prev, y, True)
        out.append((t, w, i, y, e.xl[1], e.xl[0], e.xp[1], e.xp[0], e.xk[1], e.xk[0], e.K[1], e.K[0], math.sqrt(e.Pk[1,1]), math.sqrt(e.Pk[0,0])))
        u = vef(duty(t))
        for _ in range(100):
            i += (u - ra*i - kv*w)/la*dt
            tl = tc*math.tanh(w/0.05)
            w += (kv*i - b*w - tl)/j*dt
            th += w*dt
        u_prev = u
    return np.array(out)
def steady_mask(r):
    t = r[:,0]
    return (t > 1.0) & (np.abs(t-2.0) > .3) & (np.abs(t-3.5) > .3) & (np.abs(t-5.0) > .3) & (t < 5.0)
def rms(x): return math.sqrt(np.mean(np.asarray(x)**2))
def metrics(r):
    s = steady_mask(r)
    return dict(vm=rms((r[s,3]-r[s,1])/R), vL=rms((r[s,4]-r[s,1])/R), vK=rms((r[s,8]-r[s,1])/R),
                iL=rms((r[s,5]-r[s,2])*1000), iK=rms((r[s,9]-r[s,2])*1000),
                biasK=np.mean((r[s,9]-r[s,2])*1000), biasL=np.mean((r[s,5]-r[s,2])*1000))
if __name__ == "__main__":
    print("== polos Luenberger ==")
    for pr, pi in ((0.05, 0.0), (0.25, 0.25), (0.5, 0.0), (0.7, 0.0), (0.85, 0.0)):
        par = dict(sc.OBS_DEFAULT); par["polo_re"], par["polo_im"] = pr, pi
        e = sc.Estimator(m, par, 0.01); r = run(par); mt = metrics(r)
        print("%.2f±%.2fj L=[%.4f %.4f] |z|=%.3f velL %.3f rpm  iL %.2f mA" % (pr, pi, e.L[0], e.L[1], math.hypot(pr, pi), mt["vL"], mt["iL"]))
    print("== Kalman Q,R ==")
    for si, sw, sy in ((0.010, 0.30, 0.81), (0.001, 0.30, 0.81), (0.100, 0.30, 0.81), (0.010, 0.10, 0.81), (0.010, 1.0, 0.81), (0.010, 0.30, 0.27), (0.010, 0.30, 2.5)):
        par = dict(sc.OBS_DEFAULT); par.update(sig_i=si, sig_w=sw*R, sig_y=sy*R)
        r = run(par); mt = metrics(r)
        print("si %5.1f mA sw %.2f sy %.2f rpm -> Kw %.3f Ki %.2e  velK %.3f  iK %.2f mA" % (si*1000, sw, sy, r[-1,10], r[-1,11], mt["vK"], mt["iK"]))
    print("== planta distinta ==")
    for name, pl, tc in (("igual", {}, 0), ("J+50%", {"j": m["j"]*1.5}, 0), ("Ra 25", {"ra": 25.0}, 0), ("friccion seca 0.01 Nm", {}, 0.01)):
        r = run(plant=pl, tc=tc); mt = metrics(r)
        print(name, "velL %.3f velK %.3f | iL rms %.1f sesgo %.1f | iK rms %.1f sesgo %.1f mA" % (mt["vL"], mt["vK"], mt["iL"], mt["biasL"], mt["iK"], mt["biasK"]))
    print("== convergencia K ==")
    e = sc.Estimator(m, sc.OBS_DEFAULT, 0.01); Ks = []
    for k in range(60): e.step(0, 0); Ks.append(e.K[1])
    Kss = Ks[-1]; n = next(k for k in range(60) if all(abs(x-Kss) < 0.01*Kss for x in Ks[k:]))
    print("K pasos:", [round(x,3) for x in Ks[:8]], "llega al 1% en muestra", n+1)
