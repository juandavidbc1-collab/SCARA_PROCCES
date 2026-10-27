"""Robustez del Kalman ante una planta distinta al modelo."""
from estudio_observadores import *
for name, pl, tc in (("J+50%", {"j": m["j"]*1.5}, 0), ("friccion seca extra (=0.5 V)", {}, 0.0017), ("Ra 40", {"ra": 40.0}, 0)):
    for sw in (0.30, 1.0, 3.0):
        par = dict(sc.OBS_DEFAULT); par["sig_w"] = sw*R
        r = run(par, plant=pl, tc=tc); mt = metrics(r)
        print("%-30s sw=%.1f  vel: medida %.2f L %.2f K %.2f rpm | corriente: L %.1f (sesgo %.1f)  K %.1f (sesgo %.1f) mA" % (name, sw, mt["vm"], mt["vL"], mt["vK"], mt["iL"], mt["biasL"], mt["iK"], mt["biasK"]))
