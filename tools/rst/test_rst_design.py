"""Pruebas del diseño RST. Ejecutar:  python3 -m unittest tools/rst/test_rst_design.py"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rst_design as rd  # noqa: E402

TS = 0.01
K, TAU = 0.9974, 0.14


class TestDesign(unittest.TestCase):
    def setUp(self):
        self.A, self.B = rd.plant(K, TAU, TS)
        self.d = rd.design_joint(K, TAU, TS, wn=12.0, zeta=0.9, aux=(40.0, 60.0))

    def test_plant_has_integrator_and_delay(self):
        self.assertAlmostEqual(np.polyval(self.A[::-1], 1.0), 0.0, places=12)
        self.assertEqual(self.B[0], 0.0)

    def test_closed_loop_poles_are_requested(self):
        got = np.sort_complex(np.roots(rd.closed_loop(self.A, self.B, self.d["R"], self.d["S"])))
        want = np.sort_complex(np.array(self.d["poles"]))
        np.testing.assert_allclose(got, want, atol=1e-6)

    def test_s_has_integrator_and_stable_filter(self):
        self.assertAlmostEqual(np.sum(self.d["S"]), 0.0, places=12)
        self.assertLess(abs(self.d["s1"]), 1.0)

    def test_every_form_has_unit_dc_gain(self):
        for name, (b, c) in rd.FORMS.items():
            R, _, T = rd.polys(self.d["kp"], self.d["ki"], self.d["kd"], self.d["s1"], b, c, TS)
            self.assertAlmostEqual(np.sum(T) / np.sum(R), 1.0, places=9, msg=name)

    def test_polys_rebuilds_diophantine_solution(self):
        R, S, T = rd.polys(self.d["kp"], self.d["ki"], self.d["kd"], self.d["s1"], 1.0, 1.0, TS)
        np.testing.assert_allclose(R, self.d["R"], rtol=1e-9)
        np.testing.assert_allclose(S, self.d["S"], atol=1e-12)
        np.testing.assert_allclose(T[:3], R, rtol=1e-12)
        self.assertEqual(T[3], 0.0)

    def test_velocity_feedforward_is_in_T_and_keeps_dc_gain(self):
        d = self.d
        R, S, T0 = rd.polys(d["kp"], d["ki"], d["kd"], d["s1"], 1.0, 1.0, TS)
        _, _, T1 = rd.polys(d["kp"], d["ki"], d["kd"], d["s1"], 1.0, 1.0, TS, kff=85.0)
        self.assertAlmostEqual(np.sum(T1) / np.sum(R), 1.0, places=9)
        np.testing.assert_allclose(T1 - T0, 85.0 / TS * np.convolve(S, [1.0, -1.0]), atol=1e-9)

    def test_controller_with_feedforward_outputs_kff_times_speed_when_tracking_perfectly(self):
        d = self.d
        ctl = rd.RstController(d["kp"], d["ki"], d["kd"], d["s1"], 1.0, 1.0, TS, kff=85.0)
        u = 0.0
        for k in range(200):
            r = 0.5 * k * TS                     # rampa de 0.5 rad/s seguida sin error
            u = ctl.step(r, r, 1000.0)
        self.assertAlmostEqual(u, 85.0 * 0.5, places=6)

    def test_forms_differ_only_in_T(self):
        Ts = {n: rd.polys(self.d["kp"], self.d["ki"], self.d["kd"], self.d["s1"], b, c, TS)
              for n, (b, c) in rd.FORMS.items()}
        np.testing.assert_allclose(Ts["pos"][0], Ts["inc"][0])
        np.testing.assert_allclose(Ts["pos"][1], Ts["pi_d"][1])
        self.assertFalse(np.allclose(Ts["pos"][2], Ts["inc"][2]))
        self.assertFalse(np.allclose(Ts["pos"][2], Ts["pi_d"][2]))

    def test_original_pid_becomes_an_exact_rst(self):
        """Mismo PID que src/pid.c: filtro de velocidad de 10 ms a 1 kHz; la consigna de velocidad entra sin filtrar."""
        kp, ki, kd, s1, kff = rd.pid_as_rst(5000.0, 3000.0, 180.0, 80.0, 0.010, 0.001)
        self.assertAlmostEqual(s1, -0.010 / 0.011, places=9)
        self.assertAlmostEqual(kd, 180.0 * 0.001 / 0.011, places=9)
        self.assertEqual((kp, ki, kff), (5000.0, 3000.0, 260.0))
        # Referencia: la misma ley escrita como PID, con entradas cualquiera.
        ctl = rd.RstController(kp, ki, kd, s1, *rd.FORMS["mask"], 0.001, kff=kff)
        i = qd = y0 = r0 = 0.0
        for k in range(300):
            r = 0.002 * k if k > 20 else 0.0
            y = 0.9 * r + 0.001 * ((k * 7) % 5)
            qd += 0.001 / 0.011 * ((y - y0) / 0.001 - qd)
            i += 3000.0 * (r - y) * 0.001
            want = 80.0 * (r - r0) / 0.001 + 5000.0 * (r - y) + 180.0 * ((r - r0) / 0.001 - qd) + i
            y0, r0 = y, r
            self.assertAlmostEqual(ctl.step(r, y, 1e6), want, places=5, msg="k=%d" % k)

    def test_gains_are_positive(self):
        for k in ("kp", "ki", "kd"):
            self.assertGreater(self.d[k], 0.0, msg=k)


if __name__ == "__main__":
    unittest.main()
