"""Pruebas de la lógica pura del enlace con el robot (sin ROS ni ventana).  python3 -m unittest tools/host_tests/test_link.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2"))
import scara_link as sl  # noqa: E402


class TestDeadman(unittest.TestCase):
    def setUp(self):
        self.d = sl.Deadman(gui_timeout=0.15, release_repeats=3)
        self.d.gui_beat(10.0)

    def test_idle_sends_nothing(self):
        self.assertIsNone(self.d.next_message(10.01))

    def test_arrows_without_space_do_not_arm(self):
        self.d.set_keys({"left"})
        self.assertIsNone(self.d.next_message(10.01))

    def test_space_and_arrows_give_direction(self):
        self.d.set_keys({"space", "right", "down"})
        self.assertEqual(self.d.next_message(10.01), (1.0, 1.0, -1.0))

    def test_opposite_arrows_cancel(self):
        self.d.set_keys({"space", "left", "right"})
        self.assertEqual(self.d.next_message(10.01), (1.0, 0.0, 0.0))

    def test_release_repeats_then_goes_quiet(self):
        self.d.set_keys({"space", "up"})
        self.d.next_message(10.01)
        self.d.set_keys(set())
        msgs = [self.d.next_message(10.02 + i * 0.001) for i in range(4)]
        self.assertEqual(msgs, [(0.0, 0.0, 0.0)] * 3 + [None])

    def test_frozen_gui_releases_deadman_even_with_keys_held(self):
        self.d.set_keys({"space", "up"})
        self.assertEqual(self.d.next_message(10.1), (1.0, 0.0, 1.0))
        self.assertEqual(self.d.next_message(10.2), (0.0, 0.0, 0.0), "la interfaz lleva 200 ms sin responder")

    def test_gui_recovers(self):
        self.d.set_keys({"space", "up"})
        self.d.next_message(10.2)
        self.d.gui_beat(10.3)
        self.assertEqual(self.d.next_message(10.31), (1.0, 0.0, 1.0))


class TestState(unittest.TestCase):
    BASE = [10.0, 20.0, 100.0, 50.0, 2.0, 0.0, 1.0, 1.0, 5.0, 11.0, 21.0, 100.0, -50.0, 5.0, 1.0, 320.0]

    def test_decode(self):
        s = sl.decode_state(self.BASE)
        self.assertEqual((s.q1, s.q2, s.x, s.y), (10.0, 20.0, 100.0, 50.0))
        self.assertEqual(s.mode, "HOLD")
        self.assertEqual((s.j1_on, s.j2_on, s.m3_on), (True, False, True))
        self.assertEqual(s.ctrl, "RST posicional")
        self.assertEqual((s.fc_j1_min, s.fc_j1_max, s.fc_j2_min, s.fc_j2_max), (True, False, True, False))
        self.assertEqual(s.tool, 320.0)

    def test_estop_label(self):
        d = list(self.BASE)
        d[4], d[5] = 6.0, 6.0
        s = sl.decode_state(d)
        self.assertTrue(s.estop)
        self.assertEqual(s.mode, "PARO")

    def test_other_fault_is_not_estop(self):
        d = list(self.BASE)
        d[4], d[5] = 6.0, 2.0
        s = sl.decode_state(d)
        self.assertFalse(s.estop)
        self.assertEqual(s.mode, "FALLA")
        self.assertIn("trabado", s.fault_text)

    def test_old_firmware_without_new_fields_still_decodes(self):
        s = sl.decode_state(self.BASE[:13])
        self.assertEqual(s.mode, "HOLD")
        self.assertTrue(s.j1_on and s.j2_on)
        self.assertEqual(s.ctrl, "?")

    def test_screw_fields(self):
        d = [0.0] * 16 + [1.2, 0.4, 600.0, 38.0, 12.1, 6.05, 90.0, 3.0]
        s = sl.decode_state(d)
        self.assertEqual((s.tool_i, s.tool_torque, s.tool_fuerza, s.tool_presion), (1.2, 0.4, 600.0, 38.0))
        self.assertEqual((s.tool_vueltas, s.tool_prof, s.servo, s.screw_fase), (12.1, 6.05, 90.0, "Apriete"))

    def test_original_pid_form_has_a_name(self):
        d = [0.0] * 16
        d[14] = 4.0
        self.assertEqual(sl.decode_state(d).ctrl, "RST enmascarado")

    def test_short_message_returns_none(self):
        self.assertIsNone(sl.decode_state([1.0, 2.0]))

    def test_unknown_mode_does_not_crash(self):
        d = list(self.BASE)
        d[4] = 99.0
        self.assertEqual(sl.decode_state(d).mode, "?")


class TestParse(unittest.TestCase):
    def test_gain(self):
        g = sl.parse_gains("#INFO: J1 kp=5000 ki=3000 kd=180 kff=80 fric=70 | J2 kp=1 ki=2 kd=3 kff=4 fric=5")
        self.assertEqual(g[2], [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_rst(self):
        g = sl.parse_rst("#RST: J1 kp=1.0 ki=2.0 kd=3.00 s1=-0.3000 b=1.00 c=0.00 kff=85.5 | J2 kp=4.0 ki=5.0 kd=6.00 s1=-0.2000 b=1.00 c=1.00 kff=85.5")
        self.assertEqual(g[1], [1.0, 2.0, 3.0, -0.3, 1.0, 0.0, 85.5])
        self.assertEqual(g[2][3], -0.2)

    def test_other_text_gives_nothing(self):
        self.assertEqual(sl.parse_gains("#OK: nada"), {})
        self.assertEqual(sl.parse_rst("#OK: nada"), {})


if __name__ == "__main__":
    unittest.main()
