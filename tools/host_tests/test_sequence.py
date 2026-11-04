import math
"""Pruebas de la secuencia repetitiva, sin ROS.  Ejecutar:  python3 -m unittest tools/host_tests/test_sequence.py"""
import os
import sys
import tempfile
import textwrap
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2"))
import scara_sequence as sq  # noqa: E402

ARRIVED = "#LLEGO: objetivo x=1 y=2"


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make(steps, repeat=1, timeout=20.0):
    sent, clock = [], Clock()
    return sq.SequenceRunner(steps, repeat, sent.append, clock, step_timeout=timeout), sent, clock


GOTO_A = {"kind": "goto", "x": 120.0, "y": 80.0}
GOTO_B = {"kind": "goto", "x": 100.0, "y": -60.0}


class TestRunner(unittest.TestCase):
    def test_runs_steps_in_order_and_waits_for_arrival(self):
        r, sent, _ = make([GOTO_A, GOTO_B])
        r.start()
        self.assertEqual(sent, ["goto 120 80"])
        r.tick()
        self.assertEqual(sent, ["goto 120 80"], "no manda el siguiente punto antes de llegar")
        r.on_event(ARRIVED)
        r.tick()
        self.assertEqual(sent, ["goto 120 80", "goto 100 -60"])
        r.on_event(ARRIVED)
        r.tick()
        self.assertEqual(r.state, "done")

    def test_repeats_the_requested_number_of_cycles(self):
        r, sent, _ = make([GOTO_A], repeat=3)
        r.start()
        for _ in range(3):
            r.on_event(ARRIVED)
            r.tick()
        self.assertEqual(len(sent), 3)
        self.assertEqual(r.state, "done")
        self.assertEqual(r.cycles_done, 3)

    def test_zero_repeat_runs_until_stopped(self):
        r, sent, _ = make([GOTO_A], repeat=0)
        r.start()
        for _ in range(50):
            r.on_event(ARRIVED)
            r.tick()
        self.assertEqual(r.state, "running")
        self.assertEqual(len(sent), 51)
        r.stop()
        self.assertEqual(r.state, "stopped")
        self.assertEqual(sent[-1], "stop")

    def test_wait_step_uses_the_clock(self):
        r, sent, clk = make([{"kind": "wait", "secs": 2.0}, GOTO_A])
        r.start()
        clk.t += 1.9
        r.tick()
        self.assertEqual(sent, [])
        clk.t += 0.2
        r.tick()
        self.assertEqual(sent, ["goto 120 80"])

    def test_tool_step_runs_for_its_time_renews_and_stops(self):
        r, sent, clk = make([{"kind": "tool", "pct": 40.0, "secs": 12.0}])
        r.start()
        self.assertEqual(sent, ["tool 40"])
        clk.t += sq.TOOL_RENEW_S + 0.1
        r.tick()
        self.assertEqual(sent, ["tool 40", "tool 40"], "renueva antes de que M3 se apague solo")
        clk.t += 12.0
        r.tick()
        self.assertEqual(sent[-1], "tool 0")
        self.assertEqual(r.state, "done")

    def test_fault_aborts_and_sends_nothing_more(self):
        r, sent, _ = make([GOTO_A, GOTO_B])
        r.start()
        r.on_event("#FALLA: motor trabado")
        for _ in range(5):
            r.on_event(ARRIVED)
            r.tick()
        self.assertEqual(r.state, "aborted")
        self.assertEqual(sent, ["goto 120 80"])
        self.assertIn("trabado", r.reason)

    def test_rejected_point_aborts(self):
        r, sent, _ = make([GOTO_A, GOTO_B])
        r.start()
        r.on_event("#ERROR: no se puede ir a (120, 80)")
        r.tick()
        self.assertEqual(r.state, "aborted")
        self.assertEqual(sent, ["goto 120 80"])

    def test_timeout_aborts_and_stops_the_robot(self):
        r, sent, clk = make([GOTO_A], timeout=5.0)
        r.start()
        clk.t += 5.1
        r.tick()
        self.assertEqual(r.state, "aborted")
        self.assertEqual(sent[-1], "stop")
        self.assertIn("tiempo", r.reason)

    def test_abort_during_tool_step_turns_the_tool_off(self):
        r, sent, _ = make([{"kind": "tool", "pct": 40.0, "secs": 5.0}])
        r.start()
        r.on_event("#ERROR: algo")
        self.assertEqual(sent[-1], "tool 0")

    def test_old_arrival_before_start_is_ignored(self):
        r, sent, _ = make([GOTO_A, GOTO_B])
        r.on_event(ARRIVED)
        r.start()
        r.tick()
        self.assertEqual(sent, ["goto 120 80"])

    def test_stop_when_idle_sends_nothing(self):
        r, sent, _ = make([GOTO_A])
        r.stop()
        self.assertEqual(sent, [])

    def test_cycle_time_is_recorded(self):
        r, _, clk = make([GOTO_A], repeat=2)
        r.start()
        clk.t += 3.0
        r.on_event(ARRIVED)
        r.tick()
        self.assertEqual(r.cycle_times, [3.0])


class TestLoad(unittest.TestCase):
    def load(self, text):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(textwrap.dedent(text))
        self.addCleanup(os.unlink, f.name)
        return sq.load_sequence(f.name)

    def test_valid_file(self):
        steps, repeat = self.load("""
            repeticiones: 4
            pasos:
              - ir: [120, 80]
              - esperar: 0.5
              - angulos: [20, 90]
              - m3: {potencia: 40, tiempo: 1.5}
        """)
        self.assertEqual(repeat, 4)
        self.assertEqual([s["kind"] for s in steps], ["goto", "wait", "ang", "tool"])
        self.assertEqual(steps[3]["pct"], 40)

    def test_unreachable_point_names_the_step(self):
        with self.assertRaisesRegex(sq.SequenceError, "paso 2.*alcance"):
            self.load("""
                pasos:
                  - ir: [120, 80]
                  - ir: [400, 0]
            """)

    def test_singular_point_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "singular"):
            self.load("pasos:\n  - ir: [230, 0]\n")

    def test_unknown_step_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "bailar"):
            self.load("pasos:\n  - bailar: 3\n")

    def test_empty_sequence_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "pasos"):
            self.load("repeticiones: 1\npasos: []\n")

    def test_tool_power_out_of_range_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "potencia"):
            self.load("pasos:\n  - m3: {potencia: 150, tiempo: 1}\n")

    def test_negative_repeat_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "repeticiones"):
            self.load("repeticiones: -1\npasos:\n  - esperar: 1\n")

    def test_bad_yaml_is_reported_not_raised_raw(self):
        with self.assertRaises(sq.SequenceError):
            self.load("pasos: [\n")

    def test_shipped_example_is_valid(self):
        path = os.path.join(os.path.dirname(os.path.abspath(sq.__file__)), "secuencia.yaml")
        steps, _ = sq.load_sequence(path)
        self.assertGreaterEqual(len(steps), 2)


ROS2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2")
PROCESO = """
repeticiones: 2
proceso:
  base_cm: [0, 0]
  codo: -1
  placas_cm: {A: [17, 10.2], D: [17, -10.2]}
  lado_mm: 38
  hueco_borde_mm: 3.5
  servo: {arriba_s: 1.0, bajar_s: 1.0}
  m3: {abajo_vueltas: 24.855, trabajo_vueltas: 18.641}
"""


class TestProcess(unittest.TestCase):
    def _load(self, text, **kw):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(text)
        self.addCleanup(os.unlink, f.name)
        return sq.load_sequence_full(f.name, **kw)

    def _assigned(self, **kw):
        return sq.load_sequence_full(os.path.join(ROS2, "secuencia.yaml"), **kw)

    def test_assigned_file_has_sixteen_holes_on_four_boards(self):
        seq = self._assigned()
        self.assertEqual(len(seq.process.holes), 16)
        self.assertEqual(sorted(seq.process.boards), ["A", "B", "C", "D"])
        self.assertEqual(seq.repeat, 1)

    def test_boards_are_in_a_straight_row_two_on_each_side(self):
        b = self._assigned().process.boards
        self.assertEqual(sorted({round(x, 6) for x, _ in b.values()}), [170.0])
        self.assertAlmostEqual(b["A"][1], 102.0, places=6)
        self.assertAlmostEqual(b["B"][1], 34.0, places=6)
        self.assertAlmostEqual(b["C"][1], -34.0, places=6)
        self.assertAlmostEqual(b["D"][1], -102.0, places=6)

    def test_holes_are_at_the_measured_offset_from_the_board_center(self):
        h = {x["label"]: x for x in self._load(PROCESO).process.holes}
        self.assertAlmostEqual(h["A-H2"]["x"], 170 - 15.5, places=3)
        self.assertAlmostEqual(h["A-H2"]["y"], 102 + 15.5, places=3)
        self.assertAlmostEqual(h["A-H1"]["y"], 102 - 15.5, places=3)

    def test_hole_order_is_top_row_then_bottom_row(self):
        names = [h["label"] for h in self._load(PROCESO).process.holes]
        self.assertEqual(names[:4], ["A-H2", "A-H4", "A-H3", "A-H1"])

    def test_every_hole_follows_the_robot_sequence(self):
        seq = self._assigned()
        i = next(k for k, s in enumerate(seq.steps) if s["kind"] == "goto")
        un_hueco = seq.steps[i:i + 9]
        self.assertEqual([s["kind"] for s in un_hueco], ["goto", "wait", "cmd", "wait", "m3", "wait", "cmd", "wait", "m3"])
        self.assertEqual(un_hueco[2]["text"], "servo subir")
        self.assertEqual(un_hueco[4]["vueltas"], 24.855)
        self.assertEqual(un_hueco[6]["text"], "servo bajar")
        self.assertEqual(un_hueco[8]["vueltas"], 18.641)
        self.assertEqual(un_hueco[8]["fin"], "hecho")

    def test_servo_only_alternates_full_up_and_full_down(self):
        seq = self._assigned()
        servos = [s["text"] for s in seq.steps if s["kind"] == "cmd" and s["text"].startswith("servo")]
        self.assertEqual(servos, ["servo subir", "servo bajar"] * 16)

    def test_process_ends_at_zero_zero(self):
        last = self._assigned().steps[-1]
        self.assertEqual((last["kind"], last["a"], last["b"]), ("ang", 0.0, 0.0))

    def test_all_holes_fit_the_measured_limits(self):
        seq = self._assigned()
        for h in seq.process.holes:
            self.assertIsNone(sq.point_problem(h["x"], h["y"], elbow=seq.process.elbow), h["label"])

    def test_choosing_a_pcb_keeps_only_its_holes(self):
        seq = self._assigned(pcb=3)
        self.assertEqual([h["label"] for h in seq.process.holes], ["C-H2", "C-H4", "C-H3", "C-H1"])
        self.assertEqual(sorted(seq.process.boards), ["A", "B", "C", "D"], "se siguen dibujando todas")
        self.assertEqual(seq.steps[-1]["kind"], "ang")

    def test_pcb_out_of_range_is_rejected(self):
        with self.assertRaises(sq.SequenceError) as e:
            self._assigned(pcb=5)
        self.assertIn("PCB", str(e.exception))

    def test_unreachable_hole_is_rejected_with_its_name(self):
        with self.assertRaises(sq.SequenceError) as e:
            self._load(PROCESO.replace("A: [17, 10.2]", "A: [30, 10.2]"))
        self.assertIn("A-H", str(e.exception))

    def test_joint_limits_are_checked_for_the_requested_elbow(self):
        with self.assertRaises(sq.SequenceError) as e:
            self._load(PROCESO.replace("codo: -1", "codo: 1").replace("D: [17, -10.2]", "D: [10, -17]"))
        self.assertIn("codo", str(e.exception))

    def test_m3_cannot_go_below_its_measured_limit(self):
        with self.assertRaises(sq.SequenceError) as e:
            self._load(PROCESO.replace("abajo_vueltas: 24.855", "abajo_vueltas: 26"))
        self.assertIn("m3", str(e.exception))
        with self.assertRaises(sq.SequenceError):
            self._load(PROCESO.replace("trabajo_vueltas: 18.641", "trabajo_vueltas: 25"))

    def test_servo_goes_from_minus_180_to_180(self):
        self.assertEqual(sq._parse_step({"servo": {"accion": -90}})["text"], "servo -90")
        for bad in (200, -200, "abrir"):
            with self.assertRaises(ValueError):
                sq._parse_step({"servo": {"accion": bad}})

    def test_old_format_still_loads(self):
        steps, repeat = sq.load_sequence(self._write_old())
        self.assertEqual(repeat, 3)
        self.assertEqual(steps[0]["kind"], "goto")

    def _write_old(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("repeticiones: 3\npasos:\n  - ir: [120, 80]\n  - servo: {accion: subir, tiempo: 0.5}\n  - atornillar: {vueltas: 8, torque: 0.3}\n  - codo: -1\n  - m3_ir: 18.6\n")
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_step_forms_in_pasos(self):
        steps, _ = sq.load_sequence(self._write_old())
        self.assertEqual(steps[1], {"kind": "cmd", "text": "servo subir", "secs": 0.5, "desc": "Servo: subir"})
        self.assertEqual((steps[2]["kind"], steps[2]["vueltas"], steps[2]["torque"]), ("screw", 8.0, 0.3))
        self.assertEqual(steps[3]["text"], "elbow -1")
        self.assertEqual((steps[4]["kind"], steps[4]["vueltas"]), ("m3", 18.6))

    def test_m3_ir_out_of_range_is_rejected(self):
        with self.assertRaises(ValueError):
            sq._parse_step({"m3_ir": 30})


class TestRehearsal(unittest.TestCase):
    def _load(self, ensayo=True, pcb=None):
        return sq.load_sequence_full(os.path.join(ROS2, "secuencia.yaml"), ensayo=ensayo, pcb=pcb)

    def test_visits_the_same_holes_in_the_same_order(self):
        goto = lambda st: [(s["x"], s["y"]) for s in st if s["kind"] == "goto"]
        self.assertEqual(goto(self._load().steps), goto(self._load(ensayo=False).steps))

    def test_never_touches_the_servo(self):
        self.assertFalse([s for s in self._load().steps if s["kind"] == "cmd" and "servo" in s["text"]])

    def test_m3_goes_as_low_as_in_the_process(self):
        seq = self._load(pcb=1)
        m3 = [s for s in seq.steps if s["kind"] == "m3"]
        self.assertEqual([s["vueltas"] for s in m3[:2]], [24.855, 18.641])
        self.assertEqual(m3[1]["fin"], "ensayado")

    def test_pcb_can_be_chosen(self):
        self.assertEqual(len(self._load(pcb=4).process.holes), 4)


class TestM3Runner(unittest.TestCase):
    def setUp(self):
        self.t, self.sent = 0.0, []
        steps = [sq._m3_ir(24.855, "b", "A-H2"), dict(sq._m3_ir(18.641, "s", "A-H2"), fin="hecho"), sq._wait(1.0, "w")]
        self.r = sq.SequenceRunner(steps, 1, self.sent.append, lambda: self.t)
        self.r.start()

    def test_sends_the_height_and_waits_for_the_robot(self):
        self.assertEqual(self.sent, ["m3 ir 24.855"])
        self.t = 30.0
        self.r.tick()
        self.assertEqual(self.r.index, 0)
        self.r.on_event("#M3: en 24.76 vueltas, altura 6.4 mm.")
        self.r.tick()
        self.assertEqual(self.sent[-1], "m3 ir 18.641")

    def test_records_the_hole_when_m3_comes_back(self):
        self.r.on_event("#M3: en 24.76 vueltas")
        self.r.tick()
        self.assertEqual(self.r.results, [])
        self.r.on_event("#M3: en 18.60 vueltas")
        self.r.tick()
        self.assertEqual(self.r.results, [{"hueco": "A-H2", "resultado": "hecho", "ciclo": 1}])

    def test_times_out_if_m3_never_arrives(self):
        self.t = sq.M3_TIMEOUT_S + 1.0
        self.r.tick()
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(self.sent[-2:], ["tool 0", "stop"])

    def test_stopping_with_m3_down_warns_and_stops_it(self):
        self.r.stop()
        self.assertTrue(self.r.tool_left_down)
        self.assertEqual(self.sent[-2:], ["tool 0", "stop"])

    def test_a_blocked_m3_aborts(self):
        self.r.on_event("#ERROR: M3 no avanza: puede estar trabado o tocando algo.")
        self.assertEqual(self.r.state, "aborted")
        self.assertIn("no avanza", self.r.reason)


TORNILLO_OK = "#TORNILLO: ok vueltas=12.18 prof=6.09mm avance=101% torque=0.382Nm fuerza=587N presion=37.4MPa corriente=1.18A t=7.9s"


class TestScrewRunner(unittest.TestCase):
    def setUp(self):
        self.t = 0.0
        self.sent = []
        self.steps = [{"kind": "cmd", "text": "servo subir", "secs": 0.5, "desc": "s"},
                      {"kind": "screw", "vueltas": 12.0, "torque": 0.4, "label": "A-H1", "desc": "s"},
                      {"kind": "screw", "vueltas": 12.0, "torque": 0.4, "label": "A-H3", "desc": "s"}]
        self.r = sq.SequenceRunner(self.steps, 1, self.sent.append, lambda: self.t)

    def test_parse(self):
        d = sq.parse_tornillo(TORNILLO_OK)
        self.assertEqual(d["resultado"], "ok")
        self.assertEqual((d["vueltas"], d["prof"], d["avance"], d["torque"], d["fuerza"], d["presion"], d["corriente"], d["t"]),
                         (12.18, 6.09, 101.0, 0.382, 587.0, 37.4, 1.18, 7.9))
        self.assertIsNone(sq.parse_tornillo("#LLEGO: q1=1"))

    def test_command_step_waits_its_time(self):
        self.r.start()
        self.assertEqual(self.sent, ["servo subir"])
        self.t = 0.4; self.r.tick()
        self.assertEqual(self.r.index, 0)
        self.t = 0.5; self.r.tick()
        self.assertEqual(self.r.index, 1)
        self.assertEqual(self.sent[-1], "screw 12 0.4")

    def test_screw_ok_records_result_and_continues(self):
        self.r.start(); self.t = 1.0; self.r.tick()
        self.r.on_event(TORNILLO_OK); self.r.tick()
        self.assertEqual(self.r.index, 2)
        self.assertEqual(self.r.results[0]["hueco"], "A-H1")
        self.assertEqual(self.r.results[0]["ciclo"], 1)
        self.assertEqual(self.r.results[0]["torque"], 0.382)
        self.r.on_event(TORNILLO_OK); self.r.tick()
        self.assertEqual(self.r.state, "done")
        self.assertEqual([x["hueco"] for x in self.r.results], ["A-H1", "A-H3"])

    def test_bad_screw_aborts_and_keeps_the_record(self):
        self.r = sq.SequenceRunner(self.steps[1:], 1, self.sent.append, lambda: self.t)
        self.r.start(); self.t = 1.0; self.r.tick()
        n = len(self.sent)
        self.r.on_event(TORNILLO_OK.replace(": ok", ": sin_apriete")); self.r.tick()
        self.assertEqual(self.r.state, "aborted")
        self.assertIn("A-H1", self.r.reason)
        self.assertIn("sin_apriete", self.r.reason)
        self.assertEqual(self.r.results[0]["resultado"], "sin_apriete")
        self.assertEqual(len(self.sent), n, "no manda mas ordenes")

    def test_screw_timeout_aborts_and_stops_m3(self):
        self.r.start(); self.t = 1.0; self.r.tick()
        self.t = 1.0 + sq.STEP_TIMEOUT_S + 1; self.r.tick()
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(self.sent[-1], "stop")

    def test_stop_in_servo_step_lowers_driver(self):
        self.r.start()
        self.r.stop()
        self.assertEqual(self.sent, ["servo subir", "servo bajar", "stop"])

    def test_fault_during_screw_aborts_without_commands(self):
        self.r = sq.SequenceRunner(self.steps[1:], 1, self.sent.append, lambda: self.t)
        self.r.start(); self.t = 1.0; self.r.tick()
        n = len(self.sent)
        self.r.on_event("#FALLA: PARO DE EMERGENCIA.")
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(len(self.sent), n)


class TestServoBaja(unittest.TestCase):
    """El servo sube en un paso de 0 s y lo que sigue es la espera: cortar ahí no debe dejar el destornillador arriba."""
    def setUp(self):
        self.t = 0.0
        self.sent = []
        steps = [sq._cmd("servo subir", "subir"), sq._wait(0.4, "cayendo"), sq._cmd("servo bajar", "bajar"), sq._wait(0.6, "cayendo")]
        self.r = sq.SequenceRunner(steps, 1, self.sent.append, lambda: self.t)
        self.r.start()
        self.r.tick()                    # ya abrió y está en la espera
        self.assertEqual(self.r.index, 1)
        self.sent.clear()

    def test_stop_during_the_wait_closes_it(self):
        self.r.stop()
        self.assertEqual(self.sent, ["servo bajar", "stop"])

    def test_fault_during_the_wait_closes_it(self):
        self.r.on_event("#FALLA: algo")
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(self.sent, ["servo bajar"])

    def test_error_during_the_wait_closes_it(self):
        self.r.on_event("#ERROR: algo")
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(self.sent, ["servo bajar"])

    def test_nothing_extra_once_it_is_closed(self):
        self.t = 0.4; self.r.tick()      # termina la espera y cierra
        self.r.tick()                    # pasa a la segunda espera
        self.assertEqual(self.r.index, 3)
        self.sent.clear()
        self.r.stop()
        self.assertEqual(self.sent, ["stop"])

    def test_an_angle_step_also_counts_as_up(self):
        sent = []
        r = sq.SequenceRunner([sq._parse_step({"servo": {"accion": 90, "tiempo": 0}}), sq._wait(1.0)], 1, sent.append, lambda: 0.0)
        r.start(); r.tick()
        r.stop()
        self.assertEqual(sent, ["servo 90", "servo bajar", "stop"])


if __name__ == "__main__":
    unittest.main()
