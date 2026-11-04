#!/usr/bin/env python3
"""
Prueba de extremo a extremo del simulador sobre ROS 2 real: lanza scara_sim_node y scara_rviz, y maneja el robot
por los mismos tópicos que el panel.   Ejecutar:  python3 tools/sim/test_e2e.py   (con ROS 2 cargado)
"""
import math
import os
import subprocess
import sys
import ctypes
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray, String

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2"))
import scara_sequence as sq  # noqa: E402
BEST = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
HOLD, FAULT, OFF = 2, 6, 0
L1, L2 = 120.0, 70.0


class Client(Node):
    def __init__(self):
        super().__init__("test_e2e")
        self.state, self.js, self.ev, self.markers = None, None, [], None
        self.create_subscription(MarkerArray, "/scara/markers", lambda m: setattr(self, "markers", list(m.markers)), 10)
        self.cmd_p = self.create_publisher(String, "/scara/cmd", REL)
        self.goto_p = self.create_publisher(Point, "/scara/goto", REL)
        self.jog_p = self.create_publisher(Vector3, "/scara/jog", BEST)
        self.inj_p = self.create_publisher(String, "/scara_sim/inject", REL)
        self.create_subscription(Float32MultiArray, "/scara/state", lambda m: setattr(self, "state", list(m.data)), BEST)
        self.create_subscription(String, "/scara/event", lambda m: self.ev.append(m.data), REL)
        self.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "js", list(m.position)), 10)

    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait(self, pred, timeout, what):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if pred():
                return
        raise AssertionError("no ocurrió: %s (estado %s; últimos eventos %s)" % (what, self.state, self.ev[-4:]))

    def cmd(self, text):
        self.cmd_p.publish(String(data=text))


class E2E(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        env = dict(os.environ, ROS_DOMAIN_ID=str(90 + os.getpid() % 100))
        os.environ.update(env)
        cls.procs = [subprocess.Popen([sys.executable, os.path.join(ROOT, "ros2", f)] + args, env=env)
                     for f, args in (("scara_sim_node.py", ["--velocidad", "10"]), ("scara_rviz.py", []))]
        cls.addClassCleanup(cls._kill_children)             # también si el arranque falla: no deja procesos huérfanos
        rclpy.init()
        cls.addClassCleanup(rclpy.shutdown)
        cls.n = Client()
        cls.n.wait(lambda: cls.n.state is not None, 20, "llega /scara/state")
        for _ in range(10):                       # da tiempo a que se descubran los suscriptores
            cls.n.spin(0.1)
        cls.n.cmd("home")
        cls.n.wait(lambda: cls.n.state[6] == 1 and cls.n.state[7] == 1 and cls.n.state[4] == HOLD, 40, "Home completo")

    @classmethod
    def _kill_children(cls):
        for p in cls.procs:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

    def setUp(self):
        self.n.ev.clear()

    def test_01_state_has_the_new_fields(self):
        s = self.n.state
        self.assertEqual(len(s), 24)
        self.assertEqual(int(s[13]) & 3, 3, "J1 y J2 activos")
        self.assertEqual(int(s[13]) & 4, 0, "M3 desactivado al arrancar")
        self.assertEqual(int(s[14]), 4, "arranca con el RST que da el PWM original")

    def test_02_goto_reaches_target_and_twin_follows(self):
        self.n.goto_p.publish(Point(x=120.0, y=80.0, z=0.0))
        self.n.wait(lambda: any(e.startswith("#LLEGO") for e in self.n.ev), 15, "#LLEGO")
        self.n.spin(0.3)
        s = self.n.state
        self.assertAlmostEqual(s[2], 120.0, delta=1.0)
        self.assertAlmostEqual(s[3], 80.0, delta=1.0)
        q1, q2 = self.n.js
        x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
        y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
        self.assertAlmostEqual(x, 120.0, delta=1.5, msg="el gemelo digital (/joint_states) sigue al robot")
        self.assertAlmostEqual(y, 80.0, delta=1.5)

    def test_03_estop_latches_and_clear_keeps_home(self):
        self.n.cmd("estop")
        self.n.wait(lambda: self.n.state[4] == FAULT and self.n.state[5] == 6, 3, "paro en FAULT")
        self.n.goto_p.publish(Point(x=100.0, y=60.0, z=0.0))
        self.n.spin(0.5)
        self.assertEqual(self.n.state[4], FAULT, "enclavado")
        self.n.cmd("clear")
        self.n.wait(lambda: self.n.state[4] == OFF, 3, "rearme")
        self.assertTrue(self.n.state[6] and self.n.state[7], "conserva el Home")

    def test_04_motor_toggle_is_reported(self):
        self.n.cmd("motor 3 on")
        self.n.wait(lambda: int(self.n.state[13]) & 4, 3, "M3 activo")
        n_ev = len(self.n.ev)
        self.n.cmd("m3 home")
        self.n.wait(lambda: any(e.startswith("#M3") for e in self.n.ev[n_ev:]), 40, "M3 halla su tope")
        self.n.cmd("tool 50")
        self.n.wait(lambda: abs(self.n.state[15]) > 100, 3, "M3 gira")
        self.n.cmd("motor 3 off")
        self.n.wait(lambda: not int(self.n.state[13]) & 4 and self.n.state[15] == 0, 3, "M3 desactivado y parado")

    def test_05_jog_needs_deadman_and_stops_without_heartbeat(self):
        q0 = self.n.state[0]
        for _ in range(15):
            self.n.jog_p.publish(Vector3(x=0.0, y=1.0, z=0.0))
            self.n.spin(0.04)
        self.assertAlmostEqual(self.n.state[0], q0, delta=0.05, msg="sin dead-man no se mueve")
        for _ in range(25):
            self.n.jog_p.publish(Vector3(x=1.0, y=1.0, z=0.0))
            self.n.spin(0.04)
        self.assertGreater(self.n.state[0], q0 + 2.0, "con dead-man se mueve")
        self.n.wait(lambda: self.n.state[4] == HOLD, 3, "sin latidos se detiene")

    def test_06_rviz_bridge_shows_reference_marker(self):
        self.n.spin(0.3)
        self.assertIsNotNone(self.n.markers, "llegan marcadores del gemelo digital")
        ids = {m.id for m in self.n.markers}
        self.assertIn(12, ids, "marcador de la referencia")

    def test_06b_rviz_shows_boards_and_holes(self):
        self.n.spin(0.3)
        by_id = {m.id: m for m in self.n.markers}
        self.assertEqual(len(by_id[20].points), 4, "las 4 placas")
        self.assertEqual(len(by_id[21].points), 16, "los 16 huecos")
        self.assertAlmostEqual(by_id[20].points[3].y, -0.102, places=3, msg="placa D en el marco de la base")

    def test_06c_full_process_visits_all_holes(self):
        self._run_process_until_done(sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml")))
        self.assertEqual(self.runner.state, "done", self.runner.reason)
        self.assertEqual([r["resultado"] for r in self.runner.results], ["hecho"] * 16)
        self.assertLess(max(self.errors), 1.0, "error de posición al subir el servo [mm]: %s" % self.errors)
        self.n.wait(lambda: abs(self.n.state[2] - sq.L1_MM - sq.L2_MM) < 2.0, 15, "vuelve a 0 0")

    def test_06d_one_pcb_visits_only_its_holes(self):
        self._run_process_until_done(sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml"), pcb=2))
        self.assertEqual(self.runner.state, "done", self.runner.reason)
        self.assertEqual([r["hueco"] for r in self.runner.results], ["B-H2", "B-H4", "B-H3", "B-H1"])

    def test_06e_rehearsal_does_not_move_the_servo(self):
        servos = []
        self._run_process_until_done(sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml"), ensayo=True, pcb=1), lambda s: servos.append(s[22]))
        self.assertEqual(self.runner.state, "done", self.runner.reason)
        self.assertEqual([r["resultado"] for r in self.runner.results], ["ensayado"] * 4)
        self.assertEqual(len(set(servos)), 1, "el servo no se mueve")

    def _run_process_until_done(self, seq=None, watch=None):
        self.seq = seq or sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml"))
        n_ev = len(self.n.ev)
        self.n.cmd("m3 home")
        self.n.wait(lambda: any(e.startswith("#M3") for e in self.n.ev[n_ev:]), 40, "M3 halla su tope")
        self.errors = []
        self.runner = sq.SequenceRunner(self.seq.steps, 1, self.n.cmd, time.monotonic)
        seen = len(self.n.ev)
        self.runner.start()
        last = None
        t0 = time.time()
        while self.runner.running and time.time() - t0 < 150:
            rclpy.spin_once(self.n, timeout_sec=0.005)
            for e in self.n.ev[seen:]:
                self.runner.on_event(e)
            seen = len(self.n.ev)
            self.runner.tick()
            if watch:
                watch(self.n.state)
            step = self.runner.steps[self.runner.index] if self.runner.running else None
            if step and step is not last:
                if step.get("text") == "servo subir":
                    g = self.runner.steps[self.runner.index - 2]
                    self.errors.append(math.hypot(self.n.state[2] - g["x"], self.n.state[3] - g["y"]))
                last = step

    def test_07_injected_lock_causes_fault(self):
        self.n.inj_p.publish(String(data="lock 1"))
        self.n.spin(0.2)
        self.n.cmd("ang 90 90")
        self.n.wait(lambda: self.n.state[4] == FAULT, 10, "eje trabado -> FAULT")
        self.assertIn(int(self.n.state[5]), (1, 2))
        self.n.inj_p.publish(String(data="free 1"))
        self.n.cmd("clear")
        self.n.wait(lambda: self.n.state[4] == OFF, 3, "clear")


if __name__ == "__main__":
    unittest.main(verbosity=1)
