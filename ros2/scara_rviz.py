#!/usr/bin/env python3
"""
Puente para RViz: convierte /scara/state (micro-ROS) en lo que RViz entiende.

  /scara/state (Float32MultiArray)  ->  /joint_states (JointState)   [robot_state_publisher dibuja el robot]
  /scara/goto  (Point, mm)          ->  marcador de la consigna (esfera verde)
  (estado)                          ->  /scara/markers: espacio de trabajo (corona de alcance), estela del efector

Espacio de trabajo: corona entre |l1-l2| y l1+l2 (l1, l2 de KIN en src/config.c). No incluye los
límites articulares reales (finales de carrera); es el alcance geométrico de la cinemática inversa.
Lanzar todo junto:  ros2 launch ros2/scara_rviz.launch.py
"""
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

L1_MM, L2_MM = 120.0, 70.0          # mantener igual que KIN en src/config.c
FRAME = "base_link"
BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
TRAIL_MAX = 400


def circle(r_m, n=96):
    return [Point(x=r_m * math.cos(2 * math.pi * k / n), y=r_m * math.sin(2 * math.pi * k / n), z=0.0)
            for k in range(n + 1)]


def forward_mm(q1_deg, q2_deg):
    a, b = math.radians(q1_deg), math.radians(q1_deg + q2_deg)
    return L1_MM * math.cos(a) + L2_MM * math.cos(b), L1_MM * math.sin(a) + L2_MM * math.sin(b)


class RvizBridge(Node):
    def __init__(self):
        super().__init__("scara_rviz")
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/scara/markers", 10)
        self.create_subscription(Float32MultiArray, "/scara/state", self.on_state, BEST_EFFORT)
        self.create_subscription(Point, "/scara/goto", self.on_goto, RELIABLE)
        self.target = None
        self.trail = []
        self.q = (0.0, 0.0)
        self.create_timer(0.1, self.publish_markers)

    def on_state(self, m):
        d = m.data
        if len(d) < 2:
            return
        self.q = (d[0], d[1])
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["joint1", "joint2"]
        js.position = [math.radians(d[0]), math.radians(d[1])]
        self.pub_js.publish(js)
        x, y = forward_mm(d[0], d[1])
        p = Point(x=x / 1000.0, y=y / 1000.0, z=0.05)
        if not self.trail or math.hypot(p.x - self.trail[-1].x, p.y - self.trail[-1].y) > 0.002:
            self.trail.append(p)
            self.trail = self.trail[-TRAIL_MAX:]

    def on_goto(self, m):
        self.target = (float(m.x) / 1000.0, float(m.y) / 1000.0)

    def marker(self, mid, mtype, color, scale):
        k = Marker()
        k.header.frame_id = FRAME
        k.header.stamp = self.get_clock().now().to_msg()
        k.ns, k.id, k.type, k.action = "scara", mid, mtype, Marker.ADD
        k.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
        k.scale.x, k.scale.y, k.scale.z = (float(v) for v in scale)   # ROS exige float, no int
        k.pose.orientation.w = 1.0
        return k

    def publish_markers(self):
        arr = MarkerArray()
        r_out = (L1_MM + L2_MM) / 1000.0
        r_in = abs(L1_MM - L2_MM) / 1000.0
        for i, r in enumerate((r_out, r_in)):
            k = self.marker(i, Marker.LINE_STRIP, (0.2, 0.8, 0.3, 0.9), (0.002, 0.0, 0.0))
            k.points = circle(r)
            arr.markers.append(k)
        t = self.marker(10, Marker.LINE_STRIP, (1.0, 0.2, 0.6, 0.9), (0.002, 0.0, 0.0))
        t.points = list(self.trail)
        arr.markers.append(t)
        if self.target:
            g = self.marker(11, Marker.SPHERE, (0.1, 0.9, 0.1, 0.9), (0.012, 0.012, 0.012))
            g.pose.position.x, g.pose.position.y, g.pose.position.z = float(self.target[0]), float(self.target[1]), 0.05
            arr.markers.append(g)
        self.pub_mk.publish(arr)


def main():
    rclpy.init()
    node = RvizBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
