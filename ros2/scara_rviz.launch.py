"""ros2 launch ros2/scara_rviz.launch.py   (con ROS 2 cargado y el agente micro-ROS corriendo)"""
import os
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    with open(os.path.join(HERE, "scara.urdf")) as f:
        urdf = f.read()
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, "scara_rviz.py")], output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(HERE, "scara.rviz")]),
    ])
