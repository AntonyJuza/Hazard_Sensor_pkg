"""
launch/navigation.launch.py

Starts the full navigation stack on the RPI:
  1. ultrasonic_sensor_node  — reads 3 HC-SR04s, publishes /ultrasonic/distances
  2. navigation_node         — subscribes sensor + fire topics, publishes /cmd_vel

The motor node (cytron_controller_node) is NOT started here because it lives
in the humanoid_motor_control package.  Start it separately or add it to a
top-level launch file:
  ros2 run humanoid_motor_control cytron_controller_node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rover_navigation"),
        "config",
        "navigation.yaml",
    )

    ultrasonic_node = Node(
        package="rover_navigation",
        executable="ultrasonic_sensor_node",
        name="ultrasonic_sensor_node",
        parameters=[config],
        output="screen",
        emulate_tty=True,
    )

    navigation_node = Node(
        package="rover_navigation",
        executable="navigation_node",
        name="navigation_node",
        parameters=[config],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        ultrasonic_node,
        navigation_node,
    ])