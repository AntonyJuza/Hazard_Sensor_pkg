from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # Camera stream (Python script)
        ExecuteProcess(
            cmd=['python3', '/home/pi/Humanoid/camera_stream.py'],
            output='screen'
        ),

        # Motor control launch
        ExecuteProcess(
            cmd=['ros2', 'launch', 'humanoid_motor_control', 'motor_control.launch.py'],
            output='screen'
        ),

        # MPU6050 node
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros2_mpu6050', 'ros2_mpu6050'],
            output='screen'
        ),

        # Sensors launch
        ExecuteProcess(
            cmd=['ros2', 'launch', 'sensor_pkg_python', 'sensors.launch.py'],
            output='screen'
        ),

        # Navigation launch
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rover_navigation', 'navigation.launch.py'],
            output='screen'
        ),

    ])