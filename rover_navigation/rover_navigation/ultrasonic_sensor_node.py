#!/usr/bin/env python3
"""
ultrasonic_sensor_node.py
Reads 3 HC-SR04 ultrasonic sensors (front, front-left, front-right)
and publishes distances as ROS2 topics.

GPIO wiring:
  Front Center : TRIG=GPIO 5,  ECHO=GPIO 6
  Front Left   : TRIG=GPIO 17, ECHO=GPIO 27
  Front Right  : TRIG=GPIO 22, ECHO=GPIO 23

NOTE: HC-SR04 ECHO pin outputs 5V — use a voltage divider (1kΩ + 2kΩ)
      to bring it down to 3.3V safe for RPI GPIO.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import RPi.GPIO as GPIO
import time

# ─── GPIO PIN DEFINITIONS ────────────────────────────────────────────────────
SENSORS = {
    "front":       {"trig": 5,  "echo": 6},
    "front_left":  {"trig": 17, "echo": 27},
    "front_right": {"trig": 22, "echo": 23},
}

MAX_DISTANCE_CM  = 200.0   # Ignore readings beyond this (out of useful range)
MIN_DISTANCE_CM  =   2.0   # HC-SR04 minimum reliable range
TRIGGER_PULSE_S  = 0.00001 # 10 µs trigger pulse
TIMEOUT_S        = 0.03    # 30 ms = ~5m max, avoids infinite loop
MOVING_AVG_SIZE  = 5       # Number of samples for smoothing filter
# ─────────────────────────────────────────────────────────────────────────────


class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_sensor_node")

        # ── Parameters (can override in YAML / launch file) ──────────────────
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("front_trig",       SENSORS["front"]["trig"])
        self.declare_parameter("front_echo",       SENSORS["front"]["echo"])
        self.declare_parameter("front_left_trig",  SENSORS["front_left"]["trig"])
        self.declare_parameter("front_left_echo",  SENSORS["front_left"]["echo"])
        self.declare_parameter("front_right_trig", SENSORS["front_right"]["trig"])
        self.declare_parameter("front_right_echo", SENSORS["front_right"]["echo"])

        rate = self.get_parameter("publish_rate_hz").value

        # Override GPIO pins from parameters
        SENSORS["front"]["trig"]        = self.get_parameter("front_trig").value
        SENSORS["front"]["echo"]        = self.get_parameter("front_echo").value
        SENSORS["front_left"]["trig"]   = self.get_parameter("front_left_trig").value
        SENSORS["front_left"]["echo"]   = self.get_parameter("front_left_echo").value
        SENSORS["front_right"]["trig"]  = self.get_parameter("front_right_trig").value
        SENSORS["front_right"]["echo"]  = self.get_parameter("front_right_echo").value

        # ── GPIO setup ───────────────────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for name, pins in SENSORS.items():
            GPIO.setup(pins["trig"], GPIO.OUT)
            GPIO.setup(pins["echo"], GPIO.IN)
            GPIO.output(pins["trig"], False)
            self.get_logger().info(
                f"  [{name}] TRIG=GPIO{pins['trig']}  ECHO=GPIO{pins['echo']}"
            )
        time.sleep(0.5)  # Let sensors settle after GPIO init

        # ── Moving average filter buffers ────────────────────────────────────
        self._buffers = {name: [] for name in SENSORS}

        # ── Publishers ───────────────────────────────────────────────────────
        # Combined array: [front, front_left, front_right]  (cm)
        self.pub_array = self.create_publisher(
            Float32MultiArray, "/ultrasonic/distances", 10
        )
        # Individual topics for easy debugging
        self.pub_front       = self.create_publisher(Float32, "/ultrasonic/front",       10)
        self.pub_front_left  = self.create_publisher(Float32, "/ultrasonic/front_left",  10)
        self.pub_front_right = self.create_publisher(Float32, "/ultrasonic/front_right", 10)

        # ── Timer ────────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(
            f"UltrasonicSensorNode started @ {rate} Hz"
        )

    # ─────────────────────────────────────────────────────────────────────────
    def _measure_distance_cm(self, trig: int, echo: int) -> float:
        """
        Fire one pulse and measure round-trip time.
        Returns distance in cm, or MAX_DISTANCE_CM on timeout.
        Sensors are triggered sequentially (not simultaneously) to avoid
        cross-talk between HC-SR04 units.
        """
        # Send 10 µs trigger pulse
        GPIO.output(trig, True)
        time.sleep(TRIGGER_PULSE_S)
        GPIO.output(trig, False)

        # Wait for ECHO to go HIGH (pulse start)
        t_start = time.time()
        while GPIO.input(echo) == 0:
            if time.time() - t_start > TIMEOUT_S:
                return MAX_DISTANCE_CM

        pulse_start = time.time()

        # Wait for ECHO to go LOW (pulse end)
        while GPIO.input(echo) == 1:
            if time.time() - pulse_start > TIMEOUT_S:
                return MAX_DISTANCE_CM

        pulse_end = time.time()

        # Distance = (time × speed_of_sound) / 2
        # Speed of sound ≈ 34300 cm/s at room temperature
        duration = pulse_end - pulse_start
        distance = (duration * 34300.0) / 2.0

        # Clamp to sensor's reliable range
        if distance < MIN_DISTANCE_CM or distance > MAX_DISTANCE_CM:
            return MAX_DISTANCE_CM

        return round(distance, 1)

    def _smooth(self, name: str, raw: float) -> float:
        """Simple moving average over last MOVING_AVG_SIZE readings."""
        buf = self._buffers[name]
        buf.append(raw)
        if len(buf) > MOVING_AVG_SIZE:
            buf.pop(0)
        return round(sum(buf) / len(buf), 1)

    # ─────────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        readings = {}

        # Sequential firing — avoids HC-SR04 cross-talk interference
        for name, pins in SENSORS.items():
            raw = self._measure_distance_cm(pins["trig"], pins["echo"])
            readings[name] = self._smooth(name, raw)
            time.sleep(0.015)  # 15 ms gap between sensors

        front       = readings["front"]
        front_left  = readings["front_left"]
        front_right = readings["front_right"]

        # Publish combined array
        arr_msg = Float32MultiArray()
        arr_msg.data = [front, front_left, front_right]
        self.pub_array.publish(arr_msg)

        # Publish individual topics
        self.pub_front.publish(Float32(data=front))
        self.pub_front_left.publish(Float32(data=front_left))
        self.pub_front_right.publish(Float32(data=front_right))

        self.get_logger().debug(
            f"Distances (cm) — F:{front}  FL:{front_left}  FR:{front_right}"
        )

    def destroy_node(self):
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up.")
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()