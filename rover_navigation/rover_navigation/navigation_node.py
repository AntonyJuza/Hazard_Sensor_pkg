#!/usr/bin/env python3
"""
navigation_node.py
Reactive obstacle avoidance navigation for the fire-rescue rover.

Subscribes to:
  /ultrasonic/distances          Float32MultiArray  [front, front_left, front_right] cm
  /fire_detected                 std_msgs/Bool      from YOLO/VLM server
  /fire_direction                std_msgs/String    "left" | "center" | "right"
  /emergency_stop                std_msgs/Bool      mirrors motor pkg e-stop topic

Publishes to:
  /cmd_vel                       geometry_msgs/Twist  →  consumed by CytronNode
  /nav_status                    std_msgs/String      status for UI dashboard

Motor interface (from cytron_controller_node_improved.cpp):
  Topic  : /cmd_vel  (geometry_msgs/msg/Twist)
  linear.x  : forward/backward speed  (m/s, clamped by max_linear_speed=0.5)
  angular.z : rotation speed          (rad/s, clamped by max_angular_speed=1.0)
  The CytronNode converts these to L/R wheel % using differential drive:
      left_vel  = linear - (angular * wheel_base / 2)
      right_vel = linear + (angular * wheel_base / 2)
  Then maps to -100..100 PWM range on GPIO 12 (RC1) and GPIO 13 (RC2).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Twist
import time

# ─── DISTANCE THRESHOLDS (cm) ─────────────────────────────────────────────────
DANGER_DIST   = 25.0   # Stop / emergency avoidance
CAUTION_DIST  = 50.0   # Begin turning to avoid
CLEAR_DIST    = 60.0   # Consider path clear

# ─── SPEED CONSTANTS (m/s and rad/s matching motor pkg params) ────────────────
SPEED_FORWARD_NORMAL  = 0.25   # Normal search/approach forward speed
SPEED_FORWARD_SLOW    = 0.12   # Slow approach near fire
SPEED_BACKWARD        = -0.15  # Reverse to escape dead end
TURN_SPEED_NORMAL     = 0.6    # rad/s normal turn
TURN_SPEED_SLOW       = 0.3    # rad/s fine correction toward fire

# ─── TIMING ───────────────────────────────────────────────────────────────────
REVERSE_DURATION_S    = 0.8    # How long to reverse before re-evaluating
SEARCH_ROTATE_S       = 0.5    # Rotation burst duration during search sweep
# ─────────────────────────────────────────────────────────────────────────────


class NavigationState:
    SEARCH   = "SEARCH"      # No fire — roaming/searching
    APPROACH = "APPROACH"    # Fire confirmed — navigating toward it
    AVOID    = "AVOID"       # Obstacle in path — executing avoidance
    REVERSE  = "REVERSE"     # Reversing out of dead end
    STOPPED  = "STOPPED"     # E-stop active


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("loop_rate_hz",       10.0)
        self.declare_parameter("danger_dist_cm",     DANGER_DIST)
        self.declare_parameter("caution_dist_cm",    CAUTION_DIST)
        self.declare_parameter("forward_speed",      SPEED_FORWARD_NORMAL)
        self.declare_parameter("turn_speed",         TURN_SPEED_NORMAL)
        self.declare_parameter("wheel_base",         0.235)  # Match motor pkg (measured!)

        rate              = self.get_parameter("loop_rate_hz").value
        self.danger_dist  = self.get_parameter("danger_dist_cm").value
        self.caution_dist = self.get_parameter("caution_dist_cm").value
        self.fwd_speed    = self.get_parameter("forward_speed").value
        self.turn_speed   = self.get_parameter("turn_speed").value

        # ── State ─────────────────────────────────────────────────────────────
        self.state          = NavigationState.SEARCH
        self.fire_detected  = False
        self.fire_direction = "center"   # last known fire direction
        self.emergency_stop = False

        # Sensor readings (cm) — initialise to "all clear"
        self.dist_front      = 200.0
        self.dist_front_left  = 200.0
        self.dist_front_right = 200.0

        # Reverse / search sweep timing
        self._reverse_start  = None
        self._search_dir     = 1.0   # 1.0 = left, -1.0 = right
        self._search_count   = 0

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray, "/ultrasonic/distances",
            self._ultrasonic_cb, 10)

        self.create_subscription(
            Bool, "/fire_detected",
            self._fire_detected_cb, 10)

        self.create_subscription(
            String, "/fire_direction",
            self._fire_direction_cb, 10)

        self.create_subscription(
            Bool, "/emergency_stop",
            self._estop_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        # /cmd_vel → CytronNode (cytron_controller_node_improved.cpp)
        self.cmd_vel_pub  = self.create_publisher(Twist, "/cmd_vel",    10)
        self.status_pub   = self.create_publisher(String, "/nav_status", 10)

        # ── Control loop timer ───────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f"NavigationNode ready | "
            f"danger={self.danger_dist}cm  caution={self.caution_dist}cm  "
            f"fwd={self.fwd_speed}m/s  turn={self.turn_speed}rad/s"
        )

    # ─── Subscriber callbacks ─────────────────────────────────────────────────
    def _ultrasonic_cb(self, msg: Float32MultiArray):
        """Receives [front, front_left, front_right] in cm."""
        if len(msg.data) >= 3:
            self.dist_front       = msg.data[0]
            self.dist_front_left  = msg.data[1]
            self.dist_front_right = msg.data[2]

    def _fire_detected_cb(self, msg: Bool):
        prev = self.fire_detected
        self.fire_detected = msg.data
        if self.fire_detected and not prev:
            self.get_logger().info("🔥 Fire confirmed — switching to APPROACH mode")
            self.state = NavigationState.APPROACH
        elif not self.fire_detected and prev:
            self.get_logger().info("Fire lost — switching to SEARCH mode")
            self.state = NavigationState.SEARCH

    def _fire_direction_cb(self, msg: String):
        self.fire_direction = msg.data.strip().lower()

    def _estop_cb(self, msg: Bool):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn("🚨 E-STOP received — halting navigation")
            self.state = NavigationState.STOPPED
            self._publish_cmd(0.0, 0.0)
        else:
            self.get_logger().info("E-STOP cleared — resuming SEARCH")
            self.state = NavigationState.SEARCH

    # ─── cmd_vel helper ───────────────────────────────────────────────────────
    def _publish_cmd(self, linear: float, angular: float):
        """
        Send Twist to /cmd_vel.
        CytronNode maps this as:
          left_wheel  = linear - (angular * wheel_base/2)  → -100..100 PWM
          right_wheel = linear + (angular * wheel_base/2)  → -100..100 PWM
        Positive angular.z = counter-clockwise (turn left).
        """
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f"[NAV] {text}")

    # ─── Obstacle checks ──────────────────────────────────────────────────────
    @property
    def front_blocked(self) -> bool:
        return self.dist_front < self.danger_dist

    @property
    def front_cautious(self) -> bool:
        return self.dist_front < self.caution_dist

    @property
    def left_clear(self) -> bool:
        return self.dist_front_left >= self.caution_dist

    @property
    def right_clear(self) -> bool:
        return self.dist_front_right >= self.caution_dist

    # ─── Main control loop ────────────────────────────────────────────────────
    def _control_loop(self):
        if self.emergency_stop:
            self._publish_cmd(0.0, 0.0)
            return

        # ── STOPPED ──────────────────────────────────────────────────────────
        if self.state == NavigationState.STOPPED:
            self._publish_cmd(0.0, 0.0)
            return

        # ── REVERSE ──────────────────────────────────────────────────────────
        if self.state == NavigationState.REVERSE:
            elapsed = time.time() - self._reverse_start
            if elapsed < REVERSE_DURATION_S:
                self._publish_cmd(SPEED_BACKWARD, 0.0)
                self._publish_status("REVERSE — clearing obstacle")
                return
            else:
                # After reversing, turn toward the clearer side
                turn_dir = self._search_dir
                self._publish_cmd(0.0, turn_dir * self.turn_speed)
                self.state = NavigationState.SEARCH
                return

        # ── SEARCH mode ──────────────────────────────────────────────────────
        if self.state == NavigationState.SEARCH:
            self._search_behavior()

        # ── APPROACH mode ─────────────────────────────────────────────────────
        elif self.state == NavigationState.APPROACH:
            self._approach_behavior()

    # ─────────────────────────────────────────────────────────────────────────
    def _search_behavior(self):
        """
        Wander and sweep looking for fire.
        Priority: obstacle avoidance > search rotation.
        """
        # Dead end — all blocked
        if self.front_blocked and not self.left_clear and not self.right_clear:
            self._publish_status("SEARCH — dead end, reversing")
            self.state = NavigationState.REVERSE
            self._reverse_start = time.time()
            self._publish_cmd(SPEED_BACKWARD, 0.0)
            return

        # Front blocked — turn to clearer side
        if self.front_blocked or self.front_cautious:
            if self.left_clear and not self.right_clear:
                self._publish_cmd(0.0, self.turn_speed)       # turn left
                self._publish_status(
                    f"SEARCH — front blocked ({self.dist_front:.0f}cm), turning LEFT")
                self._search_dir = 1.0
            elif self.right_clear and not self.left_clear:
                self._publish_cmd(0.0, -self.turn_speed)      # turn right
                self._publish_status(
                    f"SEARCH — front blocked ({self.dist_front:.0f}cm), turning RIGHT")
                self._search_dir = -1.0
            else:
                # Both sides similar — alternate to avoid spinning in place
                self._search_count += 1
                dir_sign = 1.0 if self._search_count % 2 == 0 else -1.0
                self._publish_cmd(0.0, dir_sign * self.turn_speed)
                self._publish_status(
                    f"SEARCH — both sides similar, turning {'LEFT' if dir_sign>0 else 'RIGHT'}")
            return

        # Path clear — move forward
        self._publish_cmd(self.fwd_speed, 0.0)
        self._publish_status(
            f"SEARCH — moving forward  "
            f"F:{self.dist_front:.0f} FL:{self.dist_front_left:.0f} FR:{self.dist_front_right:.0f}"
        )

    # ─────────────────────────────────────────────────────────────────────────
    def _approach_behavior(self):
        """
        Navigate toward the fire while avoiding obstacles.
        Uses /fire_direction ("left"|"center"|"right") from the CV server.
        """
        # Safety first — obstacle avoidance overrides fire approach
        if self.front_blocked:
            if self.left_clear:
                self._publish_cmd(0.0, self.turn_speed)
                self._publish_status("APPROACH — obstacle front, dodge LEFT")
            elif self.right_clear:
                self._publish_cmd(0.0, -self.turn_speed)
                self._publish_status("APPROACH — obstacle front, dodge RIGHT")
            else:
                self.state = NavigationState.REVERSE
                self._reverse_start = time.time()
                self._publish_cmd(SPEED_BACKWARD, 0.0)
                self._publish_status("APPROACH — blocked, reversing")
            return

        # Steer toward fire direction
        if self.fire_direction == "left":
            # Fire to the left — gentle left turn while moving forward
            self._publish_cmd(SPEED_FORWARD_SLOW, TURN_SPEED_SLOW)
            self._publish_status(
                f"APPROACH → fire LEFT  F:{self.dist_front:.0f}cm")

        elif self.fire_direction == "right":
            # Fire to the right — gentle right turn while moving forward
            self._publish_cmd(SPEED_FORWARD_SLOW, -TURN_SPEED_SLOW)
            self._publish_status(
                f"APPROACH → fire RIGHT  F:{self.dist_front:.0f}cm")

        else:
            # Fire centered — move straight toward it
            # Slow down as we get close
            speed = SPEED_FORWARD_SLOW if self.dist_front < CAUTION_DIST else SPEED_FORWARD_NORMAL
            self._publish_cmd(speed, 0.0)
            self._publish_status(
                f"APPROACH → fire CENTER  F:{self.dist_front:.0f}cm  spd={speed}")

    def destroy_node(self):
        # Stop motors on shutdown
        self._publish_cmd(0.0, 0.0)
        self.get_logger().info("NavigationNode shutting down — motors stopped")
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()