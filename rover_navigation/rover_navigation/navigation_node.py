#!/usr/bin/env python3
"""
navigation_node.py — v4  (IMU Recovery Scan)
=============================================
New feature: RECOVERY_SCAN state replaces the old simple REVERSE state.

When the rover hits a dead end it executes a structured 3-phase maneuver:

  Phase 1 — BACKING
    Reverse straight until dist_front > SCAN_CLEAR_DIST (40cm).
    Uses actual distance, not a timer — so it backs up exactly as far as needed.

  Phase 2 — SCAN LEFT then SCAN RIGHT
    Rotate +90° left  → hold 0.3s → read left sensor  → rotate back to 0°
    Rotate -90° right → hold 0.3s → read right sensor → rotate back to 0°

    Angle is tracked by integrating angular_velocity.z from the IMU at 100Hz.
    This is reliable because your IMU publishes at exactly 100Hz.
    The rover knows the actual angle turned, not a time estimate.

  Phase 3 — DECIDING + COMMITTING
    Compare recorded left_dist vs right_dist.
    Turn toward the larger value.
    Hold the turn until IMU confirms 90° rotation, then resume SEARCH.
    If both sides < BOTH_BLOCKED_DIST → back up more and rescan.

All v3 features retained:
  - Heading correction (P-controller)
  - Stuck detection
  - Stop at fire
  - Side wall nudge
  - Turn hold (for non-recovery turns)
  - Separate side/front thresholds

IMU runs at 100Hz — angle integration error over 90° turn is < 2°.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time
import math

# ─── FRONT SENSOR THRESHOLDS ─────────────────────────────────────────────────
DANGER_DIST         = 25.0    # cm — avoidance trigger
CAUTION_DIST        = 50.0    # cm — begin planning

# ─── SIDE SENSOR THRESHOLDS ──────────────────────────────────────────────────
SIDE_CLEAR_DIST     = 80.0    # cm — clear enough to turn toward
SIDE_WARN_DIST      = 40.0    # cm — gentle nudge away

# ─── FIRE ────────────────────────────────────────────────────────────────────
FIRE_STOP_DIST      = 20.0    # cm — stop this close to fire

# ─── SPEED ───────────────────────────────────────────────────────────────────
SPEED_FORWARD_NORMAL = 0.13   # m/s  (your tuned value)
SPEED_FORWARD_SLOW   = 0.07   # m/s  near fire
SPEED_BACKWARD       = -0.10  # m/s
TURN_SPEED_NORMAL    = 1.2    # rad/s
TURN_SPEED_SLOW      = 1.0    # rad/s
SIDE_AVOID_ANGULAR   = 1.0   # rad/s

# ─── TURN HOLD (prevents jerk) ────────────────────────────────────────────────
TURN_HOLD_S          = 0.5    # seconds

# ─── IMU HEADING CORRECTION ──────────────────────────────────────────────────
HEADING_KP           = 0.10
HEADING_DEADBAND     = 0.02   # rad/s
HEADING_MIN_SPEED    = 0.05   # m/s

# ─── IMU STUCK DETECTION ─────────────────────────────────────────────────────
STUCK_ACCEL_THRESHOLD = 0.08  # m/s² (lowered for 0.13 m/s forward speed)
STUCK_TIMEOUT_S       = 2.5   # seconds

# ─── RECOVERY SCAN PARAMETERS ────────────────────────────────────────────────
# How far to back up before scanning
SCAN_CLEAR_DIST      = 40.0   # cm — back up until front reads above this

# Failsafe: if backing up takes longer than this, stop and try anyway
SCAN_BACKUP_TIMEOUT  = 3.0    # seconds

# How many degrees to scan each side
SCAN_ANGLE_DEG       = 90.0   # degrees
SCAN_ANGLE_RAD       = math.radians(SCAN_ANGLE_DEG)

# Speed during scan rotation — slow for accuracy
SCAN_TURN_SPEED      = 0.9   # rad/s  (slow = less integration error)

# How long to hold still at each scan point to get stable sensor reading
SCAN_HOLD_S          = 0.4    # seconds

# Angle tolerance — consider target reached within this many degrees
SCAN_ANGLE_TOL_RAD   = math.radians(8.0)   # ±8° tolerance

# If both sides read below this after scan → back up more before deciding
BOTH_BLOCKED_DIST    = 35.0   # cm

# Maximum number of backup+rescan attempts before giving up and picking best side
MAX_SCAN_RETRIES     = 2


class NavigationState:
    SEARCH        = "SEARCH"
    APPROACH      = "APPROACH"
    RECOVERY_SCAN = "RECOVERY_SCAN"
    STOPPED       = "STOPPED"


class ScanPhase:
    """Sub-states within RECOVERY_SCAN."""
    BACKING       = "BACKING"       # reversing to get clearance
    TO_LEFT       = "TO_LEFT"       # rotating to +90°
    HOLD_LEFT     = "HOLD_LEFT"     # paused at left, reading sensor
    RETURN_LEFT   = "RETURN_LEFT"   # rotating back to 0°
    TO_RIGHT      = "TO_RIGHT"      # rotating to -90°
    HOLD_RIGHT    = "HOLD_RIGHT"    # paused at right, reading sensor
    RETURN_RIGHT  = "RETURN_RIGHT"  # rotating back to 0°
    COMMITTING    = "COMMITTING"    # final turn to chosen direction
    DONE          = "DONE"          # handoff back to SEARCH
    DECIDING      = "DECIDING"


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("loop_rate_hz",        10.0)
        self.declare_parameter("danger_dist_cm",      DANGER_DIST)
        self.declare_parameter("caution_dist_cm",     CAUTION_DIST)
        self.declare_parameter("side_clear_dist_cm",  SIDE_CLEAR_DIST)
        self.declare_parameter("side_warn_dist_cm",   SIDE_WARN_DIST)
        self.declare_parameter("fire_stop_dist_cm",   FIRE_STOP_DIST)
        self.declare_parameter("forward_speed",       SPEED_FORWARD_NORMAL)
        self.declare_parameter("turn_speed",          TURN_SPEED_NORMAL)
        self.declare_parameter("wheel_base",          0.235)
        self.declare_parameter("heading_kp",          HEADING_KP)
        self.declare_parameter("heading_deadband",    HEADING_DEADBAND)
        self.declare_parameter("stuck_timeout_s",     STUCK_TIMEOUT_S)
        self.declare_parameter("turn_hold_s",         TURN_HOLD_S)
        self.declare_parameter("scan_clear_dist_cm",  SCAN_CLEAR_DIST)
        self.declare_parameter("scan_angle_deg",      SCAN_ANGLE_DEG)
        self.declare_parameter("scan_turn_speed",     SCAN_TURN_SPEED)
        self.declare_parameter("scan_hold_s",         SCAN_HOLD_S)
        self.declare_parameter("both_blocked_dist_cm",BOTH_BLOCKED_DIST)
        self.declare_parameter("imu_topic",           "/imu/mpu6050")

        rate                 = self.get_parameter("loop_rate_hz").value
        self.danger_dist     = self.get_parameter("danger_dist_cm").value
        self.caution_dist    = self.get_parameter("caution_dist_cm").value
        self.side_clear_dist = self.get_parameter("side_clear_dist_cm").value
        self.side_warn_dist  = self.get_parameter("side_warn_dist_cm").value
        self.fire_stop_dist  = self.get_parameter("fire_stop_dist_cm").value
        self.fwd_speed       = self.get_parameter("forward_speed").value
        self.turn_speed      = self.get_parameter("turn_speed").value
        self.heading_kp      = self.get_parameter("heading_kp").value
        self.heading_db      = self.get_parameter("heading_deadband").value
        self.stuck_timeout   = self.get_parameter("stuck_timeout_s").value
        self.turn_hold_s     = self.get_parameter("turn_hold_s").value
        self.scan_clear_dist = self.get_parameter("scan_clear_dist_cm").value
        self.scan_angle_rad  = math.radians(self.get_parameter("scan_angle_deg").value)
        self.scan_turn_spd   = self.get_parameter("scan_turn_speed").value
        self.scan_hold_s     = self.get_parameter("scan_hold_s").value
        self.both_blocked    = self.get_parameter("both_blocked_dist_cm").value
        imu_topic            = self.get_parameter("imu_topic").value

        # ── Navigation state ──────────────────────────────────────────────────
        self.state           = NavigationState.SEARCH
        self.fire_detected   = False
        self.fire_direction  = "center"
        self.emergency_stop  = False

        # ── Sensor readings ───────────────────────────────────────────────────
        self.dist_front       = 200.0
        self.dist_front_left  = 200.0
        self.dist_front_right = 200.0

        # ── IMU state ─────────────────────────────────────────────────────────
        self.imu_yaw_rate    = 0.0
        self.imu_accel_x     = 0.0
        self.imu_available   = False
        self._last_imu_time  = None    # for accurate dt calculation

        # ── Heading correction ────────────────────────────────────────────────
        self._heading_correction = 0.0

        # ── Stuck detection ───────────────────────────────────────────────────
        self._stuck_timer_start  = None
        self._is_commanding_fwd  = False

        # ── Turn hold (v3 jerk fix) ───────────────────────────────────────────
        self._turn_direction      = 0.0
        self._turn_committed_time = 0.0

        # ── Recovery scan state ───────────────────────────────────────────────
        self._scan_phase          = ScanPhase.BACKING
        self._scan_angle_accum    = 0.0    # integrated angle (radians)
        self._scan_phase_start    = 0.0    # time phase began
        self._scan_backup_start   = 0.0    # time backup began
        self._scan_left_dist      = 0.0    # recorded left scan reading
        self._scan_right_dist     = 0.0    # recorded right scan reading
        self._scan_chosen_dir     = 0.0    # +1 left, -1 right
        self._scan_retry_count    = 0
        self._scan_commit_accum   = 0.0    # angle integrated during commit turn

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray, "/ultrasonic/distances",
            self._ultrasonic_cb, 10)
        self.create_subscription(
            Imu,    imu_topic,         self._imu_cb,            10)
        # self.create_subscription(
        #     Bool,   "/fire_detected",  self._fire_detected_cb,  10)
        # self.create_subscription(
        #     String, "/fire_direction", self._fire_direction_cb, 10)
        self.create_subscription(
            Bool,   "/emergency_stop", self._estop_cb,          10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist,  "/cmd_vel",    10)
        self.status_pub  = self.create_publisher(String, "/nav_status", 10)

        self.timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f"NavigationNode v4 (Recovery Scan) ready\n"
            f"  Scan angle     : {math.degrees(self.scan_angle_rad):.0f}°\n"
            f"  Scan turn speed: {self.scan_turn_spd} rad/s\n"
            f"  Backup until   : {self.scan_clear_dist}cm front clear\n"
            f"  IMU topic      : {imu_topic} (100Hz ✓)"
        )

    # ─── Sensor callbacks ─────────────────────────────────────────────────────

    def _ultrasonic_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.dist_front       = msg.data[0]
            self.dist_front_left  = msg.data[1]
            self.dist_front_right = msg.data[2]

    def _imu_cb(self, msg: Imu):
        """
        100Hz IMU callback.
        We calculate dt from actual timestamps rather than assuming 10ms
        because the control loop runs at 10Hz — the IMU data arrives 10x
        faster and we integrate it here directly for maximum accuracy.
        """
        now = time.time()
        if self._last_imu_time is not None:
            dt = now - self._last_imu_time
            # Clamp dt to reasonable range (handles occasional missed messages)
            dt = max(0.001, min(dt, 0.05))

            # Integrate yaw rate → angle accumulation
            # Used by recovery scan to know actual rotation angle
            if self.state == NavigationState.RECOVERY_SCAN:
                self._scan_angle_accum += self.imu_yaw_rate * dt

                # Also integrate commit turn separately
                if self._scan_phase == ScanPhase.COMMITTING:
                    self._scan_commit_accum += self.imu_yaw_rate * dt

        self._last_imu_time  = now
        self.imu_yaw_rate    = msg.angular_velocity.z
        self.imu_accel_x     = msg.linear_acceleration.x
        self.imu_available   = True

    def _fire_detected_cb(self, msg: Bool):
        prev = self.fire_detected
        self.fire_detected = msg.data
        if self.fire_detected and not prev:
            self.get_logger().info("🔥 Fire confirmed — APPROACH mode")
            self.state = NavigationState.APPROACH
        elif not self.fire_detected and prev:
            self.get_logger().info("Fire cleared — SEARCH mode")
            self.state = NavigationState.SEARCH

    def _fire_direction_cb(self, msg: String):
        self.fire_direction = msg.data.strip().lower()

    def _estop_cb(self, msg: Bool):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn("🚨 E-STOP")
            self.state = NavigationState.STOPPED
            self._publish_cmd(0.0, 0.0)
        else:
            self.get_logger().info("E-STOP cleared")
            self.state = NavigationState.SEARCH

    # ─── Sensor properties ────────────────────────────────────────────────────

    @property
    def front_blocked(self):
        return self.dist_front < self.danger_dist

    @property
    def front_cautious(self):
        return self.dist_front < self.caution_dist

    @property
    def left_clear_for_turn(self):
        return self.dist_front_left >= self.side_clear_dist

    @property
    def right_clear_for_turn(self):
        return self.dist_front_right >= self.side_clear_dist

    @property
    def left_wall_close(self):
        return self.dist_front_left < self.side_warn_dist

    @property
    def right_wall_close(self):
        return self.dist_front_right < self.side_warn_dist

    # ─── Turn direction management ────────────────────────────────────────────

    def _choose_turn_direction(self) -> float:
        now = time.time()
        if ((now - self._turn_committed_time) < self.turn_hold_s
                and self._turn_direction != 0.0):
            return self._turn_direction

        l = self.dist_front_left
        r = self.dist_front_right

        if self.left_clear_for_turn and not self.right_clear_for_turn:
            new_dir = 1.0
        elif self.right_clear_for_turn and not self.left_clear_for_turn:
            new_dir = -1.0
        elif l > r:
            new_dir = 1.0
        elif r > l:
            new_dir = -1.0
        else:
            new_dir = -self._turn_direction if self._turn_direction != 0.0 else 1.0

        if new_dir != self._turn_direction:
            self._turn_direction      = new_dir
            self._turn_committed_time = now

        return self._turn_direction

    # ─── IMU heading correction ───────────────────────────────────────────────

    def _apply_heading_correction(self, linear, angular):
        if not self.imu_available or abs(linear) < HEADING_MIN_SPEED:
            self._heading_correction = 0.0
            return linear, angular
        if abs(angular) > 0.05:
            self._heading_correction = 0.0
            return linear, angular
        yaw = self.imu_yaw_rate
        if abs(yaw) < self.heading_db:
            self._heading_correction = 0.0
            return linear, angular
        corr = max(-0.3, min(0.3, -self.heading_kp * yaw))
        self._heading_correction = corr
        return linear, angular + corr

    # ─── IMU stuck detection ──────────────────────────────────────────────────

    def _check_stuck(self) -> bool:
        if not self.imu_available or not self._is_commanding_fwd:
            self._stuck_timer_start = None
            return False
        if abs(self.imu_accel_x) > STUCK_ACCEL_THRESHOLD:
            self._stuck_timer_start = None
            return False
        if self._stuck_timer_start is None:
            self._stuck_timer_start = time.time()
            return False
        if (time.time() - self._stuck_timer_start) >= self.stuck_timeout:
            self._stuck_timer_start = None
            self.get_logger().warn(f"🚧 STUCK — accel={self.imu_accel_x:.3f}")
            return True
        return False

    # ─── cmd_vel ─────────────────────────────────────────────────────────────

    def _publish_cmd(self, linear: float, angular: float):
        self._is_commanding_fwd = linear > HEADING_MIN_SPEED
        # Skip heading correction during recovery scan — it would interfere
        if self.state != NavigationState.RECOVERY_SCAN:
            linear, angular = self._apply_heading_correction(linear, angular)
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f"[NAV] {text}")

    # ─── Start recovery scan ──────────────────────────────────────────────────

    def _start_recovery_scan(self):
        """Enter RECOVERY_SCAN state and reset all scan tracking variables."""
        self.get_logger().info("🔍 Starting recovery scan")
        self.state             = NavigationState.RECOVERY_SCAN
        self._scan_phase       = ScanPhase.BACKING
        self._scan_angle_accum = 0.0
        self._scan_commit_accum = 0.0
        self._scan_left_dist   = 0.0
        self._scan_right_dist  = 0.0
        self._scan_chosen_dir  = 0.0
        self._scan_backup_start = time.time()
        self._scan_phase_start = time.time()

    # ─── Recovery scan state machine ─────────────────────────────────────────

    def _recovery_scan_behavior(self):
        """
        Full 3-phase recovery scan using IMU angle integration.

        BACKING     → reverse until front clears SCAN_CLEAR_DIST
        TO_LEFT     → rotate +90° (IMU tracked)
        HOLD_LEFT   → pause, read sensor
        RETURN_LEFT → rotate back to 0° (IMU tracked)
        TO_RIGHT    → rotate -90° (IMU tracked)
        HOLD_RIGHT  → pause, read sensor
        RETURN_RIGHT→ rotate back to 0° (IMU tracked)
        COMMITTING  → turn to chosen direction for full 90° (IMU tracked)
        DONE        → back to SEARCH
        """

        # ── Phase 1: BACKING ─────────────────────────────────────────────────
        if self._scan_phase == ScanPhase.BACKING:
            elapsed = time.time() - self._scan_backup_start

            # Success: front is clear enough to scan
            if self.dist_front >= self.scan_clear_dist:
                self._publish_cmd(0.0, 0.0)
                self._scan_angle_accum = 0.0   # reset for rotation tracking
                self._scan_phase       = ScanPhase.TO_LEFT
                self._scan_phase_start = time.time()
                self._publish_status(
                    f"SCAN — backed up, front clear at {self.dist_front:.0f}cm → scanning LEFT"
                )
                return

            # Failsafe: if backing up too long, scan anyway
            if elapsed > SCAN_BACKUP_TIMEOUT:
                self.get_logger().warn(
                    "SCAN backup timeout — scanning anyway "
                    f"(front={self.dist_front:.0f}cm)"
                )
                self._publish_cmd(0.0, 0.0)
                self._scan_angle_accum = 0.0
                self._scan_phase       = ScanPhase.TO_LEFT
                self._scan_phase_start = time.time()
                return

            self._publish_cmd(SPEED_BACKWARD, 0.0)
            self._publish_status(
                f"SCAN — backing up  front={self.dist_front:.0f}cm "
                f"(target={self.scan_clear_dist:.0f}cm)"
            )

        # ── Phase 2: TO_LEFT — rotate to +90° ────────────────────────────────
        elif self._scan_phase == ScanPhase.TO_LEFT:
            angle = self._scan_angle_accum   # positive = CCW = left

            if angle >= (self.scan_angle_rad - SCAN_ANGLE_TOL_RAD):
                # Reached +90° — stop and hold
                self._publish_cmd(0.0, 0.0)
                self._scan_phase       = ScanPhase.HOLD_LEFT
                self._scan_phase_start = time.time()
                self._publish_status(
                    f"SCAN — reached LEFT {math.degrees(angle):.1f}° → holding"
                )
                return

            self._publish_cmd(0.0, self.scan_turn_spd)
            self._publish_status(
                f"SCAN → LEFT  {math.degrees(angle):.1f}° / {SCAN_ANGLE_DEG:.0f}°"
            )

        # ── Phase 3: HOLD_LEFT — pause and read ──────────────────────────────
        elif self._scan_phase == ScanPhase.HOLD_LEFT:
            elapsed = time.time() - self._scan_phase_start
            self._publish_cmd(0.0, 0.0)

            if elapsed >= self.scan_hold_s:
                # Record the best (maximum) of front and left sensors
                # at this angle — front sensor now points left
                self._scan_left_dist = self.dist_front
                self._publish_status(
                    f"SCAN — LEFT reading: {self._scan_left_dist:.0f}cm → returning to center"
                )
                # Reset accumulator to track return to 0°
                self._scan_angle_accum = self.scan_angle_rad  # starts at +90°
                self._scan_phase       = ScanPhase.RETURN_LEFT
                self._scan_phase_start = time.time()
            else:
                self._publish_status(
                    f"SCAN — holding LEFT ({elapsed:.1f}s / {self.scan_hold_s:.1f}s) "
                    f"front={self.dist_front:.0f}cm"
                )

        # ── Phase 4: RETURN_LEFT — rotate back to 0° ─────────────────────────
        elif self._scan_phase == ScanPhase.RETURN_LEFT:
            angle = self._scan_angle_accum   # counting down from +90° to 0°

            if angle <= SCAN_ANGLE_TOL_RAD:
                # Back at center
                self._publish_cmd(0.0, 0.0)
                self._scan_angle_accum = 0.0
                self._scan_phase       = ScanPhase.TO_RIGHT
                self._scan_phase_start = time.time()
                self._publish_status(
                    f"SCAN — back at center → scanning RIGHT"
                )
                return

            # Rotate right (negative) to return from left
            self._publish_cmd(0.0, -self.scan_turn_spd)
            self._publish_status(
                f"SCAN ← returning from LEFT  {math.degrees(angle):.1f}° remaining"
            )

        # ── Phase 5: TO_RIGHT — rotate to -90° ───────────────────────────────
        elif self._scan_phase == ScanPhase.TO_RIGHT:
            angle = self._scan_angle_accum   # goes negative (CW = right)

            if angle <= -(self.scan_angle_rad - SCAN_ANGLE_TOL_RAD):
                # Reached -90°
                self._publish_cmd(0.0, 0.0)
                self._scan_phase       = ScanPhase.HOLD_RIGHT
                self._scan_phase_start = time.time()
                self._publish_status(
                    f"SCAN — reached RIGHT {math.degrees(angle):.1f}° → holding"
                )
                return

            self._publish_cmd(0.0, -self.scan_turn_spd)
            self._publish_status(
                f"SCAN → RIGHT  {math.degrees(angle):.1f}° / -{SCAN_ANGLE_DEG:.0f}°"
            )

        # ── Phase 6: HOLD_RIGHT — pause and read ─────────────────────────────
        elif self._scan_phase == ScanPhase.HOLD_RIGHT:
            elapsed = time.time() - self._scan_phase_start
            self._publish_cmd(0.0, 0.0)

            if elapsed >= self.scan_hold_s:
                self._scan_right_dist = self.dist_front
                self._publish_status(
                    f"SCAN — RIGHT reading: {self._scan_right_dist:.0f}cm → returning to center"
                )
                self._scan_angle_accum = -self.scan_angle_rad  # starts at -90°
                self._scan_phase       = ScanPhase.RETURN_RIGHT
                self._scan_phase_start = time.time()
            else:
                self._publish_status(
                    f"SCAN — holding RIGHT ({elapsed:.1f}s / {self.scan_hold_s:.1f}s) "
                    f"front={self.dist_front:.0f}cm"
                )

        # ── Phase 7: RETURN_RIGHT — rotate back to 0° ────────────────────────
        elif self._scan_phase == ScanPhase.RETURN_RIGHT:
            angle = self._scan_angle_accum   # counting up from -90° to 0°

            if angle >= -SCAN_ANGLE_TOL_RAD:
                # Back at center — now decide
                self._publish_cmd(0.0, 0.0)
                self._scan_phase = ScanPhase.DECIDING
                self._publish_status(
                    f"SCAN — back at center → deciding  "
                    f"LEFT={self._scan_left_dist:.0f}cm  RIGHT={self._scan_right_dist:.0f}cm"
                )
                return

            self._publish_cmd(0.0, self.scan_turn_spd)
            self._publish_status(
                f"SCAN → returning from RIGHT  {math.degrees(angle):.1f}° remaining"
            )

        # ── Phase 8: DECIDING — pick best direction ───────────────────────────
        elif self._scan_phase == ScanPhase.DECIDING:
            left  = self._scan_left_dist
            right = self._scan_right_dist

            self.get_logger().info(
                f"🔍 SCAN RESULT — LEFT: {left:.0f}cm  RIGHT: {right:.0f}cm"
            )

            # Both sides very blocked — need to back up more
            if left < self.both_blocked and right < self.both_blocked:
                self._scan_retry_count += 1
                if self._scan_retry_count <= MAX_SCAN_RETRIES:
                    self.get_logger().warn(
                        f"SCAN — both sides blocked ({left:.0f}/{right:.0f}cm), "
                        f"backing up more (retry {self._scan_retry_count}/{MAX_SCAN_RETRIES})"
                    )
                    # Back up more and rescan
                    self._scan_phase        = ScanPhase.BACKING
                    self._scan_backup_start = time.time()
                    self._scan_angle_accum  = 0.0
                    return
                else:
                    # Max retries hit — pick whichever side is less bad
                    self.get_logger().warn(
                        "SCAN — max retries hit, picking least-blocked side"
                    )

            # Choose the side with more space
            if left >= right:
                self._scan_chosen_dir = 1.0   # turn left
                chosen_name = "LEFT"
                chosen_dist = left
            else:
                self._scan_chosen_dir = -1.0  # turn right
                chosen_name = "RIGHT"
                chosen_dist = right

            self._scan_retry_count  = 0
            self._scan_commit_accum = 0.0
            self._scan_angle_accum  = 0.0
            self._scan_phase        = ScanPhase.COMMITTING
            self._scan_phase_start  = time.time()

            self._publish_status(
                f"SCAN → chose {chosen_name} ({chosen_dist:.0f}cm) — committing 90° turn"
            )

        # ── Phase 9: COMMITTING — turn 90° to chosen direction ───────────────
        elif self._scan_phase == ScanPhase.COMMITTING:
            # Track how far we've turned using the separate commit accumulator
            turned = abs(self._scan_commit_accum)
            target = self.scan_angle_rad - SCAN_ANGLE_TOL_RAD

            if turned >= target:
                # 90° turn complete — back to SEARCH
                self._publish_cmd(0.0, 0.0)
                self._scan_phase = ScanPhase.DONE
                self._publish_status(
                    f"SCAN — committed {math.degrees(turned):.1f}° → resuming SEARCH"
                )
                return

            self._publish_cmd(0.0, self._scan_chosen_dir * self.scan_turn_spd)
            self._publish_status(
                f"SCAN — committing turn  {math.degrees(turned):.1f}° / {SCAN_ANGLE_DEG:.0f}°"
            )

        # ── Phase DONE — return to SEARCH ────────────────────────────────────
        elif self._scan_phase == ScanPhase.DONE:
            self.state = NavigationState.SEARCH
            self.get_logger().info("✅ Recovery scan complete — SEARCH resumed")

    # ─── Main control loop ────────────────────────────────────────────────────

    def _control_loop(self):
        if self.emergency_stop or self.state == NavigationState.STOPPED:
            self._publish_cmd(0.0, 0.0)
            return

        # Stuck detection — not during recovery scan (it intentionally moves slow)
        if self.state in (NavigationState.SEARCH, NavigationState.APPROACH):
            if self._check_stuck():
                self._publish_status(
                    f"🚧 STUCK — triggering recovery scan"
                )
                self._start_recovery_scan()
                return

        if self.state == NavigationState.RECOVERY_SCAN:
            self._recovery_scan_behavior()
        elif self.state == NavigationState.SEARCH:
            self._search_behavior()
        elif self.state == NavigationState.APPROACH:
            self._approach_behavior()

    # ─────────────────────────────────────────────────────────────────────────

    def _search_behavior(self):
        # Dead end → recovery scan (replaces old simple REVERSE)
        if (self.front_blocked
                and not self.left_clear_for_turn
                and not self.right_clear_for_turn):
            self._start_recovery_scan()
            return

        # Front obstacle — commit turn to clearer side
        if self.front_blocked or self.front_cautious:
            turn_dir = self._choose_turn_direction()
            dir_name = "LEFT" if turn_dir > 0 else "RIGHT"
            self._publish_cmd(0.0, turn_dir * self.turn_speed)
            self._publish_status(
                f"SEARCH — front {self.dist_front:.0f}cm → {dir_name}  "
                f"FL:{self.dist_front_left:.0f} FR:{self.dist_front_right:.0f}"
            )
            return

        # Forward with side nudge
        side_ang = 0.0
        note = ""
        if self.left_wall_close and not self.right_wall_close:
            side_ang = -SIDE_AVOID_ANGULAR
            note = f"  nudge→R(L wall {self.dist_front_left:.0f}cm)"
        elif self.right_wall_close and not self.left_wall_close:
            side_ang = SIDE_AVOID_ANGULAR
            note = f"  nudge→L(R wall {self.dist_front_right:.0f}cm)"

        imu_str = (
            f"  yaw={self.imu_yaw_rate:.3f} corr={self._heading_correction:+.3f}"
            if self.imu_available else ""
        )
        self._publish_cmd(self.fwd_speed, side_ang)
        self._publish_status(
            f"SEARCH — fwd  F:{self.dist_front:.0f} "
            f"FL:{self.dist_front_left:.0f} FR:{self.dist_front_right:.0f}"
            f"{note}{imu_str}"
        )

    # ─────────────────────────────────────────────────────────────────────────

    def _approach_behavior(self):
        # Stop at fire
        if self.fire_direction == "center" and self.dist_front <= self.fire_stop_dist:
            self._publish_cmd(0.0, 0.0)
            self._publish_status(f"🔥 FIRE REACHED — {self.dist_front:.0f}cm")
            self.state = NavigationState.STOPPED
            return

        # Front blocked during approach → recovery scan
        if self.front_blocked:
            if not self.left_clear_for_turn and not self.right_clear_for_turn:
                self._start_recovery_scan()
                return
            turn_dir = self._choose_turn_direction()
            self._publish_cmd(0.0, turn_dir * self.turn_speed)
            self._publish_status(
                f"APPROACH — obstacle, dodge {'LEFT' if turn_dir>0 else 'RIGHT'}"
            )
            return

        # Steer toward fire
        if self.fire_direction == "left":
            self._publish_cmd(SPEED_FORWARD_SLOW, TURN_SPEED_SLOW)
            self._publish_status(f"APPROACH → LEFT  F:{self.dist_front:.0f}cm")
        elif self.fire_direction == "right":
            self._publish_cmd(SPEED_FORWARD_SLOW, -TURN_SPEED_SLOW)
            self._publish_status(f"APPROACH → RIGHT  F:{self.dist_front:.0f}cm")
        else:
            speed    = SPEED_FORWARD_SLOW if self.dist_front < self.caution_dist else self.fwd_speed
            side_ang = 0.0
            if self.left_wall_close and not self.right_wall_close:
                side_ang = -SIDE_AVOID_ANGULAR
            elif self.right_wall_close and not self.left_wall_close:
                side_ang = SIDE_AVOID_ANGULAR
            self._publish_cmd(speed, side_ang)
            self._publish_status(
                f"APPROACH → CENTER  F:{self.dist_front:.0f}cm  spd={speed}"
            )

    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._publish_cmd(0.0, 0.0)
        self.get_logger().info("NavigationNode shutting down")
        super().destroy_node()


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