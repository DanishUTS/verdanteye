#!/usr/bin/env python3
# ===============================================================
# VerdantEye: auto_wander_depth (RGB + LiDAR, Nav2-integrated)
# - Drives to a standoff point facing the plant
# - LiDAR watchdog prevents bumper-kisses, creep-in if short
# - Only scans when yaw aligned + plant is in LiDAR window
# - Post-scan ESCAPE: back up + pivot toward clearer side
# - UI image + JSON checklist
# - UPDATED: classify & save only central crop (target region)
# ===============================================================

from __future__ import annotations
import os, math, json, time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

import numpy as np
import cv2
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Twist
from nav2_msgs.action import NavigateToPose


@dataclass
class Target:
    x: float
    y: float
    z: float = 0.0


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw from quaternion (Z axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class AutoWanderDepth(Node):
    def __init__(self) -> None:
        super().__init__("auto_wander_depth")

        # ---------- Parameters ----------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("scan_topic", "/scan")  # LiDAR
        self.declare_parameter("targets_topic", "/bamboo/targets")
        self.declare_parameter("run_enabled_topic", "/ui/run_enabled")
        self.declare_parameter("restart_srv", "/ui/restart")
        self.declare_parameter("nav_goal_action", "navigate_to_pose")
        self.declare_parameter("goal_frame", "map")  # set to 'odom' if no localization
        self.declare_parameter("standoff_m", 0.5)

        # LiDAR/safety & alignment (SAFER DEFAULTS)
        self.declare_parameter("safety_stop_m", 0.55)        # safer (was 0.45)
        self.declare_parameter("front_sector_deg", 45.0)     # wider sector
        self.declare_parameter("scan_min_m", 0.50)           # min scan distance
        self.declare_parameter("scan_max_m", 1.30)           # wider window (was 0.90)
        self.declare_parameter("align_kp", 1.6)
        self.declare_parameter("max_ang_vel", 2.1)
        self.declare_parameter("creep_lin_vel", 0.04)        # slower creep (was 0.09)
        self.declare_parameter("creep_allow", True)
        self.declare_parameter("yaw_ok_deg", 3.0)            # tighter yaw (was 7.0)

        # UI / timing
        self.declare_parameter("scan_time_sec", 2.5)         # shorter scan window (was 3.2)
        self.declare_parameter("publish_stop_twist", True)
        self.declare_parameter("align_timeout_sec", 4.0)
        self.declare_parameter("scan_gate_timeout_sec", 6.0)

        # ESCAPE maneuver after scan / watchdog
        self.declare_parameter("escape_backup_time", 3.5)    # quicker backup (was 3.5)
        self.declare_parameter("escape_backup_speed", 0.30)  # m/s reverse
        self.declare_parameter("escape_turn_time", 2.0)      # seconds pivot
        self.declare_parameter("escape_turn_speed", 2.0)     # rad/s
        self.declare_parameter("escape_min_clear_m", 1.20)   # smaller clearance ok (was 1.40)
        
        # Image cropping fraction (controls how much of the frame is kept)
        self.declare_parameter("crop_center_fraction", 0.6)


        # ---------- Read params ----------
        self.cmd_vel_topic: str = self.get_parameter("cmd_vel_topic").value
        self.odom_topic: str = self.get_parameter("odom_topic").value
        self.camera_topic: str = self.get_parameter("camera_topic").value
        self.scan_topic: str = self.get_parameter("scan_topic").value
        self.targets_topic: str = self.get_parameter("targets_topic").value
        self.run_enabled_topic: str = self.get_parameter("run_enabled_topic").value
        self.restart_srv: str = self.get_parameter("restart_srv").value
        self.nav_goal_action: str = self.get_parameter("nav_goal_action").value
        self.goal_frame: str = self.get_parameter("goal_frame").value

        self.standoff_m: float = float(self.get_parameter("standoff_m").value)
        self.safety_stop_m: float = float(self.get_parameter("safety_stop_m").value)
        self.front_sector_deg: float = float(self.get_parameter("front_sector_deg").value)
        self.scan_min_m: float = float(self.get_parameter("scan_min_m").value)
        self.scan_max_m: float = float(self.get_parameter("scan_max_m").value)
        self.align_kp: float = float(self.get_parameter("align_kp").value)
        self.max_ang_vel: float = float(self.get_parameter("max_ang_vel").value)
        self.creep_lin_vel: float = float(self.get_parameter("creep_lin_vel").value)
        self.creep_allow: bool = bool(self.get_parameter("creep_allow").value)
        self.yaw_ok_deg: float = float(self.get_parameter("yaw_ok_deg").value)
        self.scan_time_sec: float = float(self.get_parameter("scan_time_sec").value)
        self.publish_stop_twist: bool = bool(self.get_parameter("publish_stop_twist").value)
        self.align_timeout_sec: float = float(self.get_parameter("align_timeout_sec").value)
        self.scan_gate_timeout_sec: float = float(self.get_parameter("scan_gate_timeout_sec").value)

        self.escape_backup_time: float = float(self.get_parameter("escape_backup_time").value)
        self.escape_backup_speed: float = float(self.get_parameter("escape_backup_speed").value)
        self.escape_turn_time: float = float(self.get_parameter("escape_turn_time").value)
        self.escape_turn_speed: float = float(self.get_parameter("escape_turn_speed").value)
        self.escape_min_clear_m: float = float(self.get_parameter("escape_min_clear_m").value)

        self.crop_center_fraction: float = float(self.get_parameter("crop_center_fraction").value)

        os.makedirs(os.path.expanduser("~/.ros/plant_scans"), exist_ok=True)
        self.scans_dir = os.path.expanduser("~/.ros/plant_scans")

        # ---------- State ----------
        self.bridge = CvBridge()
        self.rgb: Optional[np.ndarray] = None
        self.robot_xy: Tuple[float, float] = (0.0, 0.0)
        self.robot_yaw: float = 0.0

        self.targets_raw: List[Target] = []
        self.remaining_targets: List[Target] = []
        self.current: Optional[Target] = None
        self.scan_point: Optional[Tuple[float, float]] = None
        self.results: List[dict] = []
        self.state: str = "IDLE"
        self.run_enabled: bool = False

        self.goal_handle = None
        self.timeout_timer = None
        self.scan_timer = None
        self.align_timer = None
        self.scan_gate_timer = None
        self.watchdog_timer = None
        self.escape_timer = None
        self._escape_phase: str = ""   # "", "BACK", "TURN"
        self._escape_end_time: float = 0.0
        self._pending_after_escape: Optional[str] = None  # next state hint

        # LiDAR cache
        self.scan_msg: Optional[LaserScan] = None
        self.last_scan_time: float = 0.0

        # throttle cache
        self._last_log: dict[str, float] = {}

        # ---------- Pub/Sub ----------
        qos_targets = QoSProfile(depth=1)
        qos_targets.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_targets.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(PoseArray, self.targets_topic, self.on_targets, qos_targets)
        self.create_subscription(Bool, self.run_enabled_topic, self.on_run_enabled, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(Image, self.camera_topic, self.on_rgb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)

        self.pub_ui = self.create_publisher(Image, "/plant_scan/ui", 1)
        self.pub_list = self.create_publisher(String, "/plant_scan/checklist", 1)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.srv_restart = self.create_service(Trigger, self.restart_srv, self.on_restart)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_goal_action)

        # LiDAR watchdog @ 10 Hz
        self.watchdog_timer = self.create_timer(0.1, self._lidar_watchdog)

        self.get_logger().info("auto_wander_depth ready: RGB+LiDAR, Nav2-integrated (target-crop mode).")

    # ---------------- Utilities ----------------
    def throttle(self, key: str, sec: float) -> bool:
        now = time.time()
        last = self._last_log.get(key, 0.0)
        if now - last >= sec:
            self._last_log[key] = now
            return True
        return False

    def _scan_window_indices(self, start_ang: float, end_ang: float) -> Optional[Tuple[int, int]]:
        scan = self.scan_msg
        if not scan or not scan.ranges:
            return None
        n = len(scan.ranges)
        idx_start = max(0, int((start_ang - scan.angle_min) / scan.angle_increment))
        idx_end = min(n - 1, int((end_ang - scan.angle_min) / scan.angle_increment))
        return (idx_start, idx_end) if idx_end >= idx_start else None

    def _range_min_in(self, start_deg: float, end_deg: float) -> Optional[float]:
        scan = self.scan_msg
        if not scan:
            return None
        idx = self._scan_window_indices(math.radians(start_deg), math.radians(end_deg))
        if not idx:
            return None
        s, e = idx
        vals = [r for r in scan.ranges[s:e + 1] if np.isfinite(r) and r > 0.0]
        return min(vals) if vals else None

    def front_min_range(self) -> Optional[float]:
        # +/- sector around forward (assume 0 rad forward)
        half = float(self.front_sector_deg)
        return self._range_min_in(-half, +half)

    def side_clearance(self) -> Tuple[Optional[float], Optional[float]]:
        # Look at left/right wedges to decide turn direction for escape
        # Right: [-100, -40], Left: [40, 100] degrees
        rmin = self._range_min_in(-100.0, -40.0)
        lmin = self._range_min_in(+40.0, +100.0)
        return lmin, rmin

    # ---------------- Callbacks ----------------
    def on_run_enabled(self, msg: Bool) -> None:
        self.run_enabled = bool(msg.data)
        if self.throttle("re", 1.0):
            self.get_logger().info(f"{self.run_enabled_topic} = {self.run_enabled}")
        if self.run_enabled and self.targets_raw and not self.remaining_targets:
            self.remaining_targets = self.targets_raw.copy()
            self._move_to_next_target()

    def on_targets(self, msg: PoseArray) -> None:
        self.targets_raw = [Target(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self.get_logger().info(f"Received {len(self.targets_raw)} bamboo targets.")
        if self.run_enabled and not self.remaining_targets:
            self.remaining_targets = self.targets_raw.copy()
            self._move_to_next_target()

    def on_rgb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # --- NEW: crop to central 60 % of the frame ---
            frame = self._crop_central_roi(frame)
            self.rgb = frame
        except Exception as e:
            if self.throttle("rgb_fail", 2.0):
                self.get_logger().warn(f"RGB conversion failed: {e}")
            self.rgb = None


    def on_odom(self, msg: Odometry) -> None:
        self.robot_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        q = msg.pose.pose.orientation
        self.robot_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg
        self.last_scan_time = self.get_clock().now().nanoseconds / 1e9

    def on_restart(self, _req: Trigger.Request, _ctx=None) -> Trigger.Response:
        self.run_enabled = False
        self.state = "IDLE"
        self.current = None
        self.scan_point = None
        self.results.clear()
        self.remaining_targets.clear()

        for t in (self.timeout_timer, self.scan_timer, self.align_timer, self.scan_gate_timer, self.escape_timer):
            if t:
                t.cancel()
        self.timeout_timer = self.scan_timer = self.align_timer = self.scan_gate_timer = self.escape_timer = None
        self._escape_phase = ""
        self._pending_after_escape = None

        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

        self._publish_stop()
        self.get_logger().info("🔄 Restarted scanner (targets preserved).")
        return Trigger.Response(success=True, message="Restarted.")

    # ---------------- Nav2 drive -> scan ----------------
    def _move_to_next_target(self) -> None:
        if not self.remaining_targets:
            self.state = "DONE"
            self._publish_checklist()
            self.get_logger().info("✅ All targets visited.")
            return

        rx, ry = self.robot_xy
        next_target = min(self.remaining_targets, key=lambda t: math.hypot(t.x - rx, t.y - ry))
        self.remaining_targets.remove(next_target)
        self.current = next_target
        self.state = "MOVE"

        gx, gy = next_target.x, next_target.y
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy) or 1e-6
        ux, uy = dx / dist, dy / dist
        sx, sy = gx - ux * self.standoff_m, gy - uy * self.standoff_m
        self.scan_point = (sx, sy)

        if self.throttle("mv", 0.5):
            self.get_logger().info(
                f"🎯 Target ({gx:.2f},{gy:.2f}); scan point=({sx:.2f},{sy:.2f}); d={dist:.2f}m"
            )
        self._send_nav_goal(sx, sy, face_x=gx, face_y=gy)

    def _send_nav_goal(self, gx: float, gy: float,
                       face_x: Optional[float] = None,
                       face_y: Optional[float] = None) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("⚠️ Nav2 action server unavailable.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        yaw = math.atan2((face_y - gy) if face_y is not None else 0.0,
                         (face_x - gx) if face_x is not None else 1.0)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

        if self.timeout_timer:
            self.timeout_timer.cancel()
        self.timeout_timer = self.create_timer(15.0, self._check_scan_timeout)

    def _on_goal_sent(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle or not self.goal_handle.accepted:
            self.get_logger().warn("❌ Nav2 goal rejected; skipping.")
            self.goal_handle = None
            self._move_to_next_target()
            return
        self.goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        if not self.current:
            return
        result = future.result()
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        self.goal_handle = None

        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            self._enter_scan_mode()
        else:
            if self.throttle("goal_fail", 1.0):
                self.get_logger().warn(f"Goal failed (status={getattr(result, 'status', '?')}); retrying scan point.")
            if self.scan_point is not None:
                sx, sy = self.scan_point
                self._send_nav_goal(sx, sy, face_x=self.current.x, face_y=self.current.y)
            else:
                self._move_to_next_target()

    def _check_scan_timeout(self):
        if self.state == "MOVE" and self.current:
            gx, gy = self.current.x, self.current.y
            rx, ry = self.robot_xy
            dist = math.hypot(gx - rx, gy - ry)
            if dist <= self.standoff_m * 2.5:
                if self.throttle("near_force", 2.0):
                    self.get_logger().warn(f"⏳ Timeout near plant ({dist:.2f} m) — forcing scan/align.")
                self._enter_scan_mode()

    # ---------------- SCAN ----------------
    def _enter_scan_mode(self):
        # cancel any active goal
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

        self._publish_stop()
        self.state = "ALIGN"

        # Begin yaw alignment loop, then open a scan gate
        self._start_align_loop()
        # In parallel, open a scan gate timer that will end alignment window
        if self.scan_gate_timer:
            self.scan_gate_timer.cancel()
        self.scan_gate_timer = self.create_timer(self.scan_gate_timeout_sec, self._force_scan)

    def _start_align_loop(self):
        # 20 Hz small controller to face the plant
        if self.align_timer:
            self.align_timer.cancel()
        self._align_start_time = time.time()
        self.align_timer = self.create_timer(0.05, self._align_step)

    def _align_step(self):
        if not self.current:
            return
        # desired yaw toward plant
        des = math.atan2(self.current.y - self.robot_xy[1], self.current.x - self.robot_xy[0])
        err = angle_wrap(des - self.robot_yaw)
        yaw_ok = abs(math.degrees(err)) <= self.yaw_ok_deg

        # stop if timeout
        if time.time() - getattr(self, "_align_start_time", 0.0) > self.align_timeout_sec:
            if self.throttle("align_to", 1.0):
                self.get_logger().warn("Alignment timeout – proceeding.")
            self._open_scan_gate()
            return

        # simple P controller on yaw, no linear motion while aligning
        cmd = Twist()
        cmd.angular.z = clamp(self.align_kp * err, -self.max_ang_vel, self.max_ang_vel)
        self.pub_cmd.publish(cmd)

        if yaw_ok:
            self._open_scan_gate()

    def _open_scan_gate(self):
        # stop turning and decide whether to creep
        self._publish_stop()
        if self.align_timer:
            self.align_timer.cancel()
            self.align_timer = None

        self.state = "SCAN"
        r = self.front_min_range()
        # Optional creep if we're too far but close to window
        if self.creep_allow and r is not None and r > self.scan_max_m and r < 3.0:
            # creep forward until in window or safety trips
            cmd = Twist()
            cmd.linear.x = self.creep_lin_vel
            self.pub_cmd.publish(cmd)
        else:
            self._publish_stop()

        # Start periodic check for LiDAR window to actually capture
        if self.scan_timer:
            self.scan_timer.cancel()
        self._scan_start_time = time.time()
        self.scan_timer = self.create_timer(0.05, self._scan_gate_check)

    def _scan_gate_check(self):
        r = self.front_min_range()
        yaw_ok = False
        if self.current:
            des = math.atan2(self.current.y - self.robot_xy[1], self.current.x - self.robot_xy[0])
            yaw_ok = abs(math.degrees(angle_wrap(des - self.robot_yaw))) <= self.yaw_ok_deg

        in_window = (r is not None) and (self.scan_min_m <= r <= self.scan_max_m)

        # capture if both ready OR timeout fallthrough
        timed_out = (time.time() - getattr(self, "_scan_start_time", 0.0)) > self.scan_time_sec
        if (yaw_ok and in_window) or timed_out:
            self._publish_stop()
            if self.scan_timer:
                self.scan_timer.cancel()
                self.scan_timer = None
            if self.scan_gate_timer:
                self.scan_gate_timer.cancel()
                self.scan_gate_timer = None
            # actually capture
            self._finish_scan()
        else:
            # keep creeping if too far
            if self.creep_allow and r is not None and r > self.scan_max_m and self.state == "SCAN":
                cmd = Twist()
                cmd.linear.x = self.creep_lin_vel
                self.pub_cmd.publish(cmd)
            else:
                # otherwise hold
                self._publish_stop()

    def _force_scan(self):
        # safety to ensure we don’t get stuck forever
        if self.state in ("ALIGN", "SCAN"):
            if self.throttle("force_scan", 1.0):
                self.get_logger().warn("Scan gate timeout – capturing anyway.")
            if self.scan_timer:
                self.scan_timer.cancel()
                self.scan_timer = None
            self._publish_stop()
            self._finish_scan()

    def _finish_scan(self):
        frame = self.rgb.copy() if self.rgb is not None else None
        gx = self.current.x if self.current else 0.0
        gy = self.current.y if self.current else 0.0
        self._analyze_and_record(frame, gx, gy)

        # After capture, perform ESCAPE so we don't get stuck turning in place
        self.state = "ESCAPE"
        self._start_escape(pending_next="MOVE")  # after escape, resume MOVE/next target

    # ---------------- ESCAPE ----------------
    def _start_escape(self, pending_next: Optional[str] = None):
        """Back up a bit, then pivot toward the side with more clearance."""
        # Cancel align/scan timers if any remain
        for t in (self.align_timer, self.scan_timer, self.scan_gate_timer):
            if t:
                t.cancel()
        self.align_timer = self.scan_timer = self.scan_gate_timer = None

        self._pending_after_escape = pending_next
        self._escape_phase = "BACK"
        self._escape_end_time = time.time() + self.escape_backup_time

        # begin periodic escape step
        if self.escape_timer:
            self.escape_timer.cancel()
        self.escape_timer = self.create_timer(0.05, self._escape_step)
        if self.throttle("esc_start", 1.0):
            self.get_logger().info("↩️ ESCAPE: backing up from plant...")

    def _escape_step(self):
        r_front = self.front_min_range() or float("inf")
        now = time.time()

        if self._escape_phase == "BACK":
            # Reverse until timer expires
            cmd = Twist()
            cmd.linear.x = -abs(self.escape_backup_speed)
            # If we're dangerously close, also add slight turn away from closest side
            lmin, rmin = self.side_clearance()
            if np.isfinite(r_front) and r_front < self.safety_stop_m:
                # pick sign so that we turn toward more open side during back-up
                turn_right_better = (rmin or 0.0) > (lmin or 0.0)
                cmd.angular.z = +0.3 if turn_right_better else -0.3
            self.pub_cmd.publish(cmd)

            if now >= self._escape_end_time:
                # pick turn direction based on side clearance
                lmin, rmin = self.side_clearance()
                turn_right = (rmin or 0.0) > (lmin or 0.0)
                self._escape_turn_sign = +1.0 if turn_right else -1.0
                self._escape_phase = "TURN"
                self._escape_end_time = now + self.escape_turn_time
                if self.throttle("esc_turn", 1.0):
                    self.get_logger().info(f"↪️ ESCAPE: pivoting {'right' if turn_right else 'left'}...")
                # fallthrough will execute TURN next tick
            return

        if self._escape_phase == "TURN":
            # Pivot in place; stop early if front is clear enough
            cmd = Twist()
            cmd.angular.z = self._escape_turn_sign * abs(self.escape_turn_speed)
            self.pub_cmd.publish(cmd)

            front_clear = (r_front >= self.escape_min_clear_m)
            if now >= self._escape_end_time or front_clear:
                # finish escape
                self._publish_stop()
                self._escape_phase = ""
                if self.escape_timer:
                    self.escape_timer.cancel()
                    self.escape_timer = None
                # Resume workflow
                next_state = self._pending_after_escape or "MOVE"
                if next_state == "MOVE":
                    self.current = None
                    self.scan_point = None
                    self.state = "MOVE"
                    self._move_to_next_target()
                else:
                    self.state = next_state
            return

    # ---------------- LiDAR Watchdog ----------------
    def _lidar_watchdog(self):
        # Runs all the time. If anything is dangerously close ahead, full stop and escape.
        r = self.front_min_range()
        if r is not None and r < self.safety_stop_m:
            self._publish_stop()
            if self.throttle("wd", 0.5):
                self.get_logger().warn(f"🛑 Safety stop: obstacle at {r:.2f} m — initiating ESCAPE.")
            # If a nav goal is active, cancel it to prevent tug-of-war
            if self.goal_handle is not None:
                try:
                    self.goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.goal_handle = None
            if self.state != "ESCAPE":
                # Escape, then return to MOVE/ALIGN depending on prior state
                resume = "MOVE" if self.current is None else "ALIGN"
                self.state = "ESCAPE"
                self._start_escape(pending_next=resume)

    # ---------------- Perception + UI ----------------
    def _crop_central_roi(self, frame: np.ndarray) -> np.ndarray:
        """
        Return a central crop of the image based on crop_center_fraction.

        If crop_center_fraction = 0.6, we keep the central 60% of width & height.
        """
        frac = clamp(self.crop_center_fraction, 0.1, 1.0)  # safety clamp
        h, w = frame.shape[:2]
        cw, ch = int(w * frac), int(h * frac)

        # centre the crop
        x0 = max(0, (w - cw) // 2)
        y0 = max(0, (h - ch) // 2)
        x1 = min(w, x0 + cw)
        y1 = min(h, y0 + ch)

        return frame[y0:y1, x0:x1]

    def _analyze_and_record(self, frame: Optional[np.ndarray], gx: float, gy: float) -> None:
        if frame is None:
            label, conf, path = "green", 0.0, ""
        else:
            # Crop to central region so we mostly see the target plant
            frame_cropped = self._crop_central_roi(frame)

            # Colour analysis on the cropped region only
            hsv = cv2.cvtColor(frame_cropped, cv2.COLOR_BGR2HSV)
            green = cv2.inRange(hsv, np.array([35, 60, 60]), np.array([85, 255, 255]))
            red = cv2.inRange(hsv, np.array([0, 90, 60]), np.array([10, 255, 255])) | \
                  cv2.inRange(hsv, np.array([170, 90, 60]), np.array([180, 255, 255]))

            g_area, r_area = int(np.count_nonzero(green)), int(np.count_nonzero(red))
            total = g_area + r_area or 1

            label = "red" if r_area > g_area else "green"
            conf = max(r_area, g_area) / float(total)

            idx = len(self.results) + 1
            path = os.path.join(self.scans_dir, f"{idx:02d}_{label}.png")
            try:
                # Save the cropped image, not the full scene
                cv2.imwrite(path, frame_cropped)
            except Exception:
                path = ""

        entry = {
            "id": len(self.results) + 1,
            "x": round(float(gx), 3),
            "y": round(float(gy), 3),
            "color": label,
            "confidence": round(float(conf), 3),
            "condition": "toxic" if label == "red" else "healthy",
            "image_path": path
        }
        self.results.append(entry)
        self._publish_checklist()

    def _publish_checklist(self) -> None:
        msg = {"visited": len(self.results), "items": self.results}
        self.pub_list.publish(String(data=json.dumps(msg)))
        self._render_ui()

    def _render_ui(self) -> None:
        canvas = np.full((360, 480, 3), 25, np.uint8)
        cv2.putText(canvas, f"STATE: {self.state}", (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(canvas, f"Visited: {len(self.results)}", (16, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
        if self.rgb is not None:
            # Show cropped view in the UI thumbnail as well
            roi = self._crop_central_roi(self.rgb)
            thumb = cv2.resize(roi, (160, 120))
            canvas[200:320, 300:460] = thumb
        try:
            self.pub_ui.publish(self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
        except Exception:
            pass

    # ---------------- Helpers ----------------
    def _publish_stop(self):
        if self.publish_stop_twist:
            self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = AutoWanderDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
