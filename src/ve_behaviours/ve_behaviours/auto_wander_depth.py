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
import os, math, json, time, xml.etree.ElementTree as ET
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
from ament_index_python.packages import get_package_share_directory


@dataclass
class Target:
    x: float
    y: float
    z: float = 0.0


@dataclass
class StaticObstacle:
    name: str
    x: float
    y: float
    radius: float


def _parse_pose_xyz(pose_text: str) -> Tuple[float, float, float, float]:
    parts = [float(p) for p in (pose_text or "0 0 0 0 0 0").split()]
    parts += [0.0] * (6 - len(parts))
    x, y, z, _, _, yaw = parts[:6]
    return x, y, z, yaw


def _load_includes_from_sdf(world_sdf_path: str) -> List[Tuple[str, str, str]]:
    tree = ET.parse(world_sdf_path)
    root = tree.getroot()
    includes: List[Tuple[str, str, str]] = []
    for inc in root.findall(".//include"):
        name_el = inc.find("name")
        uri_el = inc.find("uri")
        pose_el = inc.find("pose")
        name = (name_el.text if name_el is not None else "").strip()
        uri = (uri_el.text if uri_el is not None else "").strip()
        pose_text = (pose_el.text if pose_el is not None else "0 0 0 0 0 0").strip()
        includes.append((name, uri, pose_text))
    return includes


def _radius_for(name: str, uri: str) -> float:
    if name.startswith(("oak", "pine")):
        return 1.4
    if name.startswith("rock"):
        return 0.8
    if name.startswith("platypus"):
        return 0.6
    if name.startswith("bamboo_thicket"):
        return 0.25
    if name.startswith("forest_wall"):
        return 0.9
    if "crate" in name:
        return 0.5
    return 0.7


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
        self.declare_parameter("world_name", "large_demo")

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

        # ESCAPE manoeuvre after scan / watchdog
        self.declare_parameter("escape_backup_time", 3.5)    # quicker backup (was 3.5)
        self.declare_parameter("escape_backup_speed", 0.30)  # m/s reverse
        self.declare_parameter("escape_turn_time", 2.0)      # seconds pivot
        self.declare_parameter("escape_turn_min_time", 0.8)  # ensure we pivot long enough
        self.declare_parameter("escape_turn_speed", 2.0)     # rad/s
        self.declare_parameter("escape_min_clear_m", 1.20)   # smaller clearance ok (was 1.40)
        
        # Image cropping fraction (controls how much of the frame is kept)
        self.declare_parameter("crop_center_fraction", 0.6)
        self.declare_parameter("use_world_obstacles", True)
        self.declare_parameter("approach_clearance_min", 0.45)

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
        self.escape_turn_min_time: float = float(self.get_parameter("escape_turn_min_time").value)
        self.escape_turn_speed: float = float(self.get_parameter("escape_turn_speed").value)
        self.escape_min_clear_m: float = float(self.get_parameter("escape_min_clear_m").value)

        self.crop_center_fraction: float = float(self.get_parameter("crop_center_fraction").value)
        self.world_name: str = str(self.get_parameter("world_name").value)
        self.use_world_obstacles: bool = bool(self.get_parameter("use_world_obstacles").value)
        self.approach_clearance_min: float = float(self.get_parameter("approach_clearance_min").value)

        os.makedirs(os.path.expanduser("~/.ros/plant_scans"), exist_ok=True)
        self.scans_dir = os.path.expanduser("~/.ros/plant_scans")

        # ---------- State ----------
        self.world_obstacles: List[StaticObstacle] = (
            self._load_world_obstacles(self.world_name) if self.use_world_obstacles else []
        )
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
        self._scan_candidates: List[Tuple[float, float, float, float, float]] = []
        self._active_scan: Optional[Tuple[float, float, float, float, float]] = None

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
        self._escape_turn_start: float = 0.0

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

    def _load_world_obstacles(self, world_name: str) -> List[StaticObstacle]:
        obstacles: List[StaticObstacle] = []
        try:
            bringup_share = get_package_share_directory("41068_ignition_bringup")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not locate bringup package: {e}")
            return obstacles

        world_sdf_path = os.path.join(bringup_share, "worlds", f"{world_name}.sdf")
        if not os.path.isfile(world_sdf_path):
            self.get_logger().warn(f"⚠️ World SDF not found for obstacle map: {world_sdf_path}")
            return obstacles

        try:
            includes = _load_includes_from_sdf(world_sdf_path)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed parsing world SDF: {e}")
            return obstacles

        for name, uri, pose_text in includes:
            if name == "forest_plane":
                continue
            x, y, _, _ = _parse_pose_xyz(pose_text)
            radius = _radius_for(name, uri)
            if radius <= 0.0:
                continue
            if name.startswith("rock"):
                radius += 0.5
            elif name.startswith("bamboo_thicket"):
                radius += 0.15
            obstacles.append(StaticObstacle(name=name, x=x, y=y, radius=radius))

        if obstacles:
            self.get_logger().info(f"Loaded {len(obstacles)} static obstacles for approach planning.")
        return obstacles

    def _clearance_at(self, x: float, y: float, target: Optional[Target]) -> float:
        clearance = float("inf")
        for obs in self.world_obstacles:
            if target and obs.name.startswith("bamboo_thicket"):
                if math.hypot(obs.x - target.x, obs.y - target.y) < 0.2:
                    continue
            dist = math.hypot(x - obs.x, y - obs.y) - obs.radius
            clearance = dist if dist < clearance else clearance
        for other in self.targets_raw:
            if target is not None and other is target:
                continue
            dist = math.hypot(x - other.x, y - other.y) - 0.3
            clearance = dist if dist < clearance else clearance
        return clearance

    def _plan_scan_candidates(self, target: Target, robot_xy: Tuple[float, float]) -> List[Tuple[float, float, float, float, float]]:
        base_dx = target.x - robot_xy[0]
        base_dy = target.y - robot_xy[1]
        base_angle = math.atan2(base_dy, base_dx) if base_dx or base_dy else 0.0
        angle_offsets = [0.0, math.radians(25.0), math.radians(-25.0),
                         math.radians(45.0), math.radians(-45.0),
                         math.radians(90.0), math.radians(-90.0),
                         math.radians(135.0), math.radians(-135.0),
                         math.pi]
        extra_standoffs = [0.0, 0.25, 0.45]

        candidates: List[Tuple[float, float, float, float, float, float]] = []
        for extra in extra_standoffs:
            standoff = max(0.3, self.standoff_m + extra)
            for offset in angle_offsets:
                ang = angle_wrap(base_angle + offset)
                ux, uy = math.cos(ang), math.sin(ang)
                sx = target.x - ux * standoff
                sy = target.y - uy * standoff
                clearance = self._clearance_at(sx, sy, target)
                if math.isfinite(clearance) and clearance < self.approach_clearance_min:
                    continue
                align_penalty = abs(offset)
                distance_penalty = extra * 0.6
                base_score = clearance if math.isfinite(clearance) else 5.0
                score = base_score - 0.2 * align_penalty - distance_penalty
                candidates.append((score, sx, sy, ang, clearance, standoff))

        if not candidates:
            standoff = self.standoff_m + 0.4
            ang = base_angle
            ux, uy = math.cos(ang), math.sin(ang)
            sx = target.x - ux * standoff
            sy = target.y - uy * standoff
            clearance = self._clearance_at(sx, sy, target)
            candidates.append((-999.0, sx, sy, ang, clearance, standoff))

        candidates.sort(key=lambda c: c[0], reverse=True)
        return [(sx, sy, ang, clearance, standoff) for _, sx, sy, ang, clearance, standoff in candidates]

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
        prev = self.run_enabled
        self.run_enabled = bool(msg.data)
        if self.throttle("re", 1.0):
            self.get_logger().info(f"{self.run_enabled_topic} = {self.run_enabled}")
        if not self.run_enabled:
            self._enter_pause_state()
            return
        if not prev and self.run_enabled and self.state == "PAUSED":
            self.get_logger().info("▶️ Resuming auto_wander_depth from pause.")
            self._resume_after_pause()
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
            # Keep full frame; we crop later where needed
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
        self._scan_candidates.clear()
        self._active_scan = None

        for t in (self.timeout_timer, self.scan_timer, self.align_timer, self.scan_gate_timer, self.escape_timer):
            if t:
                t.cancel()
        self.timeout_timer = self.scan_timer = self.align_timer = self.scan_gate_timer = self.escape_timer = None
        self._escape_phase = ""
        self._pending_after_escape = None
        self._escape_turn_start = 0.0

        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

        self._publish_stop()
        self.get_logger().info("🔄 Restarted scanner (targets preserved).")
        return Trigger.Response(success=True, message="Restarted.")

    def _enter_pause_state(self) -> None:
        # stop motion immediately
        self._publish_stop()
        # cancel active timers
        for t in (self.timeout_timer, self.scan_timer, self.align_timer,
                  self.scan_gate_timer, self.escape_timer):
            if t:
                t.cancel()
        self.timeout_timer = self.scan_timer = self.align_timer = self.scan_gate_timer = self.escape_timer = None
        # cancel Nav2 goal if active
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None
        # push current target back for later
        if self.current is not None:
            self.remaining_targets.insert(0, self.current)
            self.current = None
        self.scan_point = None
        self._scan_candidates.clear()
        self._active_scan = None
        self._escape_turn_start = 0.0
        self.state = "PAUSED"

    def _resume_after_pause(self) -> None:
        self._publish_stop()
        if self.remaining_targets:
            self.state = "MOVE"
            self._move_to_next_target()
        else:
            self.state = "IDLE"

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
        candidates = self._plan_scan_candidates(next_target, (rx, ry))
        chosen = candidates[0]
        self._scan_candidates = candidates[1:]
        sx, sy, approach_ang, clearance, standoff = chosen
        self.scan_point = (sx, sy)
        self._active_scan = chosen

        dist = math.hypot(gx - rx, gy - ry)
        if self.throttle("mv", 0.5):
            clr_txt = f"{clearance:.2f}m" if math.isfinite(clearance) else "∞"
            self.get_logger().info(
                f"🎯 Target ({gx:.2f},{gy:.2f}); scan=({sx:.2f},{sy:.2f}) standoff={standoff:.2f} "
                f"clearance={clr_txt}; d={dist:.2f}m"
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
            if self._scan_candidates:
                next_scan = self._scan_candidates.pop(0)
                sx, sy, ang, clearance, standoff = next_scan
                self.scan_point = (sx, sy)
                self._active_scan = next_scan
                if self.throttle("goal_retry", 1.5):
                    clr_txt = f"{clearance:.2f}m" if math.isfinite(clearance) else "∞"
                    self.get_logger().info(
                        f"↻ Trying alternate scan point ({sx:.2f},{sy:.2f}) "
                        f"standoff={standoff:.2f} clearance={clr_txt}"
                    )
                self._send_nav_goal(sx, sy, face_x=self.current.x, face_y=self.current.y)
            elif self.scan_point is not None:
                sx, sy = self.scan_point
                self._send_nav_goal(sx, sy, face_x=self.current.x, face_y=self.current.y)
            else:
                self._move_to_next_target()

    def _check_scan_timeout(self):
        if self.state == "MOVE" and self.current:
            gx, gy = self.current.x, self.current.y
            rx, ry = self.robot_xy
            dist = math.hypot(gx - rx, gy - ry)
            active_standoff = self._active_scan[4] if self._active_scan else self.standoff_m
            if dist <= active_standoff * 2.5:
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
        self._scan_candidates.clear()

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
        self._escape_turn_start = 0.0

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
                self._escape_turn_start = now
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

            elapsed_turn = now - self._escape_turn_start if self._escape_turn_start else 0.0
            front_clear = (r_front >= self.escape_min_clear_m)
            min_turn_reached = elapsed_turn >= self.escape_turn_min_time
            if now >= self._escape_end_time or (front_clear and min_turn_reached):
                # finish escape
                self._publish_stop()
                self._escape_phase = ""
                if self.escape_timer:
                    self.escape_timer.cancel()
                    self.escape_timer = None
                self._escape_turn_start = 0.0
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
            label, conf, path, health_percent = "green", 0.0, "", 100
        else:
            # Crop to central region so we mostly see the target plant
            frame_cropped = self._crop_central_roi(frame)

            # Colour analysis on the cropped region only
            hsv = cv2.cvtColor(frame_cropped, cv2.COLOR_BGR2HSV)

            # Green / yellow / red masks (basic HSV buckets)
            green = cv2.inRange(hsv, np.array([35, 60, 60]), np.array([85, 255, 255]))
            yellow = cv2.inRange(hsv, np.array([20, 80, 60]), np.array([35, 255, 255]))
            red = cv2.inRange(hsv, np.array([0, 90, 60]), np.array([10, 255, 255])) | \
                  cv2.inRange(hsv, np.array([170, 90, 60]), np.array([180, 255, 255]))

            g_area = int(np.count_nonzero(green))
            y_area = int(np.count_nonzero(yellow))
            r_area = int(np.count_nonzero(red))
            total = g_area + y_area + r_area

            if total == 0:
                green_ratio = 1.0
                yellow_ratio = 0.0
                red_ratio = 0.0
            else:
                green_ratio = g_area / float(total)
                yellow_ratio = y_area / float(total)
                red_ratio = r_area / float(total)

            red_thresh = 0.30
            yellow_thresh = 0.30

            # Decision rules: require ~30% coverage for red/yellow
            if red_ratio >= red_thresh:
                label = "red"
                conf = red_ratio
            elif yellow_ratio >= yellow_thresh:
                label = "yellow"
                conf = yellow_ratio
            else:
                label = "green"
                conf = green_ratio

            health_percent = max(0, min(100, int(round(green_ratio * 100))))

            if self.throttle("class_dbg", 1.0):
                self.get_logger().info(
                    "classify: g=%d (%.2f)  y=%d (%.2f)  r=%d (%.2f) -> %s"
                    % (g_area, green_ratio, y_area, yellow_ratio, r_area, red_ratio, label)
                )

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
            "condition": (
                "toxic" if label == "red"
                else "stressed" if label == "yellow"
                else "healthy"
            ),
            "health_percent": health_percent,
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
