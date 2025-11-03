#!/usr/bin/env python3
# ===============================================================
# VerdantEye: AutoTargetScan (Nav2 event-driven, camera-based scan)
# - Uses Husky RGB camera to classify each plant (red / green)
# - Faces the plant at the scan point (Nav2 goal yaw)
# - Non-blocking timed scan (no time.sleep)
# - Keeps bamboo targets across "Restart" & logs coordinates
# ===============================================================

from __future__ import annotations
import os, math, json
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus

import numpy as np
import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose


@dataclass
class Target:
    x: float
    y: float
    z: float = 0.0


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Convert planar yaw -> quaternion (x,y,z,w)."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class AutoTargetScan(Node):
    def __init__(self) -> None:
        super().__init__("auto_wander_nav2")

        # --- Parameters ---
        self.declare_parameter("scan_time_sec", 3.0)
        self.declare_parameter("scans_dir", os.path.expanduser("~/.ros/plant_scans"))
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("close_distance_m", 0.4)   # for timeout check
        self.declare_parameter("standoff_m", 0.8)

        self.scan_time_sec = float(self.get_parameter("scan_time_sec").value)
        self.scans_dir = str(self.get_parameter("scans_dir").value)
        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.close_distance_m = float(self.get_parameter("close_distance_m").value)
        self.standoff_m = float(self.get_parameter("standoff_m").value)

        os.makedirs(self.scans_dir, exist_ok=True)

        # --- Subscriptions ---
        qos_targets = QoSProfile(depth=1)
        qos_targets.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_targets.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(PoseArray, "/bamboo/targets", self.on_targets, qos_targets)
        self.create_subscription(Bool, "/ui/run_enabled", self.on_run_enabled, 10)
        self.create_subscription(Image, "/camera/color/image_raw", self.on_rgb, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        # --- Publishers & Services ---
        self.pub_ui = self.create_publisher(Image, "/plant_scan/ui", 1)
        self.pub_list = self.create_publisher(String, "/plant_scan/checklist", 1)
        self.srv_restart = self.create_service(Trigger, "/ui/restart", self.on_restart)
        self.bridge = CvBridge()

        # --- State ---
        self.rgb: Optional[np.ndarray] = None
        self.robot_xy = (0.0, 0.0)
        self.targets_raw: List[Target] = []
        self.remaining_targets: List[Target] = []
        self.current: Optional[Target] = None
        self.scan_point: Optional[Tuple[float, float]] = None
        self.results: List[dict] = []
        self.state = "IDLE"
        self.run_enabled = False

        # timers
        self.timeout_timer = None
        self.scan_timer = None

        # --- Nav2 ---
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info("🌿 AutoTargetScan (camera-based, faces plant) ready.")

    # -------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------
    def on_run_enabled(self, msg: Bool) -> None:
        self.run_enabled = msg.data
        self.get_logger().info(f"/ui/run_enabled = {self.run_enabled}")
        if self.run_enabled and self.targets_raw:
            # (Re)start from full list any time Start is pressed
            self.remaining_targets = self.targets_raw.copy()
            self._move_to_next_target()

    def on_targets(self, msg: PoseArray) -> None:
        # Always refresh list – bamboo_randomizer may be re-run
        self.targets_raw = [Target(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self.get_logger().info(f"Received {len(self.targets_raw)} bamboo targets.")
        if self.run_enabled and not self.remaining_targets:
            # If UI is already in RUNNING state, start immediately
            self.remaining_targets = self.targets_raw.copy()
            self._move_to_next_target()

    def on_rgb(self, msg: Image) -> None:
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            self.rgb = None

    def on_odom(self, msg: Odometry) -> None:
        self.robot_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def on_restart(self, _req: Trigger.Request, _ctx=None) -> Trigger.Response:
        # IMPORTANT: keep self.targets_raw so we don't lose bamboo targets
        self.run_enabled = False
        self.state = "IDLE"
        self.current = None
        self.scan_point = None
        self.results.clear()
        self.remaining_targets.clear()

        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        if self.scan_timer:
            self.scan_timer.cancel()
            self.scan_timer = None

        self.get_logger().info("🔄 Restarted scanner (targets preserved).")
        return Trigger.Response(success=True, message="Restarted.")

    # -------------------------------------------------------------
    # Navigation
    # -------------------------------------------------------------
    def _move_to_next_target(self) -> None:
        if not self.remaining_targets:
            self.state = "DONE"
            self.publish_checklist()
            self.get_logger().info("✅ All targets visited.")
            return

        rx, ry = self.robot_xy
        # Pick nearest plant
        next_target = min(self.remaining_targets, key=lambda t: math.hypot(t.x - rx, t.y - ry))
        self.remaining_targets.remove(next_target)
        self.current = next_target
        self.state = "MOVE"

        gx, gy = next_target.x, next_target.y
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy) or 1e-6
        ux, uy = dx / dist, dy / dist

        # Scan point = standoff_m away from plant along robot->plant line
        sx, sy = gx - ux * self.standoff_m, gy - uy * self.standoff_m
        self.scan_point = (sx, sy)

        self.get_logger().info(
            f"🎯 New target #{len(self.results)+1}: "
            f"plant at map ({gx:.2f}, {gy:.2f}), "
            f"scan point at ({sx:.2f}, {sy:.2f}), "
            f"robot at ({rx:.2f}, {ry:.2f}), dist={dist:.2f} m."
        )

        # Go to scan point, facing the plant
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

        # Yaw so robot faces the plant
        if face_x is not None and face_y is not None:
            yaw = math.atan2(face_y - gy, face_x - gx)
        else:
            yaw = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if face_x is not None:
            self.get_logger().info(
                f"Sending Nav2 goal to scan point ({gx:.2f}, {gy:.2f}) "
                f"facing plant ({face_x:.2f}, {face_y:.2f}), yaw={yaw:.2f} rad."
            )
        else:
            self.get_logger().info(
                f"Sending Nav2 goal to ({gx:.2f}, {gy:.2f}) with yaw={yaw:.2f} rad."
            )

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

        # reset / start Nav2 timeout timer
        if self.timeout_timer:
            self.timeout_timer.cancel()
        self.timeout_timer = self.create_timer(12.0, self._check_scan_timeout)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Nav2 goal rejected, skipping to next.")
            self._move_to_next_target()
            return
        goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        """Called when Nav2 reports a goal has finished (success/failure)."""
        if not self.current:
            return

        result = future.result()
        if not result:
            self.get_logger().warn("⚠️ No result from Nav2 goal future.")
            return

        status = result.status
        gx, gy = self.current.x, self.current.y
        rx, ry = self.robot_xy
        dist = math.hypot(gx - rx, gy - ry)

        # Cancel timeout timer (goal finished)
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"✅ Arrived at scan point. Robot=({rx:.2f}, {ry:.2f}), "
                f"plant=({gx:.2f}, {gy:.2f}), dist_to_plant={dist:.2f} m. Starting timed scan."
            )
            self.state = "SCAN"
            self._perform_scan_and_continue()
        else:
            self.get_logger().warn(f"Goal ended with status {status}, retrying scan point.")
            if self.scan_point is not None:
                sx, sy = self.scan_point
                self._send_nav_goal(sx, sy, face_x=gx, face_y=gy)
            else:
                self._move_to_next_target()

    def _check_scan_timeout(self):
        """Force a scan if Nav2 stalls near target for too long."""
        if self.state == "MOVE" and self.current:
            gx, gy = self.current.x, self.current.y
            rx, ry = self.robot_xy
            dist = math.hypot(gx - rx, gy - ry)
            if dist <= self.close_distance_m * 1.5:
                self.get_logger().warn(
                    f"⏳ Timeout near plant at ({gx:.2f}, {gy:.2f}), "
                    f"robot=({rx:.2f}, {ry:.2f}), dist={dist:.2f} m — forcing scan."
                )
                self.state = "SCAN"
                self._perform_scan_and_continue()

    # -------------------------------------------------------------
    # Scanning & UI
    # -------------------------------------------------------------
    def _perform_scan_and_continue(self) -> None:
        """
        Start a non-blocking timed scan: wait scan_time_sec seconds while
        camera frames keep updating, then analyze the latest frame.
        """
        if not self.current:
            self.get_logger().warn("SCAN requested but no current target.")
            return

        gx, gy = self.current.x, self.current.y
        rx, ry = self.robot_xy
        self.get_logger().info(
            f"📸 Capturing camera for {self.scan_time_sec:.1f}s at "
            f"robot=({rx:.2f}, {ry:.2f}) looking at plant=({gx:.2f}, {gy:.2f})."
        )

        # cancel any existing scan timer
        if self.scan_timer:
            self.scan_timer.cancel()
            self.scan_timer = None

        # one-shot timer: after scan_time_sec, _finish_scan() will run
        self.scan_timer = self.create_timer(self.scan_time_sec, self._finish_scan)

    def _finish_scan(self) -> None:
        """Timer callback: actually analyze most recent camera frame."""
        if self.scan_timer:
            self.scan_timer.cancel()
            self.scan_timer = None

        frame = self.rgb.copy() if self.rgb is not None else None
        gx = self.current.x if self.current else 0.0
        gy = self.current.y if self.current else 0.0

        self.get_logger().info(
            f"🔍 Finishing scan for plant at ({gx:.2f}, {gy:.2f}). "
            f"Frame {'OK' if frame is not None else 'MISSING'}."
        )

        self.analyze_scene_and_save(frame, gx, gy)

        self.state = "MOVE"
        self.current = None
        self.scan_point = None
        self._move_to_next_target()

    def analyze_scene_and_save(self, frame: Optional[np.ndarray], gx: float, gy: float) -> None:
        """Analyze camera frame to decide red vs green and record coordinates."""
        if frame is None:
            label, conf, path = "green", 0.0, ""
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green = cv2.inRange(hsv, np.array([35, 60, 60]), np.array([85, 255, 255]))
            red = cv2.inRange(hsv, np.array([0, 90, 60]), np.array([10, 255, 255])) | \
                  cv2.inRange(hsv, np.array([170, 90, 60]), np.array([180, 255, 255]))
            g_area, r_area = int(np.count_nonzero(green)), int(np.count_nonzero(red))
            total = g_area + r_area or 1
            label = "red" if r_area > g_area else "green"
            conf = max(r_area, g_area) / float(total)
            idx = len(self.results) + 1
            path = os.path.join(self.scans_dir, f"{idx:02d}_{label}.png")
            cv2.imwrite(path, frame)
            self.rgb = frame  # thumbnail for UI

        entry = {
            "id": len(self.results) + 1,
            "x": round(float(gx), 3),
            "y": round(float(gy), 3),
            "color": label,
            "confidence": round(float(conf), 3),
            "condition": "toxic" if label == "red" else "healthy",
            "image_path": path,
            "notes": f"{label} dominance {round(conf*100)}%"
        }
        self.results.append(entry)
        self.get_logger().info(
            f"🌱 Scan result #{entry['id']}: {label.upper()} @ "
            f"({entry['x']}, {entry['y']}) conf={entry['confidence']:.2f}, img='{path}'."
        )
        self.publish_checklist()

    def publish_checklist(self) -> None:
        msg = {"visited": len(self.results), "items": self.results}
        self.pub_list.publish(String(data=json.dumps(msg)))
        self.render_ui()

    def render_ui(self) -> None:
        canvas = np.full((360, 480, 3), (25, 25, 25), np.uint8)
        cv2.putText(canvas, f"STATE: {self.state}", (16, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(canvas, f"Visited: {len(self.results)}", (16, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1)
        if self.rgb is not None:
            thumb = cv2.resize(self.rgb, (160, 120))
            canvas[200:320, 300:460] = thumb
        try:
            self.pub_ui.publish(self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = AutoTargetScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()