#!/usr/bin/env python3
# ===============================================================
# VerdantEye: auto_wander_depth (RGB-only, Nav2-integrated)
# - Subscribes & publishes like WanderNode (drone), but for Husky
# - Nav2 NavigateToPose for motion, cancel during SCAN
# - Targets: /bamboo/targets (latched)
# - UI: /plant_scan/ui (Image), /plant_scan/checklist (String JSON)
# ===============================================================

from __future__ import annotations
import os, math, json
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

import numpy as np
import cv2
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from nav2_msgs.action import NavigateToPose


@dataclass
class Target:
    x: float
    y: float
    z: float = 0.0


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class AutoWanderDepth(Node):
    def __init__(self) -> None:
        super().__init__("auto_wander_depth")

        # ---------- Parameters (mirror WanderNode style) ----------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("targets_topic", "/bamboo/targets")
        self.declare_parameter("run_enabled_topic", "/ui/run_enabled")
        self.declare_parameter("restart_srv", "/ui/restart")
        self.declare_parameter("nav_goal_action", "navigate_to_pose")

        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("standoff_m", 0.8)
        self.declare_parameter("close_distance_m", 0.4)
        self.declare_parameter("scan_time_sec", 3.0)

        
        self.declare_parameter("publish_stop_twist", True)

        # ---------- Read params ----------
        self.cmd_vel_topic: str = self.get_parameter("cmd_vel_topic").value
        self.odom_topic: str = self.get_parameter("odom_topic").value
        self.camera_topic: str = self.get_parameter("camera_topic").value
        self.targets_topic: str = self.get_parameter("targets_topic").value
        self.run_enabled_topic: str = self.get_parameter("run_enabled_topic").value
        self.restart_srv: str = self.get_parameter("restart_srv").value
        self.nav_goal_action: str = self.get_parameter("nav_goal_action").value

        self.goal_frame: str = self.get_parameter("goal_frame").value
        self.standoff_m: float = float(self.get_parameter("standoff_m").value)
        self.close_distance_m: float = float(self.get_parameter("close_distance_m").value)
        self.scan_time_sec: float = float(self.get_parameter("scan_time_sec").value)
        self.publish_stop_twist: bool = bool(self.get_parameter("publish_stop_twist").value)

        os.makedirs(os.path.expanduser("~/.ros/plant_scans"), exist_ok=True)
        self.scans_dir = os.path.expanduser("~/.ros/plant_scans")

        # ---------- State ----------
        self.bridge = CvBridge()
        self.rgb: Optional[np.ndarray] = None
        self.robot_xy: Tuple[float, float] = (0.0, 0.0)

        self.targets_raw: List[Target] = []
        self.remaining_targets: List[Target] = []
        self.current: Optional[Target] = None
        self.scan_point: Optional[Tuple[float, float]] = None
        self.results: List[dict] = []
        self.state: str = "IDLE"
        self.run_enabled: bool = False

        
        self.timeout_timer = None
        self.scan_timer = None

        # ---------- Pub/Sub like WanderNode ----------
        qos_targets = QoSProfile(depth=1)
        qos_targets.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_targets.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(PoseArray, self.targets_topic, self.on_targets, qos_targets)
        self.create_subscription(Bool, self.run_enabled_topic, self.on_run_enabled, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(Image, self.camera_topic, self.on_rgb, 10)

        self.pub_ui = self.create_publisher(Image, "/plant_scan/ui", 1)
        self.pub_list = self.create_publisher(String, "/plant_scan/checklist", 1)

        
        from geometry_msgs.msg import Twist
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._Twist = Twist  

        self.srv_restart = self.create_service(Trigger, self.restart_srv, self.on_restart)

        
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_goal_action)
        self.goal_handle = None

        self.get_logger().info(" auto_wander_depth RGB, Nav2 integrated ready.")

    # ---------------- Callbacks ----------------
    def on_run_enabled(self, msg: Bool) -> None:
        self.run_enabled = bool(msg.data)
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
            self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"RGB conversion failed: {e}")
            self.rgb = None 

    def on_odom(self, msg: Odometry) -> None:
        self.robot_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def on_restart(self, _req: Trigger.Request, _ctx=None) -> Trigger.Response:
        self.run_enabled = False
        self.state = "IDLE"
        self.current = None
        self.scan_point = None
        self.results.clear()
        self.remaining_targets.clear()

        if self.timeout_timer:
            self.timeout_timer.cancel(); self.timeout_timer = None
        if self.scan_timer:
            self.scan_timer.cancel(); self.scan_timer = None
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

        self.get_logger().info("🔄 Restarted scanner (targets preserved).")
        return Trigger.Response(success=True, message="Restarted.")

    # ---------------- Nav2 drive -> scan ----------------
    def _move_to_next_target(self) -> None:
        if not self.remaining_targets:
            self.state = "DONE"
            self.publish_checklist()
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

        self.get_logger().info(
            f"🎯 Target at ({gx:.2f},{gy:.2f}); scan point=({sx:.2f},{sy:.2f}); robot=({rx:.2f},{ry:.2f}); d={dist:.2f}m"
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

        yaw = math.atan2(face_y - gy, face_x - gx) if (face_x is not None and face_y is not None) else 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

        if self.timeout_timer:
            self.timeout_timer.cancel()
        self.timeout_timer = self.create_timer(12.0, self._check_scan_timeout)

    def _on_goal_sent(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
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
            self.timeout_timer.cancel(); self.timeout_timer = None
        self.goal_handle = None

        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            self._enter_scan_mode()
        else:
            self.get_logger().warn(f"Goal failed (status={getattr(result,'status','?')}); retrying scan point.")
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
            if dist <= self.close_distance_m * 1.5:
                self.get_logger().warn(f"⏳ Timeout near plant ({dist:.2f} m) — forcing scan.")
                self._enter_scan_mode()

    # ---------------- SCAN ----------------
    def _enter_scan_mode(self):
      
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

       
        if self.publish_stop_twist:
            t = self._Twist() 
            self.pub_cmd.publish(t)

        self.state = "SCAN"
        self._perform_scan_and_continue()

    def _perform_scan_and_continue(self):
        if not self.current:
            return
        self.get_logger().info(f"📸 Capturing RGB for {self.scan_time_sec:.1f}s ...")
        if self.scan_timer:
            self.scan_timer.cancel()
     
        self.scan_timer = self.create_timer(self.scan_time_sec, self._finish_scan)

    def _finish_scan(self):
        if self.scan_timer:
            self.scan_timer.cancel(); self.scan_timer = None

        frame = self.rgb.copy() if self.rgb is not None else None
        gx = self.current.x if self.current else 0.0
        gy = self.current.y if self.current else 0.0

        self._analyze_and_record(frame, gx, gy)

        self.current = None
        self.scan_point = None
        self.state = "MOVE"
        self._move_to_next_target()

    # ---------------- Perception + UI ----------------
    def _analyze_and_record(self, frame: Optional[np.ndarray], gx: float, gy: float) -> None:
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
            try:
                cv2.imwrite(path, frame)
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
        cv2.putText(canvas, f"STATE: {self.state}", (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(canvas, f"Visited: {len(self.results)}", (16, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 1)
        if self.rgb is not None:
            thumb = cv2.resize(self.rgb, (160, 120))
            canvas[200:320, 300:460] = thumb
        try:
            self.pub_ui.publish(self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
        except Exception:
            pass


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
