#!/usr/bin/env python3
# ===============================================================
# VerdantEye: AutoTargetScan (Nav2 version)
# - Uses Nav2 /navigate_to_pose for movement (not /cmd_vel)
# - One-shot target fetching: fetch PoseArray once, visit, re-fetch once, repeat
# - Remembers visited targets and skips near-duplicates
# - Scans & classifies at each location
# ===============================================================
from __future__ import annotations
import os, math, json, time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient

import numpy as np
import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose


@dataclass
class Target:
    x: float
    y: float
    z: float = 0.0
    name: Optional[str] = None


class AutoTargetScan(Node):
    def __init__(self) -> None:
        super().__init__("auto_wander_nav2")

        # --- Parameters ---
        self.declare_parameter("scan_time_sec", 5.0)
        self.declare_parameter("scans_dir", os.path.expanduser("~/.ros/plant_scans"))
        self.declare_parameter("world_name", "large_demo")
        self.declare_parameter("entity_name", "husky")

        self.scan_time_sec = float(self.get_parameter("scan_time_sec").value)
        self.scans_dir = str(self.get_parameter("scans_dir").value)
        os.makedirs(self.scans_dir, exist_ok=True)

        # --- ROS interfaces ---
        # NOTE: We subscribe to /bamboo/targets on-demand (one-shot) — not here.
        self.create_subscription(Bool, "/ui/run_enabled", self.on_run_enabled, 10)
        self.create_subscription(Image, "/camera/color/image_raw", self.on_rgb, 10)

        self.pub_ui = self.create_publisher(Image, "/plant_scan/ui", 1)
        self.pub_list = self.create_publisher(String, "/plant_scan/checklist", 1)

        self.srv_restart = self.create_service(Trigger, "/ui/restart", self.on_restart)
        self.bridge = CvBridge()

        # --- State ---
        self.rgb = None
        self.targets_raw: List[Target] = []
        self.plan: List[Target] = []
        self.current: Optional[Target] = None
        self.results: List[dict] = []
        self.state = "IDLE"
        self.run_enabled = False

        # One-shot target subscription handle + visited memory
        self.target_sub = None              # temporary subscriber to /bamboo/targets
        self.visited = set()                # {(x_rounded, y_rounded)}
        self.reach_radius = 0.6             # meters; tolerance for "already visited"

        # --- Nav2 Action Client ---
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(0.5, self.loop)
        self.get_logger().info("🌿 AutoTargetScan (Nav2 version) ready.")

    # -------------------------------------------------------------
    # One-shot subscription helper
    # -------------------------------------------------------------
    def subscribe_targets_once(self):
        """Subscribe to /bamboo/targets once (TRANSIENT_LOCAL). Destroy after first message."""
        if self.target_sub is not None:
            return  # already waiting for a PoseArray

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.get_logger().info("Subscribing (one-shot) for /bamboo/targets...")
        self.target_sub = self.create_subscription(
            PoseArray, "/bamboo/targets", self.on_targets, qos
        )

    # -------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------
    def on_run_enabled(self, msg: Bool) -> None:
        self.run_enabled = msg.data
        self.get_logger().info(f"/ui/run_enabled = {self.run_enabled}")
        if self.run_enabled:
            # fetch the latest targets exactly once; on_targets will build a plan
            self.subscribe_targets_once()
            self.state = "MOVE"
        else:
            self.state = "IDLE"

    def on_targets(self, msg: PoseArray) -> None:
        # Save list and immediately stop listening (one-shot)
        self.targets_raw = [Target(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self.get_logger().info(f"Received {len(self.targets_raw)} targets (one-shot).")

        if self.target_sub is not None:
            try:
                self.destroy_subscription(self.target_sub)
            except Exception:
                pass
            self.target_sub = None

        # Build a new plan excluding already-visited
        self.plan_from_current_pose()

    def on_rgb(self, msg: Image) -> None:
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            self.rgb = None

    def on_restart(self, _req: Trigger.Request, _ctx=None) -> Trigger.Response:
        self.run_enabled = False
        self.state = "IDLE"
        self.plan.clear()
        self.current = None
        self.results.clear()
        self.targets_raw.clear()
        self.visited.clear()
        self.get_logger().info("Scanner reset.")
        return Trigger.Response(success=True, message="Restarted.")

    # -------------------------------------------------------------
    # Planning
    # -------------------------------------------------------------
    def _is_visited(self, t: Target) -> bool:
        rx, ry = round(t.x, 3), round(t.y, 3)
        for vx, vy in self.visited:
            dx, dy = rx - vx, ry - vy
            if (dx * dx + dy * dy) <= (self.reach_radius * self.reach_radius):
                return True
        return False

    def plan_from_current_pose(self) -> None:
        if not self.targets_raw:
            self.plan = []
            self.state = "WAITING_FOR_TARGETS"
            return

        ordered = [t for t in self.targets_raw if not self._is_visited(t)]
        # import random; random.shuffle(ordered)  # optional randomness
        self.plan = ordered
        self.current = None

        self.get_logger().info(f"Planning: {len(self.plan)} unvisited targets queued.")
        if not self.plan and self.run_enabled:
            self.state = "DONE"

    # -------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------
    def loop(self) -> None:
        self.render_ui()
        if not self.run_enabled:
            return

        if self.state == "MOVE":
            if self.current is None and self.plan:
                self.current = self.plan.pop(0)
                self.send_nav_goal(self.current.x, self.current.y)
            elif self.current is None and not self.plan:
                # No queued plan -> fetch once and wait
                if self.target_sub is None:
                    self.subscribe_targets_once()
                self.state = "WAITING_FOR_TARGETS"

        elif self.state == "SCAN":
            # Do the scan/classification, then re-check targets exactly once
            self.analyze_scene_and_save()
            self.current = None
            self.subscribe_targets_once()  # re-fetch after reaching a target
            self.state = "MOVE"

        elif self.state == "DONE":
            self.publish_checklist()

        elif self.state == "WAITING_FOR_TARGETS":
            # Idle; on_targets will rebuild the plan and push us back to MOVE
            pass

    # -------------------------------------------------------------
    # Nav2 control
    # -------------------------------------------------------------
    def send_nav_goal(self, gx: float, gy: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 navigate_to_pose not available!")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending Nav2 goal to ({gx:.2f}, {gy:.2f})")

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            # Skip this target and continue
            self.current = None
            self.state = "MOVE" if self.plan else "DONE"
            return
        self.get_logger().info("Nav2 goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        _ = future.result().result
        self.get_logger().info("✅ Nav2 reached goal, starting scan...")
        self.state = "SCAN"

    # -------------------------------------------------------------
    # Scanning / Classification
    # -------------------------------------------------------------
    def analyze_scene_and_save(self) -> None:
        if self.rgb is None:
            label, conf, path = "green", 0.0, ""
        else:
            hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
            green = cv2.inRange(hsv, np.array([35, 60, 60]), np.array([85, 255, 255]))
            red = cv2.inRange(hsv, np.array([0, 90, 60]), np.array([10, 255, 255])) | \
                  cv2.inRange(hsv, np.array([170, 90, 60]), np.array([180, 255, 255]))
            g_area, r_area = int(np.count_nonzero(green)), int(np.count_nonzero(red))
            total = g_area + r_area or 1
            label = "red" if r_area > g_area else "green"
            conf = max(r_area, g_area) / float(total)
            idx = len(self.results) + 1
            path = os.path.join(self.scans_dir, f"{idx:02d}_{label}.png")
            cv2.imwrite(path, self.rgb)

        entry = {
            "id": len(self.results) + 1,
            "color": label,
            "confidence": round(float(conf), 3),
            "condition": "toxic" if label == "red" else "healthy",
            "image_path": path,
            "notes": f"{label} dominance {round(conf*100)}%"
        }
        self.results.append(entry)

        # Remember this just-visited target
        if self.current is not None:
            self.visited.add((round(self.current.x, 3), round(self.current.y, 3)))

        self.publish_checklist()

    def publish_checklist(self) -> None:
        msg = {"visited": len(self.results), "items": self.results}
        self.pub_list.publish(String(data=json.dumps(msg)))

    def render_ui(self) -> None:
        canvas = np.full((360, 480, 3), (25, 25, 25), dtype=np.uint8)
        cv2.putText(canvas, f"STATE: {self.state}", (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(canvas, f"Visited: {len(self.results)}", (16, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180,180,180), 1)
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
