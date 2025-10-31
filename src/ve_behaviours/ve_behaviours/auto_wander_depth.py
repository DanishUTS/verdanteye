#!/usr/bin/env python3
import math
import time
import random
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import QoSOverridingOptions, QoSPolicyKind

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import SetEntityPose  # ros_gz

def sensor_qos():
    # Equivalent to SensorDataQoS but explicit (so it’s easy to tweak)
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

class AutoWanderDepth(Node):
    def __init__(self):
        super().__init__('auto_wander_depth')

        # ------------ Parameters ------------
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('depth_topic', '/camera/depth/image')
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('min_range', 0.50)
        self.declare_parameter('wall_buffer', 0.70)
        self.declare_parameter('world_name', 'large_demo')   # for teleport
        self.declare_parameter('entity_name', 'husky')       # gazebo entity

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.wall_buffer = float(self.get_parameter('wall_buffer').value)
        self.world_name = str(self.get_parameter('world_name').value)
        self.entity_name = str(self.get_parameter('entity_name').value)

        # ------------ Pub/Sub ------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "robot_pose", 10)
        self.marker_pub = self.create_publisher(Marker, "obstacle_markers", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Depth with sensor QoS (prevents message filter spam)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, sensor_qos()
        )

        # UI
        self.run_enabled = False
        self.ui_run_sub = self.create_subscription(Bool, '/ui/run_enabled', self._ui_run_cb, 10)
        self.restart_srv = self.create_service(Trigger, '/ui/restart', self._handle_restart)

        # ------------ State ------------
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp = 0.0

        # Dead-reckon pose (for UI)
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.last_time = time.time()

        # Spawn pose (latched)
        self.spawn_x = 0.0; self.spawn_y = 0.0; self.spawn_theta = 0.0
        self.spawn_set = False

        # Steering smoothing
        self._last_ang = 0.0
        self._ang_lowpass = 0.35  # 0..1 (higher = snappier)

        # Teleport client (persistent)
        self._teleport_srv_name = f'/world/{self.world_name}/set_entity_pose'
        self._teleport_cli = self.create_client(SetEntityPose, self._teleport_srv_name)

        # Throttle markers
        self._marker_every = 5
        self._tick = 0

        # Wait for first depth before starting the control timer
        self._control_period = 0.1
        self._wait_timer = self.create_timer(0.2, self._wait_for_depth)
        self._control_timer = None

        self.get_logger().info(f"🚀 AutoWanderDepth ready (depth: {self.depth_topic}, cmd: {self.cmd_vel_topic})")

    # ---------------- UI ----------------
    def _ui_run_cb(self, msg: Bool):
        self.run_enabled = bool(msg.data)
        if not self.run_enabled:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🛑 Robot stopped by UI.")
        else:
            self.get_logger().info("▶️ Robot started by UI.")

    def _handle_restart(self, request, response):
        self.get_logger().info("🔁 Restart requested — stopping, resetting pose, teleporting…")
        self.run_enabled = False
        self.cmd_pub.publish(Twist())

        # Re-seed pose for SLAM
        init = PoseWithCovarianceStamped()
        init.header.stamp = self.get_clock().now().to_msg()
        init.header.frame_id = "map"
        init.pose.pose.position.x = self.spawn_x
        init.pose.pose.position.y = self.spawn_y
        init.pose.pose.orientation.z = math.sin(self.spawn_theta / 2.0)
        init.pose.pose.orientation.w = math.cos(self.spawn_theta / 2.0)
        init.pose.covariance[0] = 0.04
        init.pose.covariance[7] = 0.04
        init.pose.covariance[35] = 0.02
        self.initialpose_pub.publish(init)

        # Optional: clear SLAM map if service exists
        try:
            from std_srvs.srv import Empty
            clear_cli = self.create_client(Empty, '/slam_toolbox/clear')
            if clear_cli.wait_for_service(timeout_sec=1.0):
                clear_cli.call_async(Empty.Request())
        except Exception:
            pass

        # Teleport in Gazebo
        if not self._teleport_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"❌ Teleport service unavailable: {self._teleport_srv_name}")
        else:
            req = SetEntityPose.Request()
            req.pose.name = self.entity_name
            req.pose.position.x = float(self.spawn_x)
            req.pose.position.y = float(self.spawn_y)
            req.pose.position.z = 0.25
            req.pose.orientation.w = math.cos(self.spawn_theta / 2.0)
            req.pose.orientation.z = math.sin(self.spawn_theta / 2.0)

            fut = self._teleport_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            ok = (fut.result() is not None and getattr(fut.result(), "success", False))
            self.get_logger().info("✅ Teleport OK." if ok else "⚠️ Teleport failed/timed out.")

        response.success = True
        response.message = "Restart complete."
        return response

    # -------------- Depth --------------
    def depth_callback(self, msg: Image):
        try:
            # Expecting 32FC1 or 16UC1; handle both
            if msg.encoding in ("32FC1", "32FC"):
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth = np.asarray(depth, dtype=np.float32)
            elif msg.encoding in ("16UC1", "mono16"):
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
                depth = raw * 0.001  # mm->m
            else:
                # Fallback (best effort)
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)

            # Sanitize and clamp
            depth = np.nan_to_num(depth, nan=10.0, posinf=10.0, neginf=10.0)
            depth = np.clip(depth, 0.0, 20.0)

            self.latest_depth = depth
            self.latest_depth_stamp = self.get_clock().now().nanoseconds * 1e-9
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    # -------- Helpers / publishing ------
    def publish_pose(self, twist: Twist):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.x += twist.linear.x * math.cos(self.theta) * dt
        self.y += twist.linear.x * math.sin(self.theta) * dt
        self.theta += twist.angular.z * dt

        if not self.spawn_set and (abs(self.x) + abs(self.y) > 1e-5):
            self.spawn_x, self.spawn_y, self.spawn_theta = self.x, self.y, self.theta
            self.spawn_set = True
            self.get_logger().info(f"📍 Spawn recorded at x={self.spawn_x:.2f}, y={self.spawn_y:.2f}, θ={self.spawn_theta:.2f}")

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2.0)
        self.pose_pub.publish(pose_msg)

    def publish_obstacle_marker(self, distance, angle, idx):
        # Throttle to reduce RViz load
        if self._tick % self._marker_every != 0:
            return
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.ns = "obstacles"
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = distance * math.cos(angle)
        m.pose.position.y = distance * math.sin(angle)
        m.pose.position.z = 0.2
        m.scale.x = m.scale.y = m.scale.z = 0.20
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
        self.marker_pub.publish(m)

    # -------------- Control --------------
    def _wait_for_depth(self):
        # Don’t start control until we’ve seen one fresh frame
        if self.latest_depth is None:
            return
        # Also ensure its timestamp is “recent” in sim time
        if (self.get_clock().now().nanoseconds * 1e-9) - self.latest_depth_stamp > 1.0:
            return

        # Start control loop now that stream is alive
        self.destroy_timer(self._wait_timer)
        self._control_timer = self.create_timer(self._control_period, self.control_loop)
        self.get_logger().info("🎥 Depth stream detected — control loop started.")

    def control_loop(self):
        self._tick += 1

        # Always publish zero if paused
        if not self.run_enabled:
            self.cmd_pub.publish(Twist())
            return

        # Require valid & recent depth
        if self.latest_depth is None:
            return
        if (self.get_clock().now().nanoseconds * 1e-9) - self.latest_depth_stamp > 1.0:
            # stale frame (Gazebo hiccup) — don’t move this tick
            self.cmd_pub.publish(Twist())
            return

        depth = self.latest_depth
        h, w = depth.shape
        h1, h2 = h // 3, 2 * h // 3
        w1, w2 = w // 3, 2 * w // 3

        left   = depth[h1:h2, :w1]
        center = depth[h1:h2, w1:w2]
        right  = depth[h1:h2, w2:]

        min_left   = float(np.min(left))
        min_center = float(np.min(center))
        min_right  = float(np.min(right))

        # Basic reactive logic with smoothing and wall bias
        cmd = Twist()

        if min_center < self.min_range and min_left < self.min_range and min_right < self.min_range:
            # Tight box ahead — reverse and turn
            cmd.linear.x = -0.20
            target_ang = self.angular_speed
        elif min_center < self.min_range:
            # Obstacle ahead — turn toward the freer side
            target_ang = self.angular_speed if min_left > min_right else -self.angular_speed
        else:
            # Free — go forward, nudge away from close wall
            cmd.linear.x = self.linear_speed
            if min_left < self.wall_buffer and min_right >= self.wall_buffer:
                target_ang = -0.30
            elif min_right < self.wall_buffer and min_left >= self.wall_buffer:
                target_ang = 0.30
            else:
                target_ang = 0.0

        # Low-pass filter on angular velocity to prevent oscillations
        alpha = self._ang_lowpass
        cmd.angular.z = alpha * target_ang + (1.0 - alpha) * self._last_ang
        self._last_ang = cmd.angular.z

        self.cmd_pub.publish(cmd)
        self.publish_pose(cmd)
        self.publish_obstacle_marker(min_center, 0.0, 0)
        self.publish_obstacle_marker(min_left,  math.pi/4, 1)
        self.publish_obstacle_marker(min_right, -math.pi/4, 2)

def main():
    rclpy.init()
    node = AutoWanderDepth()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())  # brake on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
