#!/usr/bin/env python3
import math
import time
import random
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# Teleport service (ros_gz)
from ros_gz_interfaces.srv import SetEntityPose


class AutoWanderDepth(Node):
    def __init__(self):
        super().__init__('auto_wander_depth')

        # ---------- Parameters ----------
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('depth_topic', '/camera/depth/image')
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('wall_buffer', 0.7)
        # world used for teleport; pass from launch (default matches your launch)
        self.declare_parameter('world_name', 'large_demo')
        # entity name in Gazebo (what you see highlighted in the right panel)
        self.declare_parameter('entity_name', 'husky')

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.wall_buffer = float(self.get_parameter('wall_buffer').value)
        self.world_name = str(self.get_parameter('world_name').value)
        self.entity_name = str(self.get_parameter('entity_name').value)

        # ---------- ROS I/O ----------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "robot_pose", 10)
        self.marker_pub = self.create_publisher(Marker, "obstacle_markers", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        # UI integration
        self.run_enabled = False
        self.ui_run_sub = self.create_subscription(Bool, '/ui/run_enabled', self._ui_run_cb, 10)
        self.restart_srv = self.create_service(Trigger, '/ui/restart', self._handle_restart)

        # ---------- State ----------
        self.bridge = CvBridge()
        self.latest_depth = None

        # dead-reckoning estimate for the UI
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # remember the first real pose we estimate as "spawn"
        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.spawn_theta = 0.0
        self.spawn_set = False

        # ---------- Control loop ----------
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"🚀 AutoWanderDepth started (depth: {self.depth_topic})")

    # ===================== UI =====================
    def _ui_run_cb(self, msg: Bool):
        self.run_enabled = bool(msg.data)
        if not self.run_enabled:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🛑 Robot stopped by UI.")
        else:
            self.get_logger().info("▶️ Robot started by UI.")

    def _handle_restart(self, request, response):
        """Full reset: stop robot, reset pose for SLAM, and teleport Husky in Gazebo."""
        self.get_logger().info("🔁 Restart requested — stopping & teleporting...")
        self.run_enabled = False
        self.cmd_pub.publish(Twist())

        # publish /initialpose so SLAM / Nav2 re-align to the spawn
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

        # try to clear SLAM map (optional – if available)
        try:
            from std_srvs.srv import Empty
            clear_cli = self.create_client(Empty, '/slam_toolbox/clear')
            if clear_cli.wait_for_service(timeout_sec=2.0):
                clear_cli.call_async(Empty.Request())
                self.get_logger().info("🗺️ SLAM map clear requested.")
        except Exception:
            pass

        # Teleport in Gazebo via ros_gz service
        world_srv = f'/world/{self.world_name}/set_entity_pose'
        cli = self.create_client(SetEntityPose, world_srv)

        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"❌ Teleport service not available: {world_srv} "
                                    f"(check world name or Gazebo running).")
        else:
            req = SetEntityPose.Request()
            req.pose.name = self.entity_name
            req.pose.position.x = float(self.spawn_x)
            req.pose.position.y = float(self.spawn_y)
            req.pose.position.z = 0.25
            # face the same heading
            req.pose.orientation.w = math.cos(self.spawn_theta / 2.0)
            req.pose.orientation.z = math.sin(self.spawn_theta / 2.0)

            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            if fut.result() is not None and fut.result().success:
                self.get_logger().info("✅ Teleport OK — robot back at spawn.")
            else:
                self.get_logger().warn("⚠️ Teleport failed or timed out (check entity name).")

        response.success = True
        response.message = "Restart complete (stopped + pose reset + teleport)."
        return response

    # ===================== Depth =====================
    def depth_callback(self, msg: Image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = np.nan_to_num(depth_image, nan=10.0, posinf=10.0, neginf=10.0)
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    # ===================== Helpers =====================
    def publish_pose(self, twist: Twist):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.x += twist.linear.x * math.cos(self.theta) * dt
        self.y += twist.linear.x * math.sin(self.theta) * dt
        self.theta += twist.angular.z * dt

        # latch the first estimate as spawn only once
        if not self.spawn_set and (abs(self.x) + abs(self.y) > 1e-6):
            self.spawn_x, self.spawn_y, self.spawn_theta = self.x, self.y, self.theta
            self.spawn_set = True
            self.get_logger().info(
                f"📍 Spawn recorded at x={self.spawn_x:.2f}, y={self.spawn_y:.2f}, θ={self.spawn_theta:.2f}"
            )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2.0)
        self.pose_pub.publish(pose_msg)

    def publish_obstacle_marker(self, distance, angle, idx):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "obstacles"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = distance * math.cos(angle)
        marker.pose.position.y = distance * math.sin(angle)
        marker.pose.position.z = 0.2
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
        self.marker_pub.publish(marker)

    # ===================== Control =====================
    def control_loop(self):
        if not self.run_enabled:
            self.cmd_pub.publish(Twist())
            return
        if self.latest_depth is None:
            return

        h, w = self.latest_depth.shape
        left   = self.latest_depth[h//3:2*h//3, :w//3]
        center = self.latest_depth[h//3:2*h//3, w//3:2*w//3]
        right  = self.latest_depth[h//3:2*h//3, 2*w//3:]

        min_left   = float(np.min(left))
        min_center = float(np.min(center))
        min_right  = float(np.min(right))

        twist = Twist()
        if min_center < self.min_range and min_left < self.min_range and min_right < self.min_range:
            twist.linear.x = -0.2
            twist.angular.z = self.angular_speed
        elif min_center < self.min_range:
            twist.angular.z = self.angular_speed if min_left > min_right else -self.angular_speed
        else:
            twist.linear.x = self.linear_speed
            if min_left < self.wall_buffer:
                twist.angular.z = -0.3
            elif min_right < self.wall_buffer:
                twist.angular.z = 0.3

        self.cmd_pub.publish(twist)
        self.publish_pose(twist)
        self.publish_obstacle_marker(min_center, 0.0, 0)
        self.publish_obstacle_marker(min_left,  math.pi/4, 1)
        self.publish_obstacle_marker(min_right, -math.pi/4, 2)


def main():
    rclpy.init()
    node = AutoWanderDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
