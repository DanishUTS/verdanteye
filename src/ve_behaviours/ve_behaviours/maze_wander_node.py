#!/usr/bin/env python3
# ve_behaviours/ve_behaviours/maze_wander_node.py
#
# X3 drone maze-wander with constant-altitude hold and holonomic avoidance.
# Prefers /X3/pose (geometry_msgs/Pose) for true Z from Gazebo; falls back to
# /odometry|/odom only if pose is unavailable. If no valid Z -> linear.z=0 (safety).
#
# Subscribes:
#   /X3/pose (geometry_msgs/Pose)            -> altitude (preferred)
#   /odometry and /odom (nav_msgs/Odometry)  -> x,y,yaw (+ fallback z, vz)
#   /scan (sensor_msgs/LaserScan)
#   /imu  (sensor_msgs/Imu)
# Publishes:
#   /cmd_vel (geometry_msgs/Twist)

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def ang_norm(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class MazeWanderNode(Node):
    def __init__(self) -> None:
        super().__init__("maze_wander_node")

        # --- Topics ---
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odometry_topic_primary", "/odometry")
        self.declare_parameter("odometry_topic_fallback", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("pose_topic", "/X3/pose")  # NEW

        # --- Altitude control params ---
        self.declare_parameter("target_altitude", 1.2)
        self.declare_parameter("altitude_tolerance", 0.1)
        self.declare_parameter("kp_z", 1.0)
        self.declare_parameter("ki_z", 0.3)
        self.declare_parameter("kd_zdot", 0.05)
        self.declare_parameter("max_vz", 1.0)
        self.declare_parameter("max_int", 0.5)
        self.declare_parameter("min_altitude", 1.0)
        self.declare_parameter("max_altitude", 2.2)

        # --- XY motion / avoidance ---
        self.declare_parameter("linear_speed_fwd", 1.6)
        self.declare_parameter("linear_speed_strafe", 0.6)
        self.declare_parameter("angular_speed", 1.4)
        self.declare_parameter("goal_tolerance", 0.6)
        self.declare_parameter("obs_dist_thresh", 1.0)
        self.declare_parameter("obs_clear_thresh", 1.0)

        # Waypoints (x,y)
        self.declare_parameter(
            "waypoints",
            [ -11.0, 11.0, 10.0, 11.0,
               10.0,  5.0, -11.0,  5.0,
              -11.0,  4.0, 10.0,   4.0,
               10.0,  0.0, -11.0,  0.0,
              -11.0, -3.5, 11.0,  -3.5,
               10.0, -8.0, -11.0, -8.0
            ]
        )

        # --- Load params ---
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.odom_topic_primary = self.get_parameter("odometry_topic_primary").get_parameter_value().string_value
        self.odom_topic_fallback = self.get_parameter("odometry_topic_fallback").get_parameter_value().string_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value  # NEW

        self.target_altitude = float(self.get_parameter("target_altitude").value)
        self.alt_tol = float(self.get_parameter("altitude_tolerance").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.ki_z = float(self.get_parameter("ki_z").value)
        self.kd_zdot = float(self.get_parameter("kd_zdot").value)
        self.max_vz = float(self.get_parameter("max_vz").value)
        self.max_int = float(self.get_parameter("max_int").value)
        self.min_alt = float(self.get_parameter("min_altitude").value)
        self.max_alt = float(self.get_parameter("max_altitude").value)

        self.lin_fwd = float(self.get_parameter("linear_speed_fwd").value)
        self.lin_strafe = float(self.get_parameter("linear_speed_strafe").value)
        self.ang_speed = float(self.get_parameter("angular_speed").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.obs_thresh = float(self.get_parameter("obs_dist_thresh").value)
        self.obs_clear = float(self.get_parameter("obs_clear_thresh").value)

        wps_flat: List[float] = list(self.get_parameter("waypoints").value)
        self.waypoints: List[Tuple[float, float]] = [(wps_flat[i], wps_flat[i + 1]) for i in range(0, len(wps_flat), 2)]
        self.wp_idx = 0

        # --- State ---
        self.have_odom = False
        self.have_scan = False
        self.have_pose = False
        self.locked_odom_topic: Optional[str] = None
        self.last_wait_log = 0.0

        # Add these to __init__() under state variables
        self.stuck_timer = 0.0
        self.recovering = False
        self.recovery_start = 0.0


        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.roll = self.pitch = self.yaw = 0.0

        self.min_front = float("inf")
        self.min_left = float("inf")
        self.min_right = float("inf")

        self.int_err = 0.0
        self.prev_log_t = 0.0

        # --- ROS I/O ---
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub_pose = self.create_subscription(Pose, self.pose_topic, self.pose_callback, 10)  # NEW
        self.sub_odom_primary = self.create_subscription(Odometry, self.odom_topic_primary, self._odom_primary_cb, 30)
        self.sub_odom_fallback = self.create_subscription(Odometry, self.odom_topic_fallback, self._odom_fallback_cb, 30)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info(
            f"[maze_wander_node] Hover Z={self.target_altitude:.2f}±{self.alt_tol:.2f} m; Kp={self.kp_z:.2f}, Ki={self.ki_z:.2f}, Kd_zdot={self.kd_zdot:.2f}, max_vz={self.max_vz:.2f}"
        )
        self.get_logger().info(
            f"[maze_wander_node] Topics: cmd_vel='{self.cmd_vel_topic}', pose='{self.pose_topic}', "
            f"odom='{self.odom_topic_primary}'|'{self.odom_topic_fallback}', scan='{self.scan_topic}', imu='{self.imu_topic}'"
        )

    # ----------------- Pose & Odom handling -----------------
    def pose_callback(self, msg: Pose) -> None:
        self.z = float(msg.position.z)   # true 3D altitude from Gazebo
        self.have_pose = True

    def _process_odom(self, msg: Odometry, source_topic: str) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        if not self.have_pose:
            self.z = p.z  # fallback only
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        _, _, self.yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        if not self.have_odom:
            self.have_odom = True
            self.locked_odom_topic = source_topic
            self.get_logger().info(f"[maze_wander_node] Locked onto odometry: {source_topic}")

    def _odom_primary_cb(self, msg: Odometry) -> None:
        if self.locked_odom_topic in (None, self.odom_topic_primary):
            self._process_odom(msg, self.odom_topic_primary)

    def _odom_fallback_cb(self, msg: Odometry) -> None:
        if self.locked_odom_topic is None:
            self._process_odom(msg, self.odom_topic_fallback)

    # ----------------- Other callbacks -----------------
    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        self.roll, self.pitch, _ = quat_to_euler(q.x, q.y, q.z, q.w)

    def _sector_indices(self, msg: LaserScan, deg_lo: float, deg_hi: float) -> Tuple[int, int]:
        a_lo = max(math.radians(deg_lo), msg.angle_min)
        a_hi = min(math.radians(deg_hi), msg.angle_max)
        i_lo = int((a_lo - msg.angle_min) / msg.angle_increment)
        i_hi = int((a_hi - msg.angle_min) / msg.angle_increment)
        i_lo = max(0, min(i_lo, len(msg.ranges) - 1))
        i_hi = max(0, min(i_hi, len(msg.ranges) - 1))
        if i_lo > i_hi:
            i_lo, i_hi = i_hi, i_lo
        return i_lo, i_hi

    def _min_in_sector(self, msg: LaserScan, deg_lo: float, deg_hi: float) -> float:
        i_lo, i_hi = self._sector_indices(msg, deg_lo, deg_hi)
        vals = [r for r in msg.ranges[i_lo:i_hi+1] if r and r > 0.0 and math.isfinite(r)]
        return min(vals) if vals else msg.range_max

    def scan_callback(self, msg: LaserScan) -> None:
        self.min_right = self._min_in_sector(msg, -120.0, -20.0)
        self.min_front = self._min_in_sector(msg, -35.0, +35.0)
        self.min_left  = self._min_in_sector(msg, +20.0, +120.0)
        if not self.have_scan:
            self.have_scan = True
            self.get_logger().info(f"[maze_wander_node] Scan locked: {self.scan_topic}")

    # ----------------- Altitude control -----------------
    def altitude_hold(self, dt: float) -> float:
        # If we have no valid altitude feedback, DO NOT CLIMB.
        if not self.have_pose and abs(self.z) < 1e-3:
            return 0.0

        if self.z >= self.max_alt:
            self.int_err = 0.0
            return -self.max_vz
        if self.z <= self.min_alt:
            self.int_err = 0.0
            return +self.max_vz

        e = self.target_altitude - self.z
        self.int_err += e * dt
        self.int_err = clamp(self.int_err, -self.max_int, +self.max_int)
        vz_cmd = self.kp_z * e + self.ki_z * self.int_err - self.kd_zdot * self.vz

        if abs(e) <= self.alt_tol:
            vz_cmd = 0.0

        return clamp(vz_cmd, -self.max_vz, +self.max_vz)

    # ----------------- Avoidance -----------------
    def avoidance_cmd(self) -> Optional[Tuple[float, float, float]]:
        if not self.have_scan:
            return None
        if self.min_front < self.obs_thresh:
            if self.min_left > self.min_right:
                return (0.2 * self.lin_fwd, +self.lin_strafe, +0.5 * self.ang_speed)
            else:
                return (0.2 * self.lin_fwd, -self.lin_strafe, -0.5 * self.ang_speed)
        if self.min_front < self.obs_clear:
            return (0.2 * self.lin_fwd, 0.0, 0.0)
        return None

    # ----------------- Waypoints -----------------
    def current_waypoint(self) -> Tuple[float, float]:
        return self.waypoints[self.wp_idx]

    def maybe_advance_waypoint(self) -> None:
        gx, gy = self.current_waypoint()
        if math.hypot(gx - self.x, gy - self.y) <= self.goal_tol:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self.get_logger().info(f"[maze_wander_node] Reached waypoint, next -> index {self.wp_idx}")

    # ----------------- Main loop -----------------
    def control_loop(self) -> None:
        now = time.time()
        if not self.have_odom:
            if now - self.last_wait_log > 2.0:
                self.get_logger().warn(
                    f"Waiting for odometry on '{self.odom_topic_primary}' or '{self.odom_topic_fallback}'..."
                )
                self.last_wait_log = now
            return

        dt = 0.1  # timer period
        vz = self.altitude_hold(dt)

        self.maybe_advance_waypoint()
        gx, gy = self.current_waypoint()
        desired_yaw = math.atan2(gy - self.y, gx - self.x)
        yaw_err = ang_norm(desired_yaw - self.yaw)

        avoid = self.avoidance_cmd()
        now = time.time()

        # --- Enhanced stuck detection ---
        if abs(self.vx) < 0.05 and (self.min_front > 1.0 or not self.have_scan):
            self.stuck_timer += dt
        else:
            self.stuck_timer = 0.0

        if self.stuck_timer > 1.5 and not self.recovering:
            self.recovering = True
            self.recovery_start = now
            self.get_logger().warn("[maze_wander_node] Stuck detected — initiating recovery")

        cmd = Twist()

        # --- Recovery mode ---
        if self.recovering:
            if now - self.recovery_start < 1.5:
                cmd.linear.x = -0.3
                cmd.linear.y = 0.0
                cmd.angular.z = +0.8 if self.min_left > self.min_right else -0.8
                cmd.linear.z = vz
                self.pub_cmd.publish(cmd)
                return
            else:
                self.recovering = False

        # --- Normal motion ---
        if avoid:
            vx, vy, yaw_rate = avoid
            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = yaw_rate
        else:
            cmd.angular.z = clamp(self.ang_speed * yaw_err, -self.ang_speed, +self.ang_speed)
            cmd.linear.x = self.lin_fwd if abs(yaw_err) < 0.4 else 0.0
            cmd.linear.y = 0.0

        cmd.linear.z = vz
        self.pub_cmd.publish(cmd)

        if now - self.prev_log_t > 2.5:
            self.get_logger().info(
                f"z={self.z:.2f} vz={self.vz:.2f} cmdVz={cmd.linear.z:.2f} e={self.target_altitude - self.z:+.2f} "
                f"front={self.min_front:.2f} L={self.min_left:.2f} R={self.min_right:.2f} wp={self.wp_idx} pose={'Y' if self.have_pose else 'N'}"
            )
            self.prev_log_t = now


    def destroy_node(self) -> bool:
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
        rclpy.init(args=args)
        node = MazeWanderNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()