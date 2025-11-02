#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Image
from nav2_msgs.action import NavigateToPose
import math, random, time
from tf_transformations import quaternion_from_euler
import numpy as np
from cv_bridge import CvBridge

class WanderNode(Node):
    def __init__(self):
        super().__init__('ve_wander_node')

        # parameters (behaviour + physics)
        self.declare_parameter('cmd_vel_topic', '/X3/cmd_vel')
        self.declare_parameter('odom_topic', '/X3/odometry')
        self.declare_parameter('scan_topic', '/X3/scan')
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('depth_topic', '/X3/depth/image_raw')
        self.declare_parameter('nav_goal_action', '/navigate_to_pose')
        self.declare_parameter('world_bounds', [-10.0, 10.0, -10.0, 10.0])
        self.declare_parameter('goal_interval_s', 6.0)
        self.declare_parameter('safety_distance', 1.0)
        self.declare_parameter('altitude_setpoint', 2.0)
        self.declare_parameter('alt_pid', [1.8, 0.01, 0.25])
        self.declare_parameter('max_lin_xy', 2.0)
        self.declare_parameter('max_ang_z', 1.6)

        # physics/feedforward params
        self.declare_parameter('mass', 1.5)                   # kg (estimate)
        self.declare_parameter('gravity', 9.81)               # m/s^2
        self.declare_parameter('alt_feedforward_scale', 0.15) # map N -> cmd units (tune)
        self.declare_parameter('alt_cmd_min', -5.0)
        self.declare_parameter('alt_cmd_max', 12.0)
        self.declare_parameter('avoid_climb_extra', 1.0)      # extra climb when avoiding

        # read parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.use_lidar = self.get_parameter('use_lidar').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.nav_goal_action = self.get_parameter('nav_goal_action').value
        self.world_bounds = self.get_parameter('world_bounds').value
        self.goal_interval = self.get_parameter('goal_interval_s').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.alt_setpoint = self.get_parameter('altitude_setpoint').value
        self.alt_pid_gains = self.get_parameter('alt_pid').value
        self.max_lin_xy = self.get_parameter('max_lin_xy').value
        self.max_ang_z = self.get_parameter('max_ang_z').value

        self.mass = self.get_parameter('mass').value
        self.gravity = self.get_parameter('gravity').value
        self.ff_scale = self.get_parameter('alt_feedforward_scale').value
        self.alt_cmd_min = self.get_parameter('alt_cmd_min').value
        self.alt_cmd_max = self.get_parameter('alt_cmd_max').value
        self.avoid_climb_extra = self.get_parameter('avoid_climb_extra').value

        # state
        self.current_odom = None
        self.min_scan = float('inf')
        self.imu = None
        self.bridge = CvBridge()
        self.last_goal_time = 0.0
        self.nav_busy = False

        # PID variables
        self.alt_integral = 0.0
        self.alt_prev_err = 0.0
        self.last_time = self.get_clock().now()

        # pubs / subs
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/X3/imu', self.imu_cb, 10)
        if self.use_lidar:
            self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        else:
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)

        # nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_goal_action)

        # main timer
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('ve_wander_node started')

    # Callbacks
    def odom_cb(self, msg: Odometry):
        self.current_odom = msg

    def imu_cb(self, msg: Imu):
        self.imu = msg

    def scan_cb(self, msg: LaserScan):
        if len(msg.ranges) > 0:
            valid = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
            self.min_scan = min(valid) if valid else float('inf')

    def depth_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            h, w = cv_img.shape
            win = cv_img[h//3:2*h//3, w//3:2*w//3]
            min_d = float(np.nanmin(win))
            self.min_scan = min_d if not math.isnan(min_d) else float('inf')
        except Exception as e:
            self.get_logger().warn(f'depth processing failed: {e}')
            self.min_scan = float('inf')

    # Altitude control with feedforward
    def altitude_control(self, dt):
        # feedforward command based on mass * g mapped to cmd units
        thrust_ff_cmd = self.mass * self.gravity * self.ff_scale

        if self.current_odom is None:
            # publish feedforward until odom arrives
            return max(min(thrust_ff_cmd, self.alt_cmd_max), self.alt_cmd_min)

        z = self.current_odom.pose.pose.position.z
        err = self.alt_setpoint - z
        P, I, D = self.alt_pid_gains

        # anti-windup guard
        if abs(err) > 10.0:
            self.alt_integral = 0.0

        self.alt_integral += err * dt
        deriv = (err - self.alt_prev_err) / dt if dt > 0 else 0.0
        pid_out = P * err + I * self.alt_integral + D * deriv
        self.alt_prev_err = err

        cmd_vertical = thrust_ff_cmd + pid_out
        cmd_vertical = max(min(cmd_vertical, self.alt_cmd_max), self.alt_cmd_min)
        return cmd_vertical

    def random_goal_pose(self):
        xmin, xmax, ymin, ymax = self.world_bounds
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        yaw = random.uniform(-math.pi, math.pi)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = self.alt_setpoint
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def send_nav_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            return None
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.nav_busy = True
        future = self.nav_client.send_goal_async(goal_msg)
        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Nav2 goal rejected')
                self.nav_busy = False
                return
            self.get_logger().info('Nav2 goal accepted')
            result_future = goal_handle.get_result_async()
            def result_cb(rf):
                self.get_logger().info('Nav2 result received')
                self.nav_busy = False
            result_future.add_done_callback(result_cb)
        future.add_done_callback(goal_response_callback)
        return future

    def cancel_nav_goal(self):
        try:
            self.nav_busy = False
            self.get_logger().info('Cancelling Nav2 goal (soft)')
        except Exception as e:
            self.get_logger().warn(f'cancel failed: {e}')

    def local_avoidance(self):
        t = Twist()
        t.linear.x = 0.0
        t.linear.z = self.altitude_control(0.1) + self.avoid_climb_extra
        t.angular.z = self.max_ang_z * 0.8
        self.pub_cmd.publish(t)
        rclpy.spin_once(self, timeout_sec=0.5)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9 if self.last_time else 0.1
        self.last_time = now

        # small safety checks using IMU
        if self.imu:
            ang_x = abs(self.imu.angular_velocity.x)
            ang_y = abs(self.imu.angular_velocity.y)
            # reset integral if tilting severely
            if ang_x > 1.0 or ang_y > 1.0:
                self.alt_integral = 0.0

        # compute vertical command
        thrust_cmd = self.altitude_control(dt)

        # obstacle: climb/back off
        if self.min_scan < self.safety_distance:
            self.get_logger().info(f'obstacle detected at {self.min_scan:.2f} m, doing avoidance')
            t = Twist()
            t.linear.x = -0.2
            t.linear.z = min(thrust_cmd + self.avoid_climb_extra, self.alt_cmd_max)
            t.angular.z = self.max_ang_z * 0.6
            self.pub_cmd.publish(t)
            if self.nav_busy:
                self.cancel_nav_goal()
            return

        # periodic nav2 goal
        if (not self.nav_busy) and (time.time() - self.last_goal_time > self.goal_interval):
            pose = self.random_goal_pose()
            self.get_logger().info(f'sending new nav goal x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}')
            self.send_nav_goal(pose)
            self.last_goal_time = time.time()
            return

        # maintain altitude while nav busy
        if self.nav_busy:
            t = Twist()
            t.linear.x = 0.0
            t.linear.z = thrust_cmd
            t.angular.z = 0.0
            self.pub_cmd.publish(t)
            return

        # fallback wander
        t = Twist()
        t.linear.x = 0.6 * self.max_lin_xy
        t.linear.z = thrust_cmd
        t.angular.z = random.uniform(-0.3, 0.3) * self.max_ang_z
        self.pub_cmd.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = WanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ve_wander_node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
