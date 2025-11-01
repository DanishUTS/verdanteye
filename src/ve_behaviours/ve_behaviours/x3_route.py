#!/usr/bin/env python3
import math
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def clamp(v, lo, hi): return max(lo, min(hi, v))

class X3Route(Node):
    def __init__(self):
        super().__init__('x3_route')

        # Defaults target the X3 namespace
        self.declare_parameter('cmd_vel_topic', '/X3/cmd_vel')
        self.declare_parameter('odom_topic',    '/X3/odometry')
        self.declare_parameter('cruise_alt',    3.0)     # m
        self.declare_parameter('speed_xy',      1.3)     # m/s
        self.declare_parameter('speed_z',       0.8)     # m/s
        self.declare_parameter('yaw_rate',      0.6)     # rad/s
        # Big rectangle; tweak to fit large_demo coverage
        self.declare_parameter('waypoints_xy',  [-25,-20, 25,-20, 25,20, -25,20])
        self.declare_parameter('loiter_secs',   2.0)
        self.declare_parameter('liftoff_boost', 0.4)     # extra up-thrust to break ground contact

        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)

        wp = list(self.get_parameter('waypoints_xy').value)
        self.route: List[Tuple[float, float]] = [(wp[i], wp[i+1]) for i in range(0, len(wp), 2)]
        self.idx, self.state = 0, 'liftoff'
        self.x = self.y = self.z = self.yaw = 0.0
        self._loiter_until_ns = 0

        self.create_timer(1/30.0, self._tick)  # 30 Hz
        self.get_logger().info(f"X3 route: {len(self.route)} waypoints @ z={self.get_parameter('cruise_alt').value} m")

    def _on_odom(self, msg: Odometry):
        self.x, self.y, self.z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _tick(self):
        tw = Twist()
        alt  = float(self.get_parameter('cruise_alt').value)
        vz   = float(self.get_parameter('speed_z').value)
        vxy  = float(self.get_parameter('speed_xy').value)
        wr   = float(self.get_parameter('yaw_rate').value)

        if self.state == 'liftoff':
            tw.linear.z = vz + float(self.get_parameter('liftoff_boost').value)
            if self.z > 0.3:
                self.state = 'climb'

        elif self.state == 'climb':
            dz = alt - self.z
            tw.linear.z = clamp(dz, -vz, vz)
            if abs(dz) < 0.05:
                self.state = 'cruise'

        elif self.state in ('cruise', 'loiter'):
            gx, gy = self.route[self.idx]
            dx, dy = gx - self.x, gy - self.y
            desired_yaw = math.atan2(dy, dx)
            yaw_err = (desired_yaw - self.yaw + math.pi) % (2*math.pi) - math.pi

            tw.angular.z = clamp(1.5 * yaw_err, -wr, wr)
            tw.linear.x  = clamp(vxy * math.cos(yaw_err), -vxy, vxy)
            tw.linear.z  = clamp(alt - self.z, -vz, vz)

            if (dx*dx + dy*dy)**0.5 < 0.8:
                if self.state == 'cruise':
                    self.state = 'loiter'
                    self._loiter_until_ns = self.get_clock().now().nanoseconds + int(1e9 * float(self.get_parameter('loiter_secs').value))
                elif self.get_clock().now().nanoseconds > self._loiter_until_ns:
                    self.idx = (self.idx + 1) % len(self.route)
                    self.state = 'cruise'

        self.pub.publish(tw)

def main():
    rclpy.init()
    node = X3Route()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
