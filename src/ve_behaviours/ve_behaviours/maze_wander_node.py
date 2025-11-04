import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
import math

class MazeWanderNode(Node):
    def __init__(self):
        super().__init__('maze_wander_node')

        # Parameters
        self.target_z = 0.5
        self.z_tolerance = 0.05
        self.linear_speed = 1.4
        self.angular_speed = 1.0
        self.imu_damping = 0.5
        self.obstacle_threshold = 1.5

        # World bounds
        self.max_x, self.min_x = 12.5, -12.5
        self.max_y, self.min_y = 12.5, -12.5
        self.max_z = 1.5

        # PID gains for altitude
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.2
        self.altitude_error_integral = 0.0
        self.last_altitude_error = 0.0

        # Waypoints (x, y)
        self.waypoints = [
            (-11, 11), (10, 11),
            (10, 5), (-11, 5),
            (-11, 4), (10, 4),
            (10, 0), (-11, 0),
            (-11, -3.5), (11, -3.5),
            (10, -8), (-11, -8)
        ]
        self.current_index = 0

        # State
        self.position = (0.0, 0.0, 0.0)
        self.roll = 0.0
        self.pitch = 0.0
        self.obstacle_detected = False

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)

    def imu_callback(self, msg):
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

    def scan_callback(self, msg):
        self.obstacle_detected = any(r < self.obstacle_threshold for r in msg.ranges if r > 0.1)

    def control_loop(self):
        x, y, z = self.position

        # Clamp position to world bounds
        x = max(min(x, self.max_x), self.min_x)
        y = max(min(y, self.max_y), self.min_y)
        z = min(z, self.max_z)

        # Skip waypoints outside bounds
        while self.current_index < len(self.waypoints):
            tx, ty = self.waypoints[self.current_index]
            if self.min_x <= tx <= self.max_x and self.min_y <= ty <= self.max_y:
                break
            self.get_logger().warn(f"Skipping waypoint {tx, ty} outside bounds.")
            self.current_index += 1

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed.")
            return

        target_x, target_y = self.waypoints[self.current_index]
        self.get_logger().info(f'Current position: {(x, y, z)}, Target: {(target_x, target_y)}')

        twist = Twist()

        # Altitude PID control
        error = self.target_z - z
        self.altitude_error_integral += error * 0.1  # dt = 0.1s
        derivative = (error - self.last_altitude_error) / 0.1
        self.last_altitude_error = error

        pid_output = self.Kp * error + self.Ki * self.altitude_error_integral + self.Kd * derivative
        twist.linear.z = max(min(pid_output, 0.5), -0.5)  # Clamp thrust

        # Obstacle avoidance
        if self.obstacle_detected:
            twist.angular.z = self.angular_speed
            twist.linear.x = 0.0
        else:
            # Move toward current waypoint
            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)

            if distance < 0.5:
                self.current_index = min(self.current_index + 1, len(self.waypoints) - 1)
            else:
                angle = math.atan2(dy, dx)
                twist.linear.x = self.linear_speed
                twist.angular.z = angle - self.get_yaw()

        # IMU stabilization
        twist.angular.x = -self.roll * self.imu_damping
        twist.angular.y = -self.pitch * self.imu_damping

        self.cmd_pub.publish(twist)

    def get_yaw(self):
        # Simplified: assumes flat orientation
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = MazeWanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
