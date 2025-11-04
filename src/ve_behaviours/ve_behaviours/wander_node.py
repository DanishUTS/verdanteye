import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math

class WanderNode(Node):
    def __init__(self):
        super().__init__('wander_node')

        # Declare parameters
        self.declare_parameter('linear_speed', 1.4)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('altitude_target', 2.0)
        self.declare_parameter('altitude_tolerance', 0.1)
        self.declare_parameter('obstacle_distance_threshold', 1.5)
        self.declare_parameter('imu_damping_factor', 0.5)

        # Load parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.altitude_target = self.get_parameter('altitude_target').value
        self.altitude_tolerance = self.get_parameter('altitude_tolerance').value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.imu_damping = self.get_parameter('imu_damping_factor').value

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Internal state
        self.altitude = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.obstacle_detected = False

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.altitude = msg.pose.pose.position.z

    def imu_callback(self, msg):
        # Convert quaternion to roll/pitch
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

    def scan_callback(self, msg):
        self.obstacle_detected = any(r < self.obstacle_threshold for r in msg.ranges if r > 0.1)

    def control_loop(self):
        twist = Twist()

        # Clamp altitude strictly to target
        if self.altitude < self.altitude_target - self.altitude_tolerance:
            twist.linear.z = 0.5  # ascend
        elif self.altitude > self.altitude_target + self.altitude_tolerance:
            twist.linear.z = -0.5  # descend
        else:
            twist.linear.z = 0.0  # hold altitude

        # HARD LIMIT: prevent any upward thrust if already above target
        if self.altitude >= self.altitude_target:
            twist.linear.z = min(twist.linear.z, 0.0)  # never ascend

        # Obstacle avoidance
        if self.obstacle_detected:
            twist.angular.z = self.angular_speed
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        # IMU-based upright stabilization
        twist.angular.x = -self.roll * self.imu_damping
        twist.angular.y = -self.pitch * self.imu_damping

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
