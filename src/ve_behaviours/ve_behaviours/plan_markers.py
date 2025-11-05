#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class PlanMarkers(Node):
    def __init__(self):
        super().__init__('plan_markers')
        self.sub = self.create_subscription(Path, '/planned_path', self.cb, 10)
        self.pub = self.create_publisher(MarkerArray, '/planned_markers', 10)

    def cb(self, msg: Path):
        ma = MarkerArray()
        # Dots (SPHERE_LIST)
        dots = Marker()
        dots.header.frame_id = 'map'
        dots.ns = 'waypoints'
        dots.id = 0
        dots.type = Marker.SPHERE_LIST
        dots.action = Marker.ADD
        dots.scale.x = dots.scale.y = dots.scale.z = 0.15  # 15 cm spheres
        dots.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        for p in msg.poses:
            pt = Point(x=p.pose.position.x, y=p.pose.position.y, z=0.03)
            dots.points.append(pt)
        ma.markers.append(dots)

        # Labels (TEXT_VIEW_FACING)
        for i, p in enumerate(msg.poses):
            lab = Marker()
            lab.header.frame_id = 'map'
            lab.ns = 'labels'
            lab.id = i + 1
            lab.type = Marker.TEXT_VIEW_FACING
            lab.action = Marker.ADD
            lab.pose.position.x = p.pose.position.x
            lab.pose.position.y = p.pose.position.y
            lab.pose.position.z = 0.35
            lab.scale.z = 0.25  # text height (m)
            lab.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            lab.text = str(i)
            ma.markers.append(lab)

        self.pub.publish(ma)
        self.get_logger().info(f'Published markers for {len(msg.poses)} waypoints.')

def main():
    rclpy.init()
    node = PlanMarkers()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
