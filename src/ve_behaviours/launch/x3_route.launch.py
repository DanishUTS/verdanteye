from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ve_behaviours',
            executable='x3_route',
            name='x3_route',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/X3/cmd_vel',
                'odom_topic':    '/X3/odometry',
                'cruise_alt':    3.0,
                'speed_xy':      1.3,
                'speed_z':       0.8,
                # Swap these for a lawn-mower sweep once the rectangle works:
                'waypoints_xy':  [-25,-20, 25,-20, 25,20, -25,20],
                'loiter_secs':   2.0
            }]
        )
    ])
