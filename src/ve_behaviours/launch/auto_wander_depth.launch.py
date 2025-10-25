from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ve_behaviours',
            executable='auto_wander_depth',
            name='auto_wander_depth',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel'   # change if your robot is namespaced
            }]
        )
    ])
