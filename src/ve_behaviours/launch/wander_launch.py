from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ve_behaviours')
    params_file = os.path.join(pkg_share, 'configs', 'wander_params.yaml')

    return LaunchDescription([
        Node(
            package='ve_behaviours',
            executable='wander_node',
            name='ve_wander_node',
            output='screen',
            parameters=[params_file]
        )
    ])
