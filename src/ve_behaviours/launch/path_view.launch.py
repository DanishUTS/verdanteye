from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ve_behaviours')
    map_yaml = os.path.join(pkg_share, 'maps', 'forest.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'frame_id': 'map'}],
    )

    pathplanning = Node(
        package='ve_behaviours',
        executable='pathplanning',
        name='pathplanning',
        output='screen',
        parameters=[{
            'origin_xy': [0.0, 0.0],
            'targets_flat': [0.0,-5.0, -9.0,6.0, 5.0,1.0, -2.0,7.0, 8.0,-3.0, -7.0,1.0],
            'goal_tolerance_m': 1.0,
            'clearance_m': 0.40,
            'spacing_m': 0.75,
            'publish_path': True,
        }],
    )

    markers = Node(
        package='ve_behaviours',
        executable='plan_markers',
        name='plan_markers',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[]
    )

    return LaunchDescription([map_server, pathplanning, markers, rviz])
