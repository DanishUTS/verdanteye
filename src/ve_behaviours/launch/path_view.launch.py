from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ve_behaviours')
    map_yaml = os.path.join(pkg_share, 'maps', 'forest.yaml')

    # 1) Map server (lifecycle node)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'frame_id': 'map',
        }],
    )

    # 2) Lifecycle manager to auto-configure/activate map_server
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            # optional: make it retry on startup races
            'bond_timeout': 0.0,
            'timeout': 10.0,
        }],
    )

    # 3) Your path planner
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
            'publish_path': True,  # consider publishing with TRANSIENT_LOCAL in code
        }],
    )

    # 4) Waypoint markers (optional)
    markers = Node(
        package='ve_behaviours',
        executable='plan_markers',
        name='plan_markers',
        output='screen'
    )

    # 5) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        map_server,
        lifecycle_mgr,
        pathplanning,
        markers,
        rviz
    ])
