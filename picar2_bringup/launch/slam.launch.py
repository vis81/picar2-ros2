from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('picar2_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Empty → fresh map. Non-empty (path without extension) → load that pose
    # graph and keep mapping. Pair with `make save-map` to roundtrip.
    map_file = LaunchConfiguration('map_file')
    # 'mapping' (default, accept new scans into the map) or 'localization'
    # (freeze the loaded map, just track the robot against it). Localization
    # mode requires a different executable that subscribes to /initialpose.
    mode = LaunchConfiguration('mode')
    executable = LaunchConfiguration('executable')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_file',     default_value=''),
        DeclareLaunchArgument('mode',         default_value='mapping'),
        DeclareLaunchArgument('executable',   default_value='async_slam_toolbox_node'),

        # slam_toolbox in Jazzy is a nav2 lifecycle node — needs explicit configure+activate
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart':    True,
                'node_names':   ['slam_toolbox'],
            }],
        ),
        Node(
            package='slam_toolbox',
            executable=executable,
            name='slam_toolbox',
            output='screen',
            parameters=[
                str(bringup_share / 'config' / 'slam.yaml'),
                {
                    'use_sim_time':      use_sim_time,
                    'map_file_name':     map_file,
                    'map_start_at_dock': True,
                    'mode':              mode,
                },
            ],
        ),
    ])
