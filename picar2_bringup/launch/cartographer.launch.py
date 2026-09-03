from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _cartographer_node(context):
    """Build the cartographer_node with conditional -load_state_filename
    when load_state is non-empty. Done via OpaqueFunction because
    cartographer_node's arguments aren't ROS params — they're CLI args
    parsed by gflags before ROS init, so we can't include them
    conditionally via a normal Node parameters dict."""
    bringup_share = Path(get_package_share_directory('picar2_bringup'))
    args = [
        '-configuration_directory', str(bringup_share / 'config'),
        '-configuration_basename', 'cartographer.lua',
    ]
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    load_state = LaunchConfiguration('load_state').perform(context).strip()
    load_frozen = LaunchConfiguration('load_frozen').perform(context).strip().lower()
    if load_state:
        args += ['-load_state_filename', load_state]
        if load_frozen == 'true':
            args += ['-load_frozen_state', 'true']
    return [Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=args,
        remappings=[('/imu', '/imu/data'), ('/scan', '/lidar_node/scan')],
    )]


def generate_launch_description():
    return LaunchDescription([
        # Gazebo publishes /clock; without this cartographer stamps against
        # wall time and every scan looks stale. slam.launch.py already had it.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Empty → fresh map. Path to a .pbstream → resume from it.
        DeclareLaunchArgument('load_state', default_value=''),
        # When loading state: 'true' = read-only (pure localization),
        # 'false' = continue mapping. Ignored if load_state is empty.
        DeclareLaunchArgument('load_frozen', default_value='false'),
        OpaqueFunction(function=_cartographer_node),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'resolution': 0.05,
                'publish_period_sec': 1.0,
            }],
        ),
    ])
