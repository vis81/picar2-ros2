from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('picar2_bringup'))
    nav2_yaml = str(bringup_share / 'config' / 'nav2.yaml')

    # Assumes bringup + cartographer are already running.
    # Nav2 subscribes to /map (from cartographer), /odom and TF (from EKF),
    # and /scan (from lidar_node). It publishes /cmd_vel consumed by the relay.
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'smoother_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_yaml],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_yaml],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_yaml],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            output='screen',
            parameters=[nav2_yaml],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_yaml],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            output='screen',
            parameters=[nav2_yaml],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
