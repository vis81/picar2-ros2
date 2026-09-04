from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('picar2_bringup'))
    nav2_yaml = str(bringup_share / 'config' / 'nav2.yaml')
    # Custom navigate_through_poses BT — same as upstream w/recovery, with
    # the <Spin> node stripped (Ackermann can't spin in place).
    nav_through_poses_bt = str(bringup_share / 'config' / 'nav_through_poses_ackermann.xml')
    # navigate_to_pose needs its own Ackermann-safe tree too: explore_lite uses
    # this action, and the default one has no recovery at all — so a robot
    # pinned by RPP's collision check aborts instead of backing up.
    # Overridable so the stock (no-recovery) tree can be A/B tested against it.
    default_to_pose_bt = str(bringup_share / 'config' / 'nav_to_pose_ackermann.xml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Appended *after* nav2.yaml in every parameters=[...] list, so later values
    # win at the leaf level. This keeps nav2.yaml the single source of truth and
    # makes a config variant a ~10-line overlay instead of a forked 234-line
    # file that silently drifts. Defaults to an empty overlay = no change.
    params_overlay = LaunchConfiguration('params_overlay')

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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav_to_pose_bt', default_value=default_to_pose_bt),
        DeclareLaunchArgument(
            'params_overlay',
            default_value=str(bringup_share / 'config' / 'nav2_overlay_empty.yaml'),
            description='extra params file layered over nav2.yaml (later wins)'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_yaml, params_overlay, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_yaml, params_overlay, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_yaml, params_overlay, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            output='screen',
            parameters=[nav2_yaml, params_overlay, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, params_overlay, {
                'use_sim_time': use_sim_time,
                'default_nav_through_poses_bt_xml': nav_through_poses_bt,
                'default_nav_to_pose_bt_xml': LaunchConfiguration('nav_to_pose_bt'),
            }],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            output='screen',
            parameters=[nav2_yaml, params_overlay, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
