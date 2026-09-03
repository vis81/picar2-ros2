from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    explorer = LaunchConfiguration('explorer')

    is_ours = PythonExpression(["'", explorer, "' != 'explore_lite'"])
    is_lite = PythonExpression(["'", explorer, "' == 'explore_lite'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'explorer', default_value='frontier',
            description="'frontier' (ours, Ackermann-aware) or 'explore_lite'"),

        # Ours. Aims at drivable stand-off poses facing the unknown, scores
        # candidates with the real planner, and commits to a goal instead of
        # re-sending one every cycle. See frontier_explorer.py for why
        # explore_lite's model doesn't fit a car.
        Node(
            package='picar2_bringup',
            executable='frontier_explorer.py',
            name='frontier_explorer',
            output='screen',
            condition=IfCondition(is_ours),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_base_frame': 'base_footprint',
                'costmap_topic': '/global_costmap/costmap',
                'plan_period': 2.0,
                'free_threshold': 50,
                'min_frontier_size': 0.4,
                'min_goal_distance': 0.5,
                'turn_radius': 0.5,
                'turn_weight': 1.0,
                'gain_weight': 2.0,
                'commit_seconds': 10.0,
                'switch_margin': 1.5,
                'useful_radius': 1.0,
                'arrive_radius': 0.4,
                'stuck_seconds': 25.0,
                'stuck_distance': 0.2,
                'futile_distance': 0.15,
                'verify_top_k': 3,
                'blacklist_seconds': 45.0,
                'blacklist_radius': 0.5,
            }],
        ),

        # Kept for A/B comparison: explorer:=explore_lite
        Node(
            package='explore_lite',
            executable='explore',
            name='explore',
            output='screen',
            condition=IfCondition(is_lite),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_base_frame': 'base_footprint',
                'costmap_topic': '/global_costmap/costmap',
                'costmap_updates_topic': '/global_costmap/costmap_updates',
                'visualize': True,
                # 0.1, not 0.25: explore_lite sends a new goal whenever the
                # frontier centroid moves >1 cm, and Nav2 is a single-goal
                # server, so every re-plan preempts the current goal.
                'planner_frequency': 0.1,
                'progress_timeout': 15.0,
                'potential_scale': 3.0,
                'gain_scale': 1.0,
                'transform_tolerance': 0.3,
                'min_frontier_size': 0.5,
            }],
        ),
    ])
