from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='explore_lite',
            executable='explore',
            name='explore',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_base_frame': 'base_footprint',
                'costmap_topic': '/global_costmap/costmap',
                'costmap_updates_topic': '/global_costmap/costmap_updates',
                'visualize': True,
                # 0.1, not 0.25: explore_lite sends a new goal whenever the
                # frontier centroid moves >1 cm, and Nav2 is a single-goal
                # server, so every re-plan preempts the current goal. At 4 s
                # and 0.4 m/s the robot committed for ~1.6 m — barely one
                # turning circle at a 0.5 m minimum radius — so it abandoned
                # half-finished manoeuvres and swung elsewhere.
                'planner_frequency': 0.1,       # re-plan frontiers every 10 s
                'progress_timeout': 15.0,        # abandon frontier after 30 s of no progress
                'potential_scale': 3.0,          # distance weight — reduced to avoid nearest-only bias
                'gain_scale': 1.0,               # size weight — prefer large unexplored frontiers
                'transform_tolerance': 0.3,
                'min_frontier_size': 0.5,        # ignore frontiers < 0.5 m (10 costmap cells)
            }],
        ),
    ])
