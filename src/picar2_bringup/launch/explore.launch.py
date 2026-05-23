from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='explore_lite',
            executable='explore',
            name='explore',
            output='screen',
            parameters=[{
                'robot_base_frame': 'base_footprint',
                'costmap_topic': '/global_costmap/costmap',
                'costmap_updates_topic': '/global_costmap/costmap_updates',
                'visualize': True,
                'planner_frequency': 0.25,      # re-plan frontiers every 4 s
                'progress_timeout': 30.0,        # abandon frontier after 30 s of no progress
                'potential_scale': 1.0,          # distance weight — reduced to avoid nearest-only bias
                'gain_scale': 2.0,               # size weight — prefer large unexplored frontiers
                'transform_tolerance': 0.3,
                'min_frontier_size': 0.5,        # ignore frontiers < 0.5 m (10 costmap cells)
            }],
        ),
    ])
