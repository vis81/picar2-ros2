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
                'planner_frequency': 0.5,       # re-plan frontiers every 2 s
                'progress_timeout': 30.0,        # abandon frontier after 30 s of no progress
                'potential_scale': 3.0,
                'gain_scale': 1.0,
                'transform_tolerance': 0.3,
                'min_frontier_size': 0.25,       # ignore tiny frontiers (m)
            }],
        ),
    ])
