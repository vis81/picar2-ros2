"""AMCL localization on a saved PGM/YAML map.

Brings up:
  - nav2_map_server (publishes /map from the YAML file)
  - nav2_amcl       (Monte-Carlo localization → publishes map→odom TF)
  - lifecycle_manager_localization (configures + activates both)

After launch, set the initial pose in RViz via the '2D Pose Estimate'
tool (publishes geometry_msgs/PoseWithCovarianceStamped on /initialpose).

Args:
  map_yaml      Path to <name>.yaml (default: /ws/maps/home.yaml)
  use_sim_time  default false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('picar2_bringup'))
    amcl_yaml = str(bringup_share / 'config' / 'amcl.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='/ws/maps/home.yaml',
            description='Absolute path to <name>.yaml from save-map / save-cartographer-map'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[amcl_yaml, {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart':    True,
                'node_names':   ['map_server', 'amcl'],
            }],
        ),
    ])
