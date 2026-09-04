"""Localisation for the navigation benchmark, switchable between three modes.

Exactly one node publishes map->odom in each mode; two would silently fight and
produce a pose that is neither. The runner asserts this at trial start.

  ground_truth : map_server (rasterised map) + gt_localizer publishing map->odom
  slam         : cartographer builds the map and owns map->odom
  amcl         : map_server + AMCL, seeded from ground truth by gt_localizer

The ground-truth bridge runs in *all* modes: in ground_truth it feeds TF, and in
the other two it feeds only the metrics, so path length, clearance and stalls are
always measured against reality rather than against the robot's belief. The
difference between the two is the localisation error, which is the reason for
being able to run these arms against the same scenario.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(get_package_share_directory('picar2_bringup'))
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    is_gt = PythonExpression(["'", mode, "' == 'ground_truth'"])
    is_amcl = PythonExpression(["'", mode, "' == 'amcl'"])
    is_slam = PythonExpression(["'", mode, "' == 'slam'"])
    # both non-SLAM modes need the static map for the global costmap
    needs_map = PythonExpression(["'", mode, "' != 'slam'"])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='ground_truth',
                              description='ground_truth | slam | amcl'),
        DeclareLaunchArgument('map_yaml', default_value='',
                              description='static map; required unless mode:=slam'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='picar2_bench'),
        DeclareLaunchArgument('robot_name', default_value='picar2'),

        # Ground truth, in every mode. Its own bridge process, not the sensor
        # bridge: that one also carries /clock, and adding a 50 Hz stream to it
        # risks delaying clock and perturbing every sim-time timer in the stack.
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='gt_odom_bridge',
            arguments=[[
                '/model/', LaunchConfiguration('robot_name'),
                '/gt_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry']],
            remappings=[
                ([ '/model/', LaunchConfiguration('robot_name'), '/gt_odom'], '/gt/odom')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='picar2_benchmark', executable='gt_localizer',
            name='gt_localizer',
            parameters=[{'use_sim_time': use_sim_time, 'mode': mode}],
            output='screen',
        ),

        # Static map for ground_truth and amcl
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            condition=IfCondition(needs_map),
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
            output='screen',
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            condition=IfCondition(is_amcl),
            parameters=[str(bringup / 'config' / 'amcl.yaml'),
                        {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_benchmark_localization',
            condition=IfCondition(needs_map),
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                # amcl only appears in the amcl arm; map_server always does here
                'node_names': PythonExpression([
                    "['map_server', 'amcl'] if '", mode, "' == 'amcl' else ['map_server']"]),
            }],
            output='screen',
        ),

        # SLAM arm — cartographer owns map->odom, so gt_localizer stays off TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch' / 'cartographer.launch.py')),
            condition=IfCondition(is_slam),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
