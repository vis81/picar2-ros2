from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share    = Path(get_package_share_directory('picar2_description'))
    bringup_share = Path(get_package_share_directory('picar2_bringup'))
    ros_gz_sim_share = Path(get_package_share_directory('ros_gz_sim'))

    world_file = str(bringup_share / 'worlds' / 'room.sdf')

    robot_description = ParameterValue(
        Command([
            'xacro ', str(desc_share / 'urdf' / 'picar2.urdf.xacro'),
            ' use_sim:=true',
        ]),
        value_type=str)

    controllers_yaml = str(bringup_share / 'config' / 'controllers.yaml')

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_sim_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn robot entity from robot_description parameter
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  'picar2',
            '-topic', 'robot_description',
            '-z',     '0.05',
        ],
        output='screen',
    )

    # ── Core ROS nodes (same as real robot) ───────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
    )

    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '--controller-manager-timeout', '60'],
    )

    cmd_vel_relay = Node(
        package='picar2_bringup',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'use_sim_time': True,
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[str(bringup_share / 'config' / 'ekf.yaml'), {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odom')],
    )

    # ── ros_gz_bridge — sensor topics Gazebo → ROS ────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': str(bringup_share / 'config' / 'gz_bridge.yaml'),
        }],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        jsb_spawner,
        ackermann_spawner,
        cmd_vel_relay,
        imu_filter,
        ekf_node,
    ])
