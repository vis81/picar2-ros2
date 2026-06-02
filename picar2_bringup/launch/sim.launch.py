from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share    = Path(get_package_share_directory('picar2_description'))
    bringup_share = Path(get_package_share_directory('picar2_bringup'))
    ros_gz_sim_share = Path(get_package_share_directory('ros_gz_sim'))

    default_world = str(bringup_share / 'worlds' / 'room.sdf')

    world_arg = DeclareLaunchArgument('world', default_value=default_world,
                                      description='Full path to Gazebo world file')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0',
                                        description='Robot spawn X position')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0',
                                        description='Robot spawn Y position')
    lidar_arg = DeclareLaunchArgument('lidar', default_value='ld19',
                                      description='Lidar model: ld19 | lds02rr | none')
    use_mag_arg = DeclareLaunchArgument('use_mag', default_value='false',
                                        description='Enable magnetometer fusion in Madgwick filter')
    use_ld07_arg = DeclareLaunchArgument('use_ld07', default_value='false',
                                         description='Enable LD07 scan in costmap (sensor always simulated)')
    use_sen0628_arg = DeclareLaunchArgument('use_sen0628', default_value='true',
                                             description='Enable SEN0628 pointcloud in costmap (sensor always simulated)')

    robot_description = ParameterValue(
        Command([
            'xacro ', str(desc_share / 'urdf' / 'picar2.urdf.xacro'),
            ' use_sim:=true',
            ' lidar:=', LaunchConfiguration('lidar'),
        ]),
        value_type=str)

    controllers_yaml = str(bringup_share / 'config' / 'controllers.yaml')

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_sim_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  'picar2',
            '-topic', 'robot_description',
            '-x',     LaunchConfiguration('spawn_x'),
            '-y',     LaunchConfiguration('spawn_y'),
            '-z',     '0.05',
        ],
        output='screen',
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

    # ── Core ROS nodes ────────────────────────────────────────────────────────
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
            'use_mag':     LaunchConfiguration('use_mag'),
            'publish_tf':  False,
            'world_frame': 'enu',
            'use_sim_time': True,
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/mag',      '/imu/mag_unbiased'),
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[str(bringup_share / 'config' / 'ekf.yaml'), {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odom')],
    )

    # ── Magnetometer bias calibration pipeline ───────────────────────────────
    mag_calib_file = str(bringup_share / 'config' / 'magnetometer_calib.yaml')

    mag_bias_observer = Node(
        package='magnetometer_pipeline',
        executable='magnetometer_bias_observer.py',
        name='mag_bias_observer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mag')),
        parameters=[{
            '2d_mode': True,
            'calibration_file_path': mag_calib_file,
            'load_from_file': True,
            'save_to_file': True,
            'use_sim_time': True,
        }],
    )

    mag_bias_remover = Node(
        package='magnetometer_pipeline',
        executable='magnetometer_bias_remover_node',
        name='magnetometer_bias_remover',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mag')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        lidar_arg,
        use_mag_arg,
        use_ld07_arg,
        use_sen0628_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        jsb_spawner,
        ackermann_spawner,
        cmd_vel_relay,
        mag_bias_observer,
        mag_bias_remover,
        imu_filter,
        ekf_node,
    ])
