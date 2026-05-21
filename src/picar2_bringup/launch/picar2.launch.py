from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share    = Path(get_package_share_directory('picar2_description'))
    bringup_share = Path(get_package_share_directory('picar2_bringup'))

    port_arg  = DeclareLaunchArgument('port',  default_value='/dev/ttyYahboom0')
    baud_arg  = DeclareLaunchArgument('baud',  default_value='460800')
    lidar_arg = DeclareLaunchArgument('lidar', default_value='true',
                                      description='Launch the LiDAR node')

    robot_description = ParameterValue(
        Command([
            'xacro ', str(desc_share / 'urdf' / 'picar2.urdf.xacro'),
            ' port:=', LaunchConfiguration('port'),
            ' baud:=', LaunchConfiguration('baud'),
        ]),
        value_type=str)

    controllers_yaml = str(bringup_share / 'config' / 'controllers.yaml')

    # controller_manager loads the hardware plugin and runs the control loop
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='both',
    )

    # robot_state_publisher converts joint_states → TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Spawners — retry until controller_manager is up
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
    )

    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '--controller-manager-timeout', '30'],
    )

    cmd_vel_relay = Node(
        package='picar2_bringup',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[str(bringup_share / 'config' / 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/mag',      '/imu/mag'),
        ],
    )

    lidar_node = Node(
        package='picar2_lidar',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('lidar')),
        parameters=[{
            'target_rpm':   300.0,
            'angle_offset': -2.77,
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        lidar_arg,
        ros2_control_node,
        robot_state_publisher,
        jsb_spawner,
        ackermann_spawner,
        cmd_vel_relay,
        imu_filter,
        ekf_node,
        lidar_node,
    ])
