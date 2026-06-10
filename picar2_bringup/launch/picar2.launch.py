from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    desc_share    = Path(get_package_share_directory('picar2_description'))
    bringup_share = Path(get_package_share_directory('picar2_bringup'))

    port_arg  = DeclareLaunchArgument('port',  default_value='/dev/ttyYahboom0')
    baud_arg  = DeclareLaunchArgument('baud',  default_value='460800')
    lidar_arg = DeclareLaunchArgument(
        'lidar', default_value='lds02rr',
        description='LiDAR model: lds02rr | ld19 | none')
    use_mag_arg = DeclareLaunchArgument('use_mag', default_value='false',
                                        description='Enable magnetometer fusion in Madgwick filter')
    use_ld07_arg = DeclareLaunchArgument('use_ld07', default_value='false',
                                         description='Enable LD07 front depth sensor (replaced by SEN0628)')
    use_sen0628_arg = DeclareLaunchArgument('use_sen0628', default_value='true',
                                             description='Enable SEN0628 matrix ToF front sensor')
    sen0628_port_arg = DeclareLaunchArgument('sen0628_port', default_value='/dev/sen0628',
                                              description='Serial port for SEN0628')
    use_foxglove_arg = DeclareLaunchArgument('use_foxglove', default_value='true',
                                              description='Enable foxglove_bridge WebSocket server')
    foxglove_port_arg = DeclareLaunchArgument('foxglove_port', default_value='8765',
                                               description='Port for foxglove_bridge WebSocket server')
    use_vizanti_arg = DeclareLaunchArgument('use_vizanti', default_value='true',
                                             description='Enable Vizanti web mission planner')
    vizanti_port_arg = DeclareLaunchArgument('vizanti_port', default_value='5000',
                                              description='Port for Vizanti Flask UI')
    vizanti_rosbridge_port_arg = DeclareLaunchArgument('vizanti_rosbridge_port', default_value='5001',
                                                        description='Port for Vizanti rosbridge WebSocket')
    calib_arg = DeclareLaunchArgument(
        'calib_file',
        default_value=str(bringup_share / 'config' / 'imu_calib.yaml'),
        description='imu-calib-ros calibration file (gyro + accel bias/scale)')

    robot_description = ParameterValue(
        Command([
            'xacro ', str(desc_share / 'urdf' / 'picar2.urdf.xacro'),
            ' port:=', LaunchConfiguration('port'),
            ' baud:=', LaunchConfiguration('baud'),
            ' lidar:=', LaunchConfiguration('lidar'),
        ]),
        value_type=str)

    controllers_yaml = str(bringup_share / 'config' / 'controllers.yaml')

    def lidar_is(model):
        return IfCondition(EqualsSubstitution(LaunchConfiguration('lidar'), model))

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

    # Apply gyro/accel calibration (bias + scale) before Madgwick filtering.
    # Calibration file generated by: ros2 run imu_calib do_calib
    imu_calib = Node(
        package='imu_calib',
        executable='apply_calib_node',
        parameters=[{
            'calib_file':       LaunchConfiguration('calib_file'),
            'calibrate_gyros':  True,
            'gyro_calib_samples': 100,
        }],
        remappings=[
            ('raw',       '/imu/data_raw'),
            ('corrected', '/imu/data_corrected'),
        ],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[{
            'use_mag': LaunchConfiguration('use_mag'),
            'publish_tf': False,
            'world_frame': 'enu',
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_corrected'),
            ('/imu/mag',      '/imu/mag_unbiased'),
        ],
    )

    # ── Magnetometer bias calibration pipeline ───────────────────────────────
    # Only active when use_mag:=true. Loads hard-iron bias from calibration
    # file, subtracts it from /imu/mag and republishes as /imu/mag_unbiased.
    # Calibrate with: ros2 service call /calibrate_magnetometer std_srvs/srv/Trigger {}
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
        }],
    )

    mag_bias_remover = Node(
        package='magnetometer_pipeline',
        executable='magnetometer_bias_remover_node',
        name='magnetometer_bias_remover',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mag')),
    )

    # ── LiDAR: lds02rr (Neato XV-11 / LDS02RR, GPIO motor PWM) ──────────────
    lidar_lds02rr = Node(
        package='lds02rr_lidar',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        condition=lidar_is('lds02rr'),
        parameters=[{
            'target_rpm':   300.0,
            'angle_offset': -2.77,
            'signal_min':   20,
        }],
        remappings=[('/scan', '/lidar_node/scan')],
    )

    # ── LiDAR: ld19 (LDRobot LD19, lifecycle composable node) ────────────────
    # Uses picar2_bringup/config/lidar_ld19.yaml (serial port: /dev/ldlidar).
    # Skips ldlidar_node's own robot_state_publisher — picar2 already has one.
    lidar_ld19_config = str(bringup_share / 'config' / 'lidar_ld19.yaml')

    lidar_ld19_container = ComposableNodeContainer(
        name='lidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='ldlidar_component',
                plugin='ldlidar::LdLidarComponent',
                name='lidar_node',
                parameters=[lidar_ld19_config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        condition=lidar_is('ld19'),
    )

    lidar_ld19_lc_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lidar_lifecycle_manager',
        output='screen',
        condition=lidar_is('ld19'),
        parameters=[{
            'autostart':  True,
            'node_names': ['lidar_node'],
        }],
    )

    # ── SEN0628 matrix ToF sensor — front of car (replaces LD07) ────────────
    # Publishes /sen0628/pointcloud in sen0628_link frame.
    sen0628_node = LifecycleNode(
        package='tof_imager_ros',
        executable='tof_imager_publisher',
        name='tof_imager',
        namespace='',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sen0628')),
        parameters=[
            str(Path(get_package_share_directory('tof_imager_ros')) / 'config' / 'sensor_params.yaml'),
            {'frame_id': 'sen0628_link',
             'serial_port': LaunchConfiguration('sen0628_port')},
        ],
        remappings=[('pointcloud', '/sen0628/pointcloud')],
    )
    sen0628_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(sen0628_node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    sen0628_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=sen0628_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(sen0628_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    # Statistical outlier removal: removes points whose mean distance to their

    # ── LD07 structured-light depth sensor — front of car ────────────────────
    # Publishes /ld07/scan in ld07_link frame. Serial port: /dev/ld07 (udev).
    ld07_node = Node(
        package='ldrobot_ld07',
        executable='ldrobot_ld07_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ld07')),
        parameters=[
            str(Path(get_package_share_directory('ldrobot_ld07')) / 'params' / 'ld07.yaml')
        ],
    )

    # ── Vizanti — mobile-friendly web visualizer / mission planner ─────────
    # Open browser at http://<pi-ip>:5000 to access the UI.
    # Includes vizanti_server (Flask), rosbridge_server, rosapi,
    # vizanti_tf_consolidator (C++), and vizanti_service_handler.
    vizanti_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('vizanti_server')) /
                'launch' / 'vizanti_server.launch.py')
        ),
        launch_arguments={
            'port':           LaunchConfiguration('vizanti_port'),
            'port_rosbridge': LaunchConfiguration('vizanti_rosbridge_port'),
            'flask_debug':    'False',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_vizanti')),
    )

    # ── Foxglove bridge — WebSocket server for Foxglove Studio ──────────────
    # Connect from Foxglove Studio (desktop or studio.foxglove.dev) to:
    #   ws://<pi-ip>:8765
    # Exposes all topics, services, parameters, /tf and connection graph.
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_foxglove')),
        parameters=[{
            'port':                    LaunchConfiguration('foxglove_port'),
            'address':                 '0.0.0.0',
            'tls':                     False,
            'topic_whitelist':         ['.*'],
            'param_whitelist':         ['.*'],
            'service_whitelist':       ['.*'],
            'client_topic_whitelist':  ['.*'],
            'send_buffer_limit':       10_000_000,
            'use_sim_time':            False,
            'capabilities':            ['clientPublish', 'parameters', 'parametersSubscribe',
                                        'services', 'connectionGraph', 'assets'],
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        lidar_arg,
        use_mag_arg,
        use_ld07_arg,
        use_sen0628_arg,
        sen0628_port_arg,
        use_foxglove_arg,
        foxglove_port_arg,
        use_vizanti_arg,
        vizanti_port_arg,
        vizanti_rosbridge_port_arg,
        calib_arg,
        ros2_control_node,
        robot_state_publisher,
        jsb_spawner,
        ackermann_spawner,
        cmd_vel_relay,
        imu_calib,
        mag_bias_observer,
        mag_bias_remover,
        imu_filter,
        ekf_node,
        lidar_lds02rr,
        lidar_ld19_container,
        lidar_ld19_lc_mgr,
        sen0628_node,
        sen0628_configure,
        sen0628_activate,
        ld07_node,
        foxglove_bridge,
        vizanti_launch,
    ])
