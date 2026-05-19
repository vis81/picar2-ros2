from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port',     default_value='/dev/serial0'),
        DeclareLaunchArgument('frame_id', default_value='laser_link'),

        Node(
            package='picar2_lidar',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'port':         LaunchConfiguration('port'),
                'frame_id':     LaunchConfiguration('frame_id'),
                'target_rpm':   300.0,
                'angle_offset': -2.8,
            }],
        ),
    ])
