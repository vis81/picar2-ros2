from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port',         default_value='/dev/serial0'),
        DeclareLaunchArgument('target_rpm',   default_value='300.0'),
        DeclareLaunchArgument('frame_id',     default_value='laser_link'),
        DeclareLaunchArgument('angle_offset', default_value='0.0'),

        Node(
            package='picar2_lidar',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'port':         LaunchConfiguration('port'),
                'target_rpm':   LaunchConfiguration('target_rpm'),
                'frame_id':     LaunchConfiguration('frame_id'),
                'angle_offset': LaunchConfiguration('angle_offset'),
            }],
        ),
    ])
