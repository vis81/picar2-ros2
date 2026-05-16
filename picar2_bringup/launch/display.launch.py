from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = Path(get_package_share_directory('picar2_description'))
    bringup_share = Path(get_package_share_directory('picar2_bringup'))

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch joint_state_publisher_gui for interactive joint control')

    robot_description = ParameterValue(
        Command(['xacro ', str(desc_share / 'urdf' / 'picar2.urdf.xacro')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(bringup_share / 'config' / 'display.rviz')],
        output='screen')

    return LaunchDescription([
        gui_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
