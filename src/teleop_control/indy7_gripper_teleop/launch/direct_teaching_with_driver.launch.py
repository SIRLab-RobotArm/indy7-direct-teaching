#!/usr/bin/env python3
"""Launch file for direct teaching with indy_driver"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for direct teaching with driver."""

    # Declare launch arguments
    indy_ip_arg = DeclareLaunchArgument(
        'indy_ip',
        default_value='192.168.0.6',
        description='IP address of the Indy7 robot'
    )

    indy_type_arg = DeclareLaunchArgument(
        'indy_type',
        default_value='indy7',
        description='Type of Indy robot (indy7, indy12, etc.)'
    )

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='~/teaching_data',
        description='Directory to save demonstration data'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable_teaching',
        default_value='true',
        description='Automatically enable teaching mode on startup'
    )

    # Include indy_driver launch file
    indy_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('indy_driver'),
                'launch',
                'indy_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'indy_ip': LaunchConfiguration('indy_ip'),
            'indy_type': LaunchConfiguration('indy_type'),
        }.items()
    )

    # Direct teaching node
    direct_teaching_node = Node(
        package='indy7_gripper_teleop',
        executable='teaching_control.py',
        name='direct_teaching_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'data_dir': LaunchConfiguration('data_dir'),
            'auto_enable_teaching': LaunchConfiguration('auto_enable_teaching'),
        }]
    )

    return LaunchDescription([
        indy_ip_arg,
        indy_type_arg,
        data_dir_arg,
        auto_enable_arg,
        indy_driver_launch,
        direct_teaching_node,
    ])
