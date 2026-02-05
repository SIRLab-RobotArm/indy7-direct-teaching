#!/usr/bin/env python3
"""Launch file for direct teaching node only (assumes indy_driver is already running)"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for direct teaching."""

    # Declare launch arguments
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

    # Direct teaching node
    direct_teaching_node = Node(
        package='indy7_gripper_teleop',
        executable='teaching_control.py',
        name='direct_teaching_node',
        output='screen',
        parameters=[{
            'data_dir': LaunchConfiguration('data_dir'),
            'auto_enable_teaching': LaunchConfiguration('auto_enable_teaching'),
        }]
    )

    return LaunchDescription([
        data_dir_arg,
        auto_enable_arg,
        direct_teaching_node,
    ])
