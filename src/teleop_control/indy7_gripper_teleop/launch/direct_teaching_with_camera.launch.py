#!/usr/bin/env python3
"""Launch file for direct teaching with RealSense D435 camera"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for direct teaching with camera.

    This launch file starts:
    1. Indy7 robot driver
    2. RealSense D435 camera
    3. Direct teaching node with camera synchronization
    """

    # Declare launch arguments
    indy_ip_arg = DeclareLaunchArgument(
        'indy_ip',
        default_value='192.168.1.10',
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
        default_value='false',
        description='Automatically enable teaching mode on startup'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Enable depth image recording'
    )

    resize_images_arg = DeclareLaunchArgument(
        'resize_images',
        default_value='true',
        description='Resize images to 224x224 for training'
    )

    # RealSense camera parameters
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='15',
        description='Camera frame rate (15 or 30)'
    )

    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera image width'
    )

    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera image height'
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

    # RealSense D435 camera node
    # Using realsense2_camera package
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'rgb_camera.profile': [
                LaunchConfiguration('camera_width'),
                LaunchConfiguration('camera_height'),
                LaunchConfiguration('camera_fps')
            ],
            'depth_module.profile': [
                LaunchConfiguration('camera_width'),
                LaunchConfiguration('camera_height'),
                LaunchConfiguration('camera_fps')
            ],
            'align_depth.enable': True,  # Align depth to color
            'pointcloud.enable': False,
        }]
    )

    # Direct teaching node with camera enabled
    direct_teaching_node = Node(
        package='indy7_gripper_teleop',
        executable='teaching_control.py',
        name='direct_teaching_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'data_dir': LaunchConfiguration('data_dir'),
            'auto_enable_teaching': LaunchConfiguration('auto_enable_teaching'),
            'enable_camera': True,
            'enable_depth': LaunchConfiguration('enable_depth'),
            'resize_images': LaunchConfiguration('resize_images'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        indy_ip_arg,
        indy_type_arg,
        data_dir_arg,
        auto_enable_arg,
        enable_depth_arg,
        resize_images_arg,
        camera_fps_arg,
        camera_width_arg,
        camera_height_arg,
        # Nodes
        indy_driver_launch,
        realsense_camera_node,
        direct_teaching_node,
    ])
