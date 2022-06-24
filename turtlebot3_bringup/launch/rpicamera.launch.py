#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    camera_params = LaunchConfiguration(
        'camera_params',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            'camera.yaml'))

    return LaunchDescription([

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[camera_params],
            arguments=['--ros-args', '--remap', '__ns:=/camera'],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw/uncompressed'),
            ],
            output='screen'
            ),
    ])
