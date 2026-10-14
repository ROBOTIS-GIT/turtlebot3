#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
# http://www.apache.org/licenses/LICENSE-2.0
#
# Original author: Darby Lim. Trimmed down for a single fixed robot by Donald Williamson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ---- Fixed settings for this robot: edit here, nowhere else ----
MODEL = 'burger'                  
OPENCR_PORT = '/dev/ttyACM0'
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_PKG = 'ld08_driver'         
LIDAR_LAUNCH = 'ld08.launch.py'   


def generate_launch_description():
    # The included state publisher launch file still reads this variable,
    # so set it here instead of relying on ~/.bashrc.
    os.environ['TURTLEBOT3_MODEL'] = MODEL

    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    param_file = os.path.join(bringup_dir, 'param', MODEL + '.yaml')

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'turtlebot3_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'false',
                         'namespace': ''}.items(),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(LIDAR_PKG), 'launch', LIDAR_LAUNCH)),
        launch_arguments={'port': LIDAR_PORT,
                          'frame_id': 'base_scan',
                          'namespace': ''}.items(),
    )

    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        parameters=[param_file, {'namespace': ''}],
        arguments=['-i', OPENCR_PORT],
        output='screen',
    )

    oled_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_utils'),
            'launch', 'oled_display.launch.py')),
    )

    return LaunchDescription([
        state_publisher,
        lidar,
        turtlebot3_node,
        oled_display,
    ])
