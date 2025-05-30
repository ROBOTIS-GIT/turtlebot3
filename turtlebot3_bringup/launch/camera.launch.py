#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python.resources import has_resource

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:

    camera_param_name = 'camera'
    camera_param_default = str(0)
    camera_param = LaunchConfiguration(
        camera_param_name,
        default=camera_param_default,
    )
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name,
        default_value=camera_param_default,
        description='camera ID or name'
    )

    format_param_name = 'format'
    format_param_default = str()
    format_param = LaunchConfiguration(
        format_param_name,
        default=format_param_default,
    )
    format_launch_arg = DeclareLaunchArgument(
        format_param_name,
        default_value=format_param_default,
        description='pixel format'
    )

    use_image_view_name = 'use_image_view'
    use_image_view_default = 'false'
    use_image_view_param = LaunchConfiguration(use_image_view_name)
    use_image_view_launch_arg = DeclareLaunchArgument(
        use_image_view_name,
        default_value=use_image_view_default,
        description='Whether to launch image_view (true/false)'
    )

    width_name = 'width'
    width_default = '640'
    width_param = LaunchConfiguration(width_name)
    width_launch_arg = DeclareLaunchArgument(
        width_name,
        default_value=width_default,
        description='Camera image width'
    )

    height_name = 'height'
    height_default = '480'
    height_param = LaunchConfiguration(height_name)
    height_launch_arg = DeclareLaunchArgument(
        height_name,
        default_value=height_default,
        description='Camera image height'
    )

    composable_nodes = [
        ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            parameters=[{
                'camera': camera_param,
                'sensor_mode': '1640:1232',
                'width': width_param,
                'height': height_param,
                'format': format_param,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    if has_resource('packages', 'image_view'):
        composable_nodes.append(
            ComposableNode(
                package='image_view',
                plugin='image_view::ImageViewNode',
                remappings=[('/image', '/camera/image_raw')],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(use_image_view_param),
            )
        )

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
    )

    return LaunchDescription([
        camera_launch_arg,
        format_launch_arg,
        use_image_view_launch_arg,
        width_launch_arg,
        height_launch_arg,
        container,
    ])
