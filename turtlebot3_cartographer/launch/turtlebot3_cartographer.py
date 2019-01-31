# Copyright 2017 Open Source Robotics Foundation, Inc.
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

# /* Author: Darby Lim */

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = launch.substitutions.LaunchConfiguration('cartographer_config_dir', 
                                                                        default=os.path.join(turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = launch.substitutions.LaunchConfiguration('configuration_basename', default='turtlebot3_lds_2d.lua')

    resolution = launch.substitutions.LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = launch.substitutions.LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        launch.actions.DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        launch_ros.actions.Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            node_name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename]),

        launch.actions.DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        launch.actions.DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        launch_ros.actions.Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            node_name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
    ])