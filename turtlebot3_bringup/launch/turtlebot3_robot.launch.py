# /*******************************************************************************
# * Copyright 2017 ROBOTIS CO., LTD.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# *******************************************************************************/

# /* Author: Darby Lim */

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_burger.urdf')
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen',
            parameters=[{' use_sim_time': use_sim_time}],
            arguments=[urdf]),
    ])