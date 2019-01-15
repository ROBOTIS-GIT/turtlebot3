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
from ros2run.api import get_executable_path

def launch(launch_descriptor, argv):
    ld = launch_descriptor
    # ld.add_process(
    #     cmd=[get_executable_path(package_name=package, executable_name='occupancy_grid_node_main'),
    #         '-resolution', '0.05'],
    #     name='occupancy_grid_node',
    #     exit_handler=restart_exit_handler,
    # )
    package = 'cartographer_ros'
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = os.path.join(turtlebot3_cartographer_prefix, 'config')
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='cartographer_node'),
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'turtlebot3_lds_2d.lua'
        ],
        name='cartographer_node',
    )

    return ld