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

import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescriptor
from launch.exit_handler import restart_exit_handler
from launch.launcher import DefaultLauncher
from launch.output_handler import ConsoleOutput
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    parser = argparse.ArgumentParser(description='launch amcl turtlebot demo')
    parser.add_argument(
        '--map',
        help='path to map (will be passed to map_server)')
    args = parser.parse_args(argv)

    ld = launch_descriptor

    # package = 'turtlebot2_drivers'
    # ld.add_process(
    #     cmd=[get_executable_path(package_name=package, executable_name='kobuki_node')],
    #     name='kobuki_node',
    #     exit_handler=restart_exit_handler,
    # )
    # package = 'astra_camera'
    # ld.add_process(
    #     cmd=[
    #         get_executable_path(package_name=package, executable_name='astra_camera_node'),
    #         '-dw', '320', '-dh', '240', '-C', '-I'],
    #     name='astra_camera_node',
    #     exit_handler=restart_exit_handler,
    # )
    # package = 'depthimage_to_laserscan'
    # ld.add_process(
    #     cmd=[
    #         get_executable_path(
    #             package_name=package, executable_name='depthimage_to_laserscan_node')],
    #     name='depthimage_to_laserscan_node',
    #     exit_handler=restart_exit_handler,
    # )
    # package = 'tf2_ros'
    # ld.add_process(
    #     # The XYZ/Quat numbers for base_link -> camera_rgb_frame are taken from the
    #     # turtlebot URDF in
    #     # https://github.com/turtlebot/turtlebot/blob/931d045/turtlebot_description/urdf/sensors/astra.urdf.xacro
    #     cmd=[
    #         get_executable_path(
    #             package_name=package, executable_name='static_transform_publisher'),
    #         '-0.087', '-0.0125', '0.287',
    #         '0', '0', '0', '1',
    #         'base_link',
    #         'camera_rgb_frame'
    #     ],
    #     name='static_tf_pub_base_rgb',
    #     exit_handler=restart_exit_handler,
    # )
    # package = 'tf2_ros'
    # ld.add_process(
    #     # The XYZ/Quat numbers for camera_rgb_frame -> camera_depth_frame are taken from the
    #     # turtlebot URDF in
    #     # https://github.com/turtlebot/turtlebot/blob/931d045/turtlebot_description/urdf/sensors/astra.urdf.xacro
    #     cmd=[
    #         get_executable_path(
    #             package_name=package, executable_name='static_transform_publisher'),
    #         '0', '0.0250', '0',
    #         '0', '0', '0', '1',
    #         'camera_rgb_frame',
    #         'camera_depth_frame'
    #     ],
    #     name='static_tf_pub_rgb_depth',
    #     exit_handler=restart_exit_handler,
    # )
    # package = 'joy'
    # ld.add_process(
    #     cmd=[get_executable_path(package_name=package, executable_name='joy_node')],
    #     name='joy_node',
    #     exit_handler=restart_exit_handler,
    # )
    # package = 'teleop_twist_joy'
    # ld.add_process(
    #     cmd=[get_executable_path(package_name=package, executable_name='teleop_node')],
    #     name='teleop_node',
    #     exit_handler=restart_exit_handler,
    # )
    turtlebot2_amcl_share = get_package_share_directory('turtlebot2_amcl')
    map_path = os.path.join(turtlebot2_amcl_share, 'examples', 'osrf_map.yaml')
    if args.map:
        map_path = args.map
    package = 'map_server'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='map_server'), map_path],
        name='map_server',
    )
    package = 'amcl'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='amcl'), '--use-map-topic'],
        name='amcl',
        exit_handler=restart_exit_handler,
        output_handlers=[ConsoleOutput()],
    )

    return ld


def main(argv=sys.argv[1:]):
    launcher = DefaultLauncher()
    launch_descriptor = launch(LaunchDescriptor(), argv)
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())
