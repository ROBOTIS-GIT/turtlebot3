#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Ryan Shim, Gilbert, YeonSoo Noh

import rclpy
from geometry_msgs.msg import Twist

from turtlebot3_example.turtlebot3_relative_move.turtlebot3_relative_move \
    import Turtlebot3RelativeMove


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_relative_move = Turtlebot3RelativeMove()
    try:
        rclpy.spin(turtlebot3_relative_move)
    except KeyboardInterrupt:
        pass
    finally:
        stop_twist = Twist()
        turtlebot3_relative_move.cmd_vel_pub.publish(stop_twist)

        turtlebot3_relative_move.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
