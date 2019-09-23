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
# Authors: Ryan Shim, Gilbert

import math

from geometry_msgs.msg import Twist


class Turtlebot3Path():

    def turn(angle, angular_velocity, step):
        twist = Twist()

        if math.fabs(angle) > 0.01:  # 0.01 is small enough value
            if angle >= math.pi:
                twist.angular.z = -angular_velocity
            elif math.pi > angle and angle >= 0:
                twist.angular.z = angular_velocity
            elif 0 > angle and angle >= -math.pi:
                twist.angular.z = -angular_velocity
            elif angle > -math.pi:
                twist.angular.z = angular_velocity
        else:
            step += 1

        return twist, step

    def go_straight(distance, linear_velocity, step):
        twist = Twist()

        if distance > 0.01:  # 0.01 is small enough value
            twist.linear.x = linear_velocity
        else:
            step += 1

        return twist, step
