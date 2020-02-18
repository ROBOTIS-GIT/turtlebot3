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

from geometry_msgs.msg import Twist


class Turtlebot3Path():

    def drive_circle(radius, velocity):
        twist = Twist()
        linear_velocity = velocity  # unit: m/s
        angular_velocity = linear_velocity / radius  # unit: rad/s

        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        return twist
