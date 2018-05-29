#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
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
#################################################################################

# Authors: Gilbert #

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState

class Bumper():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.bumper_sub = rospy.Subscriber('sensor_state', SensorState, self.get_bumper, queue_size = 1)
        self.twist = Twist()
        self.bumper()
    def get_bumper(self, sensor):
        self.bumper_state = sensor.bumper
        if self.bumper_state == 1:
            self.twist.linear.x = -0.1
        elif self.bumper_state == 2:
            self.twist.linear.x = 0.1

    def bumper(self):
        rate = rospy.Rate(10)
        self.twist.linear.x = 0.1
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)
            rate.sleep()

def main():
    rospy.init_node('turtlebot3_bumper')
    try:
        bumper = Bumper()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()