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
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('turtlebot3_LDS')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    scan = LaserScan()
    r = rospy.Rate(10)
    LIDAR_ERR = 0.05

    while not rospy.is_shutdown():
        scan = rospy.wait_for_message("/scan", LaserScan)
        twist = Twist()
        scan_filter = []
        twist.linear.x = 0.5
        twist.angular.z = 0.0

        for i in range(360):
            if i <= 15 or i > 335:
                if scan.ranges[i] >= LIDAR_ERR:
                    scan_filter.append(scan.ranges[i])

        if min(scan_filter) < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.05
            twist.angular.z = 0.0

        print(min(scan_filter))
        cmd_pub.publish(twist)
        r.sleep()

