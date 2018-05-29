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
import actionlib
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from turtlebot3_msgs.msg import SensorState
import turtlebot3_example.msg 
import numpy as np

class Turtlebot3Action(object):
    _feedback = turtlebot3_example.msg.Turtlebot3ActionFeedback()
    _result = turtlebot3_example.msg.Turtlebot3ActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, turtlebot3_example.msg.Turtlebot3Action,
                                                execute_cb=self.execute_cb, auto_start=False)
        self.stats_sub = rospy.Subscriber('joint_states', JointState, self.get_state)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.get_odom)
        self.init_stats = True
        self._as.start()
        rospy.loginfo('Server On')

    def get_odom(self, odom):
        self.position = Point()
        self.position = odom.pose.pose.position

    def get_state(self, data):
        TICK2RAD = 0.001533981
        last_pos = 0.0
        diff_pos = 0.0
        cur_pos = 0.0
        encoder = 0

        cur_pos = data.position[0]
        diff_pos = cur_pos - last_pos
        encoder = encoder + (diff_pos / TICK2RAD)
        self.right_encoder = encoder

    def turn(self, angle):
        if self.init_stats:
            self.init_right_encoder = self.right_encoder
            self.init_stats = False
        diff_encoder = (np.deg2rad(angle) * 0.080) / (0.207 / 4096)
        while (abs(self.init_right_encoder - self.right_encoder) < abs(diff_encoder)):
            if diff_encoder >= 0:
                self.twist.angular.z = -0.5
            else:
                self.twist.angular.z = 0.5

            self.cmd_pub.publish(self.twist)
            self.r.sleep()
        self.init_stats = True
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)
        self.r.sleep()

    def go_front(self, lenght, count):
        if count == 0:
            while self.position.x  < lenght:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
        elif count == 1:
            while self.position.y < lenght:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
        elif count == 2:
            while self.position.x > lenght:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
        else:
            while self.position.y > lenght:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
        self.twist.linear.x = 0.0
        self.cmd_pub.publish(self.twist)
        self.r.sleep()

    def execute_cb(self, goal):
        position = Point()
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.twist = Twist()
        self.r = rospy.Rate(15)
        self.r1 = rospy.Rate(1)
        success = True
        mode = goal.goal.x
        patrol_count = int(goal.goal.z)
        circle_mode = True
        half_patrol = False
        circle_count = 0

        for i in range(patrol_count):
            if mode == 1:
                area = [0,0,0,0]
                area[0] = goal.goal.y
                area[1] = goal.goal.y
                for i in range(4):
                    self.go_front(area[i], i)
                    self.r1.sleep()
                    self.turn(-90)

            elif mode == 2:
                area = [0, 0, 0]
                area[0] = goal.goal.y
                area[1] = goal.goal.y
                for i in range(3):
                    self.go_front(area[i], i)
                    self.turn(-120)
            elif mode == 3:
                while(circle_mode):
                    if self.position.x < -goal.goal.y / 2:
                        half_patrol = True
                        count_flag = True
                    else:
                        self.twist.linear.x = goal.goal.y / 2
                        self.twist.angular.z = 0.5
                    if half_patrol == True and self.position.x > 0:
                        if count_flag == True:
                            circle_count = circle_count + 1
                            count_flag = False
                        half_patrol = False
                        if circle_count == patrol_count:
                            circle_mode = False
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0
                    self.cmd_pub.publish(self.twist)
                    self.r.sleep()

        if success:
            self._result = 0
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('turtlebot3')
    server = Turtlebot3Action(rospy.get_name())
    rospy.spin()
    