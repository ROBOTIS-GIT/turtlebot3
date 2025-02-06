#!/usr/bin/env python3
#
# Copyright 2023 ROBOTIS CO., LTD.
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
# Authors: Jeonggeun Lim, Gilbert


import time
import rclpy
import math
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, Point
from turtlebot3_msgs.action import Turtlebot3

class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')

        print("TurtleBot3 Patrol Server")
        print("----------------------------------------------")

        self._action_server = ActionServer(
            self,
            Turtlebot3,
            'turtlebot3',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback)

        self.goal_msg = Turtlebot3.Goal()
        self.twist = Twist()
        self.position = Point()
        self.rotation = 0.0

        # self.linear_x = 0.2
        # self.angular_z = 4.0 * math.pi / 100.0
        self.linear_x = 1.0
        self.angular_z = 4.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.cmd_vel = Twist()

    def init_twist(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def go_front(self, position, length):
        while(1):
            position += self.twist.linear.x
            if position >= length:
                break;
            self.twist.linear.x = self.linear_x
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            
            time.sleep(1)
        self.init_twist()

    def turn(self, angle, target_angle):
        while (1):
            angle += self.twist.angular.z
            if (angle >= target_angle * math.pi / 180.0):
                break;
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_z
            self.cmd_vel_pub.publish(self.twist)

            time.sleep(1)
        self.init_twist()

    def goal_callback(self, goal_request):
        self.goal_msg = goal_request

        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Turtlebot3.Feedback()

        length = self.goal_msg.goal.y
        iteration = int(self.goal_msg.goal.z)

        while(1):
            if self.goal_msg.goal.x == 1:
                for count in range(iteration):
                    self.square(feedback_msg, goal_handle, length)
                feedback_msg.state = "square patrol complete!!"
                break
            elif self.goal_msg.goal.x == 2:
                for count in range(iteration):
                    self.triangle(feedback_msg, goal_handle, length)
                feedback_msg.state = "triangle patrol complete!!"
                break

        goal_handle.succeed()
        result = Turtlebot3.Result()
        result.result = feedback_msg.state

        return result

    def square(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 8 * (90.0 / 180.0) * math.pi / 100.0

        for i in range(4):
            self.position.x = 0.0
            self.angle = 0.0

            self.go_front(self.position.x, length)
            self.turn(self.angle, 90.0)

            feedback_msg.state = "line " + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        self.init_twist()

    def triangle(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 4 * (120.0 / 180.0) * math.pi / 100.0

        for i in range(3):
            self.position.x = 0.0
            self.angle = 0.0

            self.go_front(self.position.x, length)
            self.turn(self.angle, 120.0)

            feedback_msg.state = "line " + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        self.init_twist()

def main(args=None):
    rclpy.init(args=args)

    turtlebot3_patrol_server = Turtlebot3PatrolServer()

    rclpy.spin(turtlebot3_patrol_server)

if __name__ == '__main__':
    main()