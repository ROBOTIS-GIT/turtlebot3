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

import sys
import termios

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebot3_msgs.action import Patrol

terminal_msg = """
TurtleBot3 Circle Patrol
------------------------------------------------------
radius: circle radius (unit: m)
------------------------------------------------------
"""


class Turtlebot3PatrolClient(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_client')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.radius = 0.0  # unit: m

        """************************************************************
        ** Initialise ROS clients
        ************************************************************"""
        # Initialise clients
        self.action_client = ActionClient(self, Patrol, 'patrol')

        self.get_logger().info("Turtlebot3 patrol node has been initialised.")

        # Get keyboard input and send goal
        self.get_key()

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def get_key(self):
        print(terminal_msg)
        settings = termios.tcgetattr(sys.stdin)
        input_radius = input("Input radius: ")

        self.radius = float(input_radius)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        goal_msg = Patrol.Goal()
        goal_msg.radius = self.radius

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(
            'Time left until the robot stops: {0}'.format(feedback.feedback.left_time))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.success))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()
