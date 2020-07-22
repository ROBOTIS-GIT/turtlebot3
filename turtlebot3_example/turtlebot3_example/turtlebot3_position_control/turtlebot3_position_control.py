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
import numpy
import sys
import termios

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

from turtlebot3_example.turtlebot3_position_control.turtlebot3_path import Turtlebot3Path

terminal_msg = """
Turtlebot3 Position Control
------------------------------------------------------
From the current pose,
x: goal position x (unit: m)
y: goal position y (unit: m)
theta: goal orientation (range: -180 ~ 180, unit: deg)
------------------------------------------------------
"""


class Turtlebot3PositionControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 1
        self.get_key_state = False
        self.init_odom_state = False  # To get the initial pose at the beginning

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(0.010, self.update_callback)  # unit: s

        self.get_logger().info("Turtlebot3 position control node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        self.init_odom_state = True

    def update_callback(self):
        if self.init_odom_state is True:
            self.generate_path()

    def generate_path(self):
        twist = Twist()

        if self.get_key_state is False:
            input_x, input_y, input_theta = self.get_key()
            self.goal_pose_x = self.last_pose_x + input_x
            self.goal_pose_y = self.last_pose_y + input_y
            self.goal_pose_theta = self.last_pose_theta + input_theta
            self.get_key_state = True

        else:
            # Step 1: Turn
            if self.step == 1:
                path_theta = math.atan2(
                    self.goal_pose_y - self.last_pose_y,
                    self.goal_pose_x - self.last_pose_x)
                angle = path_theta - self.last_pose_theta
                angular_velocity = 0.1  # unit: rad/s

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            # Step 2: Go Straight
            elif self.step == 2:
                distance = math.sqrt(
                    (self.goal_pose_x - self.last_pose_x)**2 +
                    (self.goal_pose_y - self.last_pose_y)**2)
                linear_velocity = 0.1  # unit: m/s

                twist, self.step = Turtlebot3Path.go_straight(distance, linear_velocity, self.step)

            # Step 3: Turn
            elif self.step == 3:
                angle = self.goal_pose_theta - self.last_pose_theta
                angular_velocity = 0.1  # unit: rad/s

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            # Reset
            elif self.step == 4:
                self.step = 1
                self.get_key_state = False

            self.cmd_vel_pub.publish(twist)

    def get_key(self):
        # Print terminal message and get inputs
        print(terminal_msg)
        input_x = float(input("Input x: "))
        input_y = float(input("Input y: "))
        input_theta = float(input("Input theta: "))
        while input_theta > 180 or input_theta < -180:
            self.get_logger().info("Enter a value for theta between -180 and 180")
            input_theta = input("Input theta: ")
        input_theta = numpy.deg2rad(input_theta)  # Convert [deg] to [rad]

        settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return input_x, input_y, input_theta

    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""
    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
