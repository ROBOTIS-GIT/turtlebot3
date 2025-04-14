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

import math
import os
import sys
import termios

from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


ros_distro = os.environ.get('ROS_DISTRO', 'humble').lower()
if ros_distro == 'humble':
    from geometry_msgs.msg import Twist as CmdVelMsg
else:
    from geometry_msgs.msg import TwistStamped as CmdVelMsg

terminal_msg = """
TurtleBot3 Relative Move
------------------------------------------------------
From the current pose,
x: goal position x (unit: m)
y: goal position y (unit: m)
theta: goal orientation (range: -180 ~ 180, unit: deg)
------------------------------------------------------
"""


class Turtlebot3Path():

    @staticmethod
    def turn(angle, angular_velocity, step):
        twist = CmdVelMsg()

        angle = math.atan2(math.sin(angle), math.cos(angle))

        if abs(angle) > 0.01:
            twist.angular.z = angular_velocity if angle > 0 else -angular_velocity
        else:
            step += 1

        return twist, step

    @staticmethod
    def go_straight(distance, linear_velocity, step):
        twist = CmdVelMsg()

        if distance > 0.01:
            twist.linear.x = linear_velocity
        else:
            step += 1

        return twist, step


class Turtlebot3RelativeMove(Node):

    def __init__(self):
        super().__init__('turtlebot3_relative_move')

        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 1
        self.get_key_state = False
        self.init_odom_state = False

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(CmdVelMsg, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        self.update_timer = self.create_timer(0.010, self.update_callback)

        self.get_logger().info('TurtleBot3 relative move node has been initialised.')

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        self.init_odom_state = True

    def update_callback(self):
        if self.init_odom_state:
            self.generate_path()

    def generate_path(self):
        twist = CmdVelMsg()
        if not self.init_odom_state:
            return

        if not self.get_key_state:
            input_x, input_y, input_theta = self.get_key()
            input_x_global = (
                math.cos(self.last_pose_theta) * input_x - math.sin(self.last_pose_theta) * input_y
            )
            input_y_global = (
                math.sin(self.last_pose_theta) * input_x + math.cos(self.last_pose_theta) * input_y
                )

            self.goal_pose_x = self.last_pose_x + input_x_global
            self.goal_pose_y = self.last_pose_y + input_y_global
            self.goal_pose_theta = self.last_pose_theta + input_theta
            self.get_key_state = True

        else:
            if self.step == 1:
                path_theta = math.atan2(
                    self.goal_pose_y - self.last_pose_y,
                    self.goal_pose_x - self.last_pose_x)
                angle = path_theta - self.last_pose_theta
                angular_velocity = 0.3

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            elif self.step == 2:
                distance = math.sqrt(
                    (self.goal_pose_x - self.last_pose_x)**2 +
                    (self.goal_pose_y - self.last_pose_y)**2)
                linear_velocity = 0.1

                twist, self.step = Turtlebot3Path.go_straight(distance, linear_velocity, self.step)

            elif self.step == 3:
                angle = self.goal_pose_theta - self.last_pose_theta
                angular_velocity = 0.3

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            elif self.step == 4:
                self.step = 1
                self.get_key_state = False

            if ros_distro == 'humble':
                self.cmd_vel_pub.publish(twist)
            else:
                stamped = CmdVelMsg()
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.twist = twist
                self.cmd_vel_pub.publish(stamped)

    def get_key(self):
        print(terminal_msg)
        while True:
            try:
                input_x = float(input('Input x: '))
                break
            except ValueError:
                self.get_logger().info('Invalid input! Please enter a number for x.')
        while True:
            try:
                input_y = float(input('Input y: '))
                break
            except ValueError:
                self.get_logger().info('Invalid input! Please enter a number for y.')
        while True:
            try:
                input_theta = float(input('Input theta (deg): '))
                if -180 <= input_theta <= 180:
                    break
                else:
                    self.get_logger().info('Enter a value for theta between -180 and 180.')
            except ValueError:
                self.get_logger().info('Invalid input! Please enter a number for theta.')

        input_theta = numpy.deg2rad(input_theta)

        settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return input_x, input_y, input_theta

    def euler_from_quaternion(self, quat):
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


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3RelativeMove()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        stop_twist = CmdVelMsg()
        node.cmd_vel_pub.publish(stop_twist)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
