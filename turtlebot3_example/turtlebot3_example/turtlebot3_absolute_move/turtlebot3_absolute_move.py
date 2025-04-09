#!/usr/bin/env python3
#
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
#
# Authors: Wonho Yun, Jeonggeun Lim, Ryan Shim, Gilbert

import math
import os

from geometry_msgs.msg import Point
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


class Turtlebot3AbsoluteMove(Node):

    def __init__(self):
        super().__init__('turtlebot3_absolute_move')

        print('TurtleBot3 Absolute Move')
        print('----------------------------------------------')
        print('Enter absolute coordinates in odometry frame')
        print('goal x: absolute x position (unit: m)')
        print('goal y: absolute y position (unit: m)')
        print('goal heading: absolute orientation (range: -180 ~ 180, unit: deg)')
        print('----------------------------------------------')

        self.goal_position = Point()
        self.goal_heading = 0.0
        self.position = Point()
        self.heading = 0.0
        self.position_error = Point()
        self.heading_error = 0.0

        self.angular_speed = 0.15
        self.linear_speed = 0.5

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(CmdVelMsg, 'cmd_vel', qos)
        self.cmd_vel = CmdVelMsg()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.get_odom,
            10)

        self.get_logger().info('Ready to receive goal inputs.')

        self.get_key()

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.position_error.x = self.goal_position.x - self.position.x
        self.position_error.y = self.goal_position.y - self.position.y

        distance = math.sqrt(pow(self.position_error.x, 2) +
                             pow(self.position_error.y, 2))
        goal_direction = math.atan2(self.position_error.y, self.position_error.x)

        if distance > 0.05:
            path_angle = goal_direction - self.heading

            if path_angle < -math.pi:
                path_angle += 2 * math.pi
            elif path_angle > math.pi:
                path_angle -= 2 * math.pi

            self.cmd_vel.angular.z = path_angle
            self.cmd_vel.linear.x = min(self.linear_speed * distance, 0.1)

            self.cmd_vel.angular.z = max(min(self.cmd_vel.angular.z, 1.5), -1.5)

            self.get_logger().info(
                f'Moving to x: {self.goal_position.x:.2f}, y: {self.goal_position.y:.2f} '
                f'(current: {self.position.x:.2f}, {self.position.y:.2f})'
            )

            self.cmd_vel_pub.publish(self.cmd_vel)

        else:
            self.cmd_vel.linear.x = 0.0
            self.heading_error = self.goal_heading - self.heading

            if self.heading_error < -math.pi:
                self.heading_error += 2 * math.pi
            elif self.heading_error > math.pi:
                self.heading_error -= 2 * math.pi

            turn_speed = max(min(abs(self.heading_error) * 1.0, 1.0), 0.1)
            self.cmd_vel.angular.z = turn_speed if self.heading_error > 0 else -turn_speed

            self.get_logger().info(
                f'Rotating to heading: {math.degrees(self.goal_heading):.2f}° '
                f'(current: {math.degrees(self.heading):.2f}°)'
            )

            if abs(math.degrees(self.heading_error)) < 1.0:
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)

                self.get_logger().info(
                    f'Goal reached: x: {self.goal_position.x:.2f}, y: {self.goal_position.y:.2f}, '
                    f'heading: {math.degrees(self.goal_heading):.2f}°'
                )

                self.get_key()

        self.cmd_vel_pub.publish(self.cmd_vel)

    def get_odom(self, msg):
        self.position = msg.pose.pose.position
        _, _, self.heading = self.transfrom_from_quaternion_to_eular(msg.pose.pose.orientation)

    def get_key(self):
        self.goal_position.x = float(input('goal x (absolute): '))
        self.goal_position.y = float(input('goal y (absolute): '))
        self.goal_heading = float(input('goal heading (absolute, degrees): '))

        self.goal_heading = math.radians(self.goal_heading)
        if self.goal_heading > math.pi:
            self.goal_heading -= 2 * math.pi
        elif self.goal_heading < -math.pi:
            self.goal_heading += 2 * math.pi

        self.get_logger().info(
            f'New goal: x: {self.goal_position.x:.2f}, y: {self.goal_position.y:.2f}, '
            f'heading: {math.degrees(self.goal_heading):.2f}°'
        )

    def transfrom_from_quaternion_to_eular(self, q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init()
    node = Turtlebot3AbsoluteMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
