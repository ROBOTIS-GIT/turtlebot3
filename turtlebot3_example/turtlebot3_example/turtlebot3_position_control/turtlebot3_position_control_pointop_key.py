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

import rclpy
from rclpy.node import Node
import numpy
import math

from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

class Turtlebot3PositionControlAbsolute(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control_absolute')

        print("TurtleBot3 Absolute Position Control")
        print("----------------------------------------------")
        print("Enter absolute coordinates in odometry frame")
        print("goal x: absolute x position (unit: m)")
        print("goal y: absolute y position (unit: m)")
        print("goal heading: absolute orientation (range: -180 ~ 180, unit: deg)")
        print("----------------------------------------------")

        self.goal_position = Point()
        self.goal_heading = 0.0
        self.position = Point()
        self.heading = 0.0
        self.position_error = Point()
        self.heading_error = 0.0

        self.angular_speed = 0.15
        self.linear_speed = 0.5

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.cmd_vel = Twist()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.get_odom,
            10)
        self.odom_sub

        self.get_logger().info("Ready to receive goal inputs.")

        self.get_key()

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.position_error.x = self.goal_position.x - self.position.x
        self.position_error.y = self.goal_position.y - self.position.y

        distance = math.sqrt(pow(self.position_error.x, 2) + pow(self.position_error.y, 2))
        goal_direction = math.atan2(self.position_error.y, self.position_error.x)

        if distance > 0.05:
            path_angle = goal_direction - self.heading

            if path_angle < -math.pi:
                path_angle += 2 * math.pi
            elif path_angle > math.pi:
                path_angle -= 2 * math.pi

            self.cmd_vel.angular.z = path_angle
            self.cmd_vel.linear.x = min(self.linear_speed * distance, 0.1)

            if self.cmd_vel.angular.z > 1.5:
                self.cmd_vel.angular.z = 1.5
            elif self.cmd_vel.angular.z < -1.5:
                self.cmd_vel.angular.z = -1.5

            self.get_logger().info("Moving to x: {:.2f}, y: {:.2f} (current: {:.2f}, {:.2f})".format(
                self.goal_position.x, self.goal_position.y, self.position.x, self.position.y))

            self.cmd_vel_pub.publish(self.cmd_vel)

        else:
            self.cmd_vel.linear.x = 0.0
            self.heading_error = self.goal_heading - self.heading

            if self.heading_error < -math.pi:
                self.heading_error += 2 * math.pi
            elif self.heading_error > math.pi:
                self.heading_error -= 2 * math.pi

            turn_speed = max(min(abs(self.heading_error) * 1.0, 1.0), 0.1)
            if self.heading_error > 0:
                self.cmd_vel.angular.z = turn_speed
            else:
                self.cmd_vel.angular.z = -turn_speed

            self.get_logger().info("Rotating in place to heading: {:.2f}° (current: {:.2f}°)".format(
                math.degrees(self.goal_heading), math.degrees(self.heading)))

            if abs(math.degrees(self.heading_error)) < 1.0:
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)

                self.get_logger().info("Final goal reached: x: {:.2f}, y: {:.2f}, heading: {:.2f}°".format(
                    self.goal_position.x, self.goal_position.y, math.degrees(self.goal_heading)))

                self.get_key()

        self.cmd_vel_pub.publish(self.cmd_vel)


    def get_odom(self, msg):
        self.position = msg.pose.pose.position
        _, _, self.heading = self.transfrom_from_quaternion_to_eular(msg.pose.pose.orientation)

    def get_key(self):
        self.goal_position.x = float(input("goal x (absolute): "))
        self.goal_position.y = float(input("goal y (absolute): "))
        self.goal_heading = float(input("goal heading (absolute, degrees): "))

        self.goal_heading = math.radians(self.goal_heading)
        if self.goal_heading > math.pi:
            self.goal_heading -= 2 * math.pi
        elif self.goal_heading < -math.pi:
            self.goal_heading += 2 * math.pi

        self.get_logger().info("New goal: x: {:.2f}, y: {:.2f}, heading: {:.2f}°".format(
            self.goal_position.x, self.goal_position.y, math.degrees(self.goal_heading)))

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
    node = Turtlebot3PositionControlAbsolute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
