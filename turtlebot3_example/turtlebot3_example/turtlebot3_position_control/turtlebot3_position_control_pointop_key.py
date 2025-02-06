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


import rclpy
from rclpy.node import Node
import numpy
import math

from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


class Turtlebot3PositionControlPointOpKey(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control_pointop_key')

        print("TurtleBot3 Position Control")
        print("----------------------------------------------")
        print("From the current pose,")
        print("goal x: goal position x (unit: m)")
        print("goal y: goal position y (unit: m)")
        print("goal heading: goal orientation (range: -180 ~ 180, unit: deg)")
        print("----------------------------------------------")

        self.goal_position = Point()
        self.goal_heading = 0.0
        self.position = Point()
        self.heading = 0.0 
        self.position_error = Point()
        self.heading_error = 0.0 

        self.angular_speed = 0.3
        self.linear_speed = 0.5

        self.get_key()

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.cmd_vel = Twist()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.get_odom,
            10)
        self.odom_sub

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.position_error.x = self.goal_position.x - self.position.x
        self.position_error.y = self.goal_position.y - self.position.y

        distance = math.sqrt(pow(self.position_error.x, 2) + pow(self.position_error.y, 2))
        goal_direction = math.atan2(self.position_error.y, self.position_error.x)

        if distance > 0.1:
            path_angle = goal_direction - self.heading

            if path_angle < -math.pi:
                path_angle = path_angle + 2 * math.pi
            elif path_angle > math.pi:
                path_angle = path_angle - 2 * math.pi

            self.cmd_vel.angular.z = path_angle
            self.cmd_vel.linear.x = min(self.linear_speed * distance, 0.1)

            if self.cmd_vel.angular.z > 0:
                self.cmd_vel.angular.z = min(self.cmd_vel.angular.z, 1.5)
            else:
                self.cmd_vel.angular.z = max(self.cmd_vel.angular.z, -1.5)

            self.get_logger().info("goal x,y: {:.2f}, {:.2f}, robot x,y: {:.2f}, {:.2f}".format( \
                self.goal_position.x, self.goal_position.y, \
                self.position.x, self.position.y))

            self.cmd_vel_pub.publish(self.cmd_vel)   
        else:
            self.heading_error = self.goal_heading - self.heading

            if self.heading_error < -math.pi:
                self.heading_error = self.heading_error+ 2 * math.pi
            elif self.heading_error > math.pi:
                self.heading_error = self.heading_error- 2 * math.pi

            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.heading_error

            self.get_logger().info("goal heading: {:.2f}, robot heading: {:.2f}".format( \
                self.goal_heading * 180.0 / math.pi, self.heading * 180.0 / math.pi))

            if abs(self.heading_error * 180.0 / math.pi) < 1.0:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)

                self.get_logger().info("goal heading: {:.2f}, robot heading: {:.2f}".format( \
                    self.goal_heading * 180.0 / math.pi, self.heading * 180.0 / math.pi))
                rclpy.shutdown()
                
                
        self.cmd_vel_pub.publish(self.cmd_vel)

    def get_odom(self, msg):   
        self.position = msg.pose.pose.position
        _, _, self.heading = self.transfrom_from_quaternion_to_eular(msg.pose.pose.orientation)
        
    def get_key(self):
        self.goal_position.x = float(input("goal x: "))
        self.goal_position.y = float(input("goal y: "))
        self.goal_heading = float(input("goal heading: "))

        if self.goal_heading >= math.pi:
            self.goal_heading = self.goal_heading % (math.pi * 180.0 / math.pi)
        elif self.goal_heading <= -math.pi:
            self.goal_heading = -(-self.goal_heading % (math.pi * 180.0 / math.pi))
        self.goal_heading = self.goal_heading * math.pi / 180.0

        self.get_logger().info("goal position xy: {:.2f}, {:.2f}, goal heading: {:.2f}".format( \
            self.goal_position.x, self.goal_position.y, self.goal_heading * 180.0 / math.pi))
 
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
    turtlebot3_position_control_pointop_key = Turtlebot3PositionControlPointOpKey()
    try:
        rclpy.spin(turtlebot3_position_control_pointop_key)
    except KeyboardInterrupt:
        turtlebot3_position_control_pointop_key.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally: 
        rclpy.shutdown()
        turtlebot3_position_control_pointop_key.destroy_node()

if __name__ == '__main__':
    main()
