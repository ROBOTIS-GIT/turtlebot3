#!/usr/bin/env python
#################################################################################
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
#################################################################################

# Authors: Jeonggeun Lim, Gilbert #


import sys
from rclpy.node import Node

from interactive_markers import InteractiveMarkerServer
import rclpy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl

from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Turtlebot3InteractiveMarker(Node):

    def __init__(self):
        super().__init__('turtlebot3_interactive_marker')

        print("TurtleBot3 Interactive Markers")
        print("----------------------------------------------")
        print("Move red arrows while click the arrows")
        print("----------------------------------------------")

        qos = QoSProfile(depth=10)

        self.odom = Odometry()
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.get_odom,
            10)
        self.odom_sub

    def get_odom(self, odom):
        self.odom = odom

    def processFeedback(self, feedback):
        goal_position = feedback.pose.position

        twist = Twist()
        twist.linear.x = goal_position.x - self.odom.pose.pose.position.x

        if twist.linear.x > 0.1:
            twist.linear.x = 0.1
        elif twist.linear.x < -0.1:
            twist.linear.x = -0.1

        self.get_logger().info("goal_position.x: {:.3f}, robot_x: {:.3f}". \
            format(goal_position.x, self.odom.pose.pose.position.x))
        self.cmd_vel_pub.publish(twist)

        
def main(args=None):
    rclpy.init(args=sys.argv)
    turtlebot3_interactive_marker = Turtlebot3InteractiveMarker()
   
    qos = QoSProfile(depth=10)
    turtlebot3_interactive_marker.cmd_vel_pub = turtlebot3_interactive_marker.create_publisher(Twist, 'cmd_vel', qos)

    server = InteractiveMarkerServer(turtlebot3_interactive_marker, 'turtlebot3_interactive_marker')

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'base_link'
    int_marker.name = 'turtlebot3_marker'

    rotate_control = InteractiveMarkerControl()
    rotate_control.name = 'move_x'
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(rotate_control)

    server.insert(int_marker, feedback_callback=turtlebot3_interactive_marker.processFeedback)
    server.applyChanges()

    rclpy.spin(turtlebot3_interactive_marker)
    server.shutdown()

if __name__ == '__main__':
    main()
