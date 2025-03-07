#!/usr/bin/env python
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
# Authors: Junyeong Jeong, Jeonggeun Lim, Gilbert

import math
import sys

from geometry_msgs.msg import Twist
from interactive_markers import InteractiveMarkerServer
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl


class Turtlebot3InteractiveMarker(Node):

    def __init__(self):
        super().__init__('turtlebot3_interactive_marker')

        print('TurtleBot3 Interactive Markers')
        print('----------------------------------------------')
        print('Move red arrows while clicking the arrows')
        print('Rotate with the circular handles along Z-axis')
        print('----------------------------------------------')

        qos = QoSProfile(depth=10)

        self.odom = Odometry()
        self.goal_position = None
        self.goal_orientation = None
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        self.server = InteractiveMarkerServer(self, 'turtlebot3_interactive_marker')

        self.move_marker = InteractiveMarker()
        self.move_marker.header.frame_id = 'odom'
        self.move_marker.name = 'turtlebot3_move_marker'

        move_control = InteractiveMarkerControl()
        move_control.name = 'move_x'
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_control.always_visible = True
        self.move_marker.controls.append(move_control)

        self.server.insert(self.move_marker, feedback_callback=self.processMoveFeedback)

        rotate_marker = InteractiveMarker()
        rotate_marker.header.frame_id = 'base_link'
        rotate_marker.name = 'turtlebot3_rotate_marker'

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = 'rotate_z'
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.x = 0.0
        rotate_control.orientation.y = 1.0
        rotate_control.orientation.z = 0.0
        rotate_marker.controls.append(rotate_control)

        self.server.insert(rotate_marker, feedback_callback=self.processRotateFeedback)

        self.server.applyChanges()

        self.create_timer(0.1, self.publish_cmd_vel)

    def odom_callback(self, msg):
        self.odom = msg

    def processMoveFeedback(self, feedback):
        self.goal_position = feedback.pose.position
        self.goal_orientation = None

        self.update_move_marker_pose()

    def processRotateFeedback(self, feedback):
        self.goal_orientation = feedback.pose.orientation
        self.goal_position = None

    def get_yaw(self):
        q = self.odom.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

    def update_move_marker_pose(self):
        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y
        current_yaw = self.get_yaw()

        new_quat = quaternion_from_euler(0, 0, current_yaw)

        self.move_marker.pose.position.x = current_x
        self.move_marker.pose.position.y = current_y
        self.move_marker.pose.orientation.x = new_quat[0]
        self.move_marker.pose.orientation.y = new_quat[1]
        self.move_marker.pose.orientation.z = new_quat[2]
        self.move_marker.pose.orientation.w = new_quat[3]

        self.server.insert(self.move_marker)
        self.server.applyChanges()

    def publish_cmd_vel(self):
        twist = Twist()

        if self.goal_position is not None:
            current_x = self.odom.pose.pose.position.x
            current_y = self.odom.pose.pose.position.y
            current_yaw = self.get_yaw()

            goal_x = self.goal_position.x
            goal_y = self.goal_position.y

            dx = goal_x - current_x
            dy = goal_y - current_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.01:
                self.goal_position = None
            else:
                forward_speed = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
                lateral_speed = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)

                twist.linear.x = max(-0.1, min(0.1, forward_speed))
                twist.linear.y = max(-0.1, min(0.1, lateral_speed))

        elif self.goal_orientation is not None:
            current_yaw = self.get_yaw()

            goal_q = self.goal_orientation
            goal_euler = euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])
            target_yaw = goal_euler[2]

            yaw_diff = math.atan2(
                math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw)
            )

            if abs(yaw_diff) < 0.01:
                self.goal_orientation = None
            else:
                twist.angular.z = max(-0.5, min(0.5, yaw_diff))

        self.cmd_vel_pub.publish(twist)
        self.update_move_marker_pose()


def main(args=None):
    rclpy.init(args=sys.argv)
    turtlebot3_interactive_marker = Turtlebot3InteractiveMarker()
    rclpy.spin(turtlebot3_interactive_marker)


if __name__ == '__main__':
    main()
