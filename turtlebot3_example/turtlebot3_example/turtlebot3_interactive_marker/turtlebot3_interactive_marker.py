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
# Authors: Jeonggeun Lim, Gilbert #

import sys
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
import rclpy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class Turtlebot3InteractiveMarker(Node):

    def __init__(self):
        super().__init__('turtlebot3_interactive_marker')

        print("TurtleBot3 Interactive Markers")
        print("----------------------------------------------")
        print("Move red arrows while clicking the arrows")
        print("Rotate with the circular handles along Z-axis")
        print("----------------------------------------------")

        qos = QoSProfile(depth=10)

        self.odom = Odometry()
        self.goal_position = None  # 목표 위치 저장
        self.goal_orientation = None  # 목표 방향 저장
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.server = InteractiveMarkerServer(self, 'turtlebot3_interactive_marker')

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'odom'  # odom 기준
        int_marker.name = 'turtlebot3_marker'

        # 이동 컨트롤
        move_control = InteractiveMarkerControl()
        move_control.name = 'move_x'
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(move_control)

        # 회전 컨트롤
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = 'rotate_z'
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.x = 0.0
        rotate_control.orientation.y = 1.0
        rotate_control.orientation.z = 0.0
        int_marker.controls.append(rotate_control)

        self.server.insert(int_marker, feedback_callback=self.processFeedback)
        self.server.applyChanges()

        # 이동을 주기적으로 업데이트하는 타이머 추가
        self.create_timer(0.1, self.publish_cmd_vel)

    def odom_callback(self, msg):
        """ 현재 로봇 위치를 업데이트 """
        self.odom = msg

    def processFeedback(self, feedback):
        """ 사용자가 마커를 조작하면 목표 위치를 업데이트 """
        self.goal_position = feedback.pose.position
        self.goal_orientation = feedback.pose.orientation

    def publish_cmd_vel(self):

        """ 로봇이 목표 위치로 이동하도록 cmd_vel을 지속적으로 업데이트 """
        if self.goal_position is None or self.goal_orientation is None:
            return  # 목표 위치가 없으면 실행하지 않음

        # 현재 위치
        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y

        # 목표 위치
        goal_x = self.goal_position.x
        goal_y = self.goal_position.y

        # X축 방향으로 이동하는 거리 계산
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx**2 + dy**2)

        self.get_logger().info("goal_position.x: {:.3f}, robot_x: {:.3f}, distance: {:.3f}". \
            format(goal_x, self.odom.pose.pose.position.x, distance))

        # 목표까지 너무 가까우면 정지
        if distance < 0.01:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return

        # 이동 방향을 고려하여 속도 결정 (뒤로 이동도 가능하도록 수정)
        twist = Twist()
        twist.linear.x = max(-0.1, min(0.1, dx))  # dx 방향으로 이동
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=sys.argv)
    turtlebot3_interactive_marker = Turtlebot3InteractiveMarker()
    rclpy.spin(turtlebot3_interactive_marker)


if __name__ == '__main__':
    main()