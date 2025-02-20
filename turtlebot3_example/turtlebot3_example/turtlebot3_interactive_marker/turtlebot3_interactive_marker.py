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
import math
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
import rclpy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler


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
        self.goal_position = None  # 이동 목표 위치
        self.goal_orientation = None  # 회전 목표 방향
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.server = InteractiveMarkerServer(self, 'turtlebot3_interactive_marker')

        # 🎯 이동 컨트롤 마커 추가 (Move Control)
        self.move_marker = InteractiveMarker()
        self.move_marker.header.frame_id = 'odom'  # odom 기준으로 이동
        self.move_marker.name = 'turtlebot3_move_marker'

        move_control = InteractiveMarkerControl()
        move_control.name = 'move_x'
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.move_marker.controls.append(move_control)

        self.server.insert(self.move_marker, feedback_callback=self.processMoveFeedback)

        # 🎯 회전 컨트롤 마커 추가 (Rotate Control)
        rotate_marker = InteractiveMarker()
        rotate_marker.header.frame_id = 'base_link'  # odom 기준으로 회전
        rotate_marker.name = 'turtlebot3_rotate_marker'

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = 'rotate_z'
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS  # 회전 전용으로 변경
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.x = 0.0
        rotate_control.orientation.y = 1.0
        rotate_control.orientation.z = 0.0
        rotate_marker.controls.append(rotate_control)

        self.server.insert(rotate_marker, feedback_callback=self.processRotateFeedback)

        self.server.applyChanges()

        # 🎯 이동 & 회전 각각의 컨트롤을 주기적으로 업데이트
        self.create_timer(0.1, self.publish_cmd_vel)

    def odom_callback(self, msg):
        """ 현재 로봇 위치를 업데이트 """
        self.odom = msg

    def processMoveFeedback(self, feedback):
        """ 🔹 이동 마커 조작 시 목표 위치 업데이트 """
        self.goal_position = feedback.pose.position
        self.goal_orientation = None  # 이동 시 회전 목표는 리셋

    def processRotateFeedback(self, feedback):
        """ 🔹 회전 마커 조작 시 목표 방향 업데이트 (선속도는 무시) """
        self.goal_orientation = feedback.pose.orientation
        self.goal_position = None  # 회전 시 이동 목표는 리셋

        self.update_move_marker_pose()

    def get_yaw(self):
        """ 로봇의 현재 Yaw (회전 각도) 가져오기 """
        q = self.odom.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]  # Yaw 값 반환

    def update_move_marker_pose(self):
        """ 🎯 move_marker의 위치를 회전에 맞게 업데이트 """
        # 현재 로봇 위치를 가져옴
        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y
        current_yaw = self.get_yaw()

        # 현재 Yaw 값을 quaternion으로 변환
        new_quat = quaternion_from_euler(0, 0, current_yaw)

        # move_marker의 pose 업데이트
        self.move_marker.pose.position.x = current_x
        self.move_marker.pose.position.y = current_y
        self.move_marker.pose.orientation.x = new_quat[0]
        self.move_marker.pose.orientation.y = new_quat[1]
        self.move_marker.pose.orientation.z = new_quat[2]
        self.move_marker.pose.orientation.w = new_quat[3]

        self.server.insert(self.move_marker)
        self.server.applyChanges()

    def publish_cmd_vel(self):
        """ 로봇이 목표 위치 or 목표 방향으로 이동하도록 cmd_vel을 지속적으로 업데이트 """
        twist = Twist()

        # 🎯 1. 이동 컨트롤 처리 (회전 목표가 없을 때만)
        if self.goal_position is not None:
            current_x = self.odom.pose.pose.position.x
            current_y = self.odom.pose.pose.position.y
            current_yaw = self.get_yaw()

            goal_x = self.goal_position.x
            goal_y = self.goal_position.y

            # 목표까지의 거리 계산
            dx = goal_x - current_x
            dy = goal_y - current_y
            distance = math.sqrt(dx**2 + dy**2)

            # 목표에 가까우면 정지
            if distance < 0.01:
                self.goal_position = None  # 목표 완료 후 초기화
            else:
                # 🔹 현재 회전 각도를 반영하여 로봇이 목표 방향을 향하도록 변환
                forward_speed = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
                lateral_speed = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)

                twist.linear.x = max(-0.1, min(0.1, forward_speed))  # 전진 속도
                twist.linear.y = max(-0.1, min(0.1, lateral_speed))  # 좌우 속도 반영

        # 🎯 2. 회전 컨트롤 처리 (이동 목표가 없을 때만)
        elif self.goal_orientation is not None:
            # 현재 Yaw 각도 계산
            current_yaw = self.get_yaw()

            # 목표 방향에서 Yaw 각도 추출
            goal_q = self.goal_orientation
            goal_euler = euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])
            target_yaw = goal_euler[2]  # 목표 Yaw 값

            # Yaw 차이 계산 (음수면 시계 방향, 양수면 반시계 방향)
            yaw_diff = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))

            # 목표에 가까우면 멈춤
            if abs(yaw_diff) < 0.01:
                self.goal_orientation = None  # 목표 완료 후 초기화
            else:
                twist.angular.z = max(-0.5, min(0.5, yaw_diff))  # 각속도 조정

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=sys.argv)
    turtlebot3_interactive_marker = Turtlebot3InteractiveMarker()
    rclpy.spin(turtlebot3_interactive_marker)


if __name__ == '__main__':
    main()
