#!/usr/bin/env python
#################################################################################
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
#################################################################################

# Authors: Gilbert #

import rospy
from geometry_msgs.msg import Twist, Pose
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import tf
from tf.transformations import euler_from_quaternion
import copy

def processFeedback(feedback):
    _,_,yaw = euler_from_quaternion((feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w))

    twist = Twist()
    twist.angular.z = 2.2 * yaw
    twist.linear.x = 1.0 * feedback.pose.position.x

    vel_pub.publish(twist)

    server.setPose("turtlebot3_marker", Pose())
    server.applyChanges()

if __name__ == "__main__":
    rospy.init_node("turtlebot3_interactive_marker_server")
    server = InteractiveMarkerServer("turtlebot3_marker_server")
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "turtlebot3_marker"


    control = InteractiveMarkerControl()
    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    int_marker.controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"

    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    int_marker.controls.append(copy.deepcopy(control))

    server.insert(int_marker, processFeedback)

    server.applyChanges()

    rospy.spin()