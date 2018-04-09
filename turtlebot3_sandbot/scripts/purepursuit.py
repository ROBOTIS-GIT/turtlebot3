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
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, sin
from tf.transformations import euler_from_quaternion
import numpy as np
import sys

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

increment = 5

arr_path_B = []
with open(sys.argv[1], "r") as file_path_B:
    for idx, line in enumerate(file_path_B):
        str = line.split()
        if not (len(str)==0):
            #print(str)
            arr_path_B.append([(float)(str[1]), (float)(str[0])])
        else:
            break
# now we imported arr_path_B from file to arr_path_B

class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_sandbot', anonymous=False, disable_signals=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)

        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'
        self.isFirst = True
        print("isFirst initialized")
        self.offset_x=0
        self.offset_y=0
        self.offset_rot=0

        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")
        
        (position, rotation) = self.get_odom()
        if self.isFirst:
            self.offset_x=position.x
            self.offset_y=position.y
            self.offset_rot=rotation
            self.isFirst=False
            print("offset initialized")
        (position, rotation) = self.get_odom()
        print("x, y, rotation", position.x, position.y, np.rad2deg(rotation))
        
        lin_vel=0.1
        # (goal_x, goal_y, goal_z) = self.getkey()

        # go through path array
        ind = 1
        length = len(arr_path_B)
        init_goal = arr_path_B[0]
        goal_x = arr_path_B[ind][0]-init_goal[0]
        goal_y = arr_path_B[ind][1]-init_goal[1]

        thres1=np.deg2rad(30)
        thres2=np.deg2rad(15)
        ang_vel_1=0.4
        ang_vel_2=0.15

        while ind < length:
            # if goal_z > 180 or goal_z < -180:
            #     print("you input worng z range.")
            #     self.shutdown()
            # goal_z = np.deg2rad(goal_z)
            # (position,rotation) = self.get_odom()
            
            goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
            distance = goal_distance
            while distance > 0.1:
                try:
                    print ("distance= ", distance)
                    
                    x_start= position.x
                    y_start= position.y

                    print("goal", goal_x, goal_y, "position", x_start, y_start)
                    # alpha=atan2(goal_x-x_start, goal_y-y_start)-rotation
                    alpha=atan2(goal_y-y_start, goal_x-x_start)-rotation

                    #Alpha normalization
                    if alpha>pi:
                        alpha=alpha-2*pi
                    elif alpha<-pi:
                        alpha=alpha+2*pi

                    print("alpha", np.rad2deg(alpha),"rotation", np.rad2deg(rotation))

                    if abs(alpha)> thres1: #abs?
                        if alpha>0 or alpha<-pi:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=ang_vel_1
                          
                        else:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=-ang_vel_1
                         
                    elif abs(alpha)>thres2:
                        if alpha>0 or alpha<-pi:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=ang_vel_2
                         
                        else:
                            move_cmd.linear.x=0
                            move_cmd.angular.z=-ang_vel_2
                         
                    else:
                        x=distance*sin(alpha)
                        curv=2*x/pow(distance,2)
                        move_cmd.linear.x=lin_vel
                        move_cmd.angular.z=curv*lin_vel
                        
                  
                    self.cmd_vel.publish(move_cmd)

                    (position, rotation) = self.get_odom()
                    distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

                    r.sleep()
                except KeyboardInterrupt:
                    rospy.signal_shutdown("KeboardInterrupt")
                    break
            
            print("Now at Waypoint No.", ind)
            ind = ind + increment
            goal_x = arr_path_B[ind][0]-init_goal[0]
            goal_y = arr_path_B[ind][1]-init_goal[1]


            if rospy.is_shutdown():
                break

        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        pnt=Point(*trans)
        pnt.x=pnt.x-self.offset_x
        pnt.y=pnt.y-self.offset_y

        # if rotation[2]-self.offset_rot < -pi:
        #     return(pnt, rotation[2]-self.offset_rot+2*pi)
        # return (pnt, rotation[2]-self.offset_rot)
        # return (pnt, rotation[2]-self.offset_rot)
        return (pnt, rotation[2])
        # return (Point(*trans), rotation[2])

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            #print(msg)
            GotoPoint()

    except:
        rospy.loginfo("shutdown program.")
