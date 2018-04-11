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

waypoint_increment = 1

def get_path(word):
    arr_path=[]
    dir_1= "/home/bkjung/catkin_ws/src/turtlebot3/turtlebot3_sandbot/data_path/path_"
    dir_2=".txt"
    i=0.0
    for letter in word:
        if letter==' ':
            pass
            #if spacing is included
        else:
            with open(dir_1+letter.capitalize()+dir_2,"r") as file_path:
                for idx, line in enumerate(file_path):
                    _str = line.split()
                    if not (len(_str)==0):
                        arr_path.append([(float)(_str[1]), (float)(_str[0])+i])
                    else:
                        pass
        i=i+1
        
    return arr_path

# arr_path_B = []
# with open(sys.argv[1], "r") as file_path_B:
#     for idx, line in enumerate(file_path_B):
#         str = line.split()
#         if not (len(str)==0):
#             #print(str)
#             arr_path_B.append([(float)(str[1]), (float)(str[0])])
#         else:
#             break
# now we imported arr_path_B from file to arr_path_B

class GotoPoint():
    def __init__(self, arr_path):
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
            print("base_footprint")
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
                print("base_link")
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
        
        lin_vel=0.08
        # (goal_x, goal_y, goal_z) = self.getkey()

        # go through path array
        waypoint_index = 0
        waypoints_length = len(arr_path)
        waypoints=[]
        for idx in range(waypoints_length-1):
            waypoints.append([arr_path[idx+1][0]-arr_path[0][0], arr_path[idx+1][1]-arr_path[0][1]])

        print("size of waypoints = ", len(waypoints), len(waypoints[0]))

        thres1=np.deg2rad(30)
        thres2=np.deg2rad(15)
        ang_vel_1=0.3
        ang_vel_2=0.1

        while waypoint_index < waypoints_length:
            waypoint = [waypoints[waypoint_index][0], waypoints[waypoint_index][1]]
            # if goal_z > 180 or goal_z < -180:
            #     print("you input worng z range.")
            #     self.shutdown()
            # goal_z = np.deg2rad(goal_z)
            # (position,rotation) = self.get_odom()
            
            goal_distance = sqrt(pow(waypoint[0] - position.x, 2) + pow(waypoint[1] - position.y, 2))
            distance = goal_distance
            
            while distance > 0.1:
                try:
                    print ("distance= ", '%.3f' % distance)
                    
                    print("goal_position", '%.3f' % waypoint[0], '%.3f' % waypoint[1], "current_position", '%.3f' % position.x, '%.3f' % position.y)
                    # alpha=atan2(goal_x-x_start, goal_y-y_start)-rotation
                    alpha=atan2(waypoint[1]-position.y, waypoint[0]-position.x)-rotation

                    #Alpha normalization
                    if alpha>pi:
                        alpha=alpha-2*pi
                    elif alpha<-pi:
                        alpha=alpha+2*pi

                    print("goal_angle", '%.3f' % np.rad2deg(alpha),"current_angle", '%.3f' % np.rad2deg(rotation))

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
                    distance = sqrt(pow((waypoint[0] - position.x), 2) + pow((waypoint[1] - position.y), 2))

                    r.sleep()
                except KeyboardInterrupt:
                    print("Got KeyboardInterrupt")
                    rospy.signal_shutdown("KeboardInterrupt")                    
                    break
            
            print("Now at Waypoint No.", waypoint_index)
            waypoint_index = waypoint_index + waypoint_increment

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
        #print("point:", pnt.x, pnt.y)
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
        word=raw_input("Type a word:")
        print("word:", word)
        path=get_path(word)
        print("path loaded")
        GotoPoint(path)

        print("End of Main Function")

    except Exception as e:
        print(e)
        rospy.loginfo("shutdown program.")
