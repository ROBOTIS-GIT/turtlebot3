#!/usr/bin/env python

import rospy
import sys, termios, tty, select
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

msg = """
Control Your Turtlebot3!
---------------------------
x : position x
y : position y
z : orientation z
"""
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rotation_q = msg.pose.pose.orientation

    (roll, pitch, theta) = euler_from_quaternion([rotation_q.x, rotation_q.y, rotation_q.z, rotation_q.w])

rospy.init_node("amcl")

rospy.loginfo('Turtlebot3 amcl node initialized')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber("/odom", Odometry, newOdom)

twist = Twist()

r = rospy.Rate(10)

goal = Point()
goal.x = 0
goal.y = 0

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1:
        twist.linear.x = 0.0
        twist.angular.z = 0.3

    else:
        twist.linear.x = 0.1
        twist.angular.z = 0.0

    pub.publish(twist)
    r.sleep()
