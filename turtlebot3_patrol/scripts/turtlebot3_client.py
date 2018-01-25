#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
import turtlebot3_patrol.msg
import sys

msg = """
patrol your Turtlebot3!
-----------------------
mode : s - Patrol to Square
       t - Patrol to Triangle
       c - Patrol to Circle

area : Square, Triangle mode - length of side (m)
       Circle mode - radius (m)

count - patrol count
"""

def getkey():
    mode, area, count= raw_input("| mode | area | count |\n").split()
    mode, area ,count= [str(mode), float(area), int(count)]

    return mode, area, count

def turtlebot3_client():
    client = actionlib.SimpleActionClient('turtlebot3', turtlebot3_patrol.msg.turtlebot3Action)
    mode, area, count = getkey()
    if mode == 's':
        mode = 1
    elif mode == 't':
        mode = 2
    elif mode == 'c':
        mode = 3
    else:
        print("you select wrong mode")
    print("wait for server")
    client.wait_for_server()
    goal = turtlebot3_patrol.msg.turtlebot3Goal()
    goal.goal.x = mode
    goal.goal.y = area
    goal.goal.z = count
    client.send_goal(goal)
    print("send to goal")
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        print (msg)
        while(1):
            rospy.init_node('turtlebot3_client')
            result = turtlebot3_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)


