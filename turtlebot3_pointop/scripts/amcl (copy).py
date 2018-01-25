#!/usr/bin/env python

import rospy
import sys, termios, tty, select
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

msg = """
Control Your Turtlebot3!
---------------------------
x : position x
y : position y
z : orientation z
"""
def callback(data):

    quaternion = quaternion_from_euler(0, 0, data)
    posestamped.pose.orientation.x = quaternion[0]
    posestamped.pose.orientation.y = quaternion[1]
    posestamped.pose.orientation.z = quaternion[2]
    posestamped.pose.orientation.w = quaternion[3]

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.loginfo('Turtlebot3 amcl node initialized')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber("odom", Odometry)
    rospy.init_node('Amcl', anonymous=True)
    posestamped=PoseStamped()

    try:

        while(1):
            key = getKey()
            if (key == '\x03'):
                    break
            posestamped.header.stamp = rospy.Time.now()
            posestamped.header.seq += 1
            posestamped.header.frame_id = 'odom'
            x, y, z = raw_input("| x | y | z |\n").split()
            x, y, z = [float(x), float(y), float(z)]

            posestamped.pose.position.x = x/100
            posestamped.pose.position.y = y/100
            posestamped.pose.orientation.z = z
            callback(z)
            pub.publish(posestamped)
    except:
        print e

    finally:
        print msg

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)