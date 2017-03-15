#!/usr/bin/env python
#
#

import roslib; roslib.load_manifest('turtlebot_panorama')
import rospy
from turtlebot_panorama.srv import *
from sensor_msgs.msg import Image


in_progress = False

def imageCallback(msg):
  global in_progress
  rospy.loginfo("Final Image")
  in_progress = False


if __name__ == '__main__':
  global in_progress
  rospy.init_node('loop_panorama')


  # Setting service
  srv_takepano = rospy.ServiceProxy('/turtlebot_panorama/take_pano',TakePano)
  takepanoReq = TakePanoRequest() 
  takepanoReq.angle = 360
  takepanoReq.snap_interval = 0.5
  takepanoReq.zvel = -0.2

  # Setting final image subscriber
  sub_pano = rospy.Subscriber('/turtlebot_panorama/pano_server/stitch',Image,imageCallback)

  in_progress = False

  iteration = 1
  print str(takepanoReq)

  rospy.loginfo("Initialized")
  while not rospy.is_shutdown():
    rospy.loginfo("Iteration " + str(iteration))
    # send take panorama call 
    takepanoReq.zvel = -takepanoReq.zvel
    srv_takepano(takepanoReq)
    in_progress = True
    # wait until the final image
    while (not rospy.is_shutdown()) and in_progress == True:
      print "is_shutdown" + str(rospy.is_shutdown())
      print "inprogress " + str(in_progress)
      
      rospy.sleep(1.0)
  
    iteration = iteration + 1

  rospy.loginfo("Bye Bye")
#  takepanoReq.angle = -1
#  srv_takepano(takepanoReq)
  

