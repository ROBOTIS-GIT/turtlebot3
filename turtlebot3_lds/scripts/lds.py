#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('turtlebot3_LDS')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    scan = LaserScan()
    r = rospy.Rate(10)
    LIDAR_ERR = 0.05

    while not rospy.is_shutdown():
        scan = rospy.wait_for_message("/scan", LaserScan)
        twist = Twist()
        scan_filter = []
        twist.linear.x = 0.5
        twist.angular.z = 0.0

        for i in range(360):
            if i <= 15 or i > 335:
                if scan.ranges[i] >= LIDAR_ERR:
                    scan_filter.append(scan.ranges[i])

        if min(scan_filter) < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.05
            twist.angular.z = 0.0

        print(min(scan_filter))
        cmd_pub.publish(twist)
        r.sleep()

