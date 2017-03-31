/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#ifndef GAZEBO_ROS_TURTLEBOT3_H_
#define GAZEBO_ROS_TURTLEBOT3_H_

#include <ros/ros.h>
#include <ros/time.h>

#include <math.h>
#include <limits.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#define CENTER 0
#define RIGHT  1
#define LEFT   2

class GazeboRosTurtleBot3
{
 public:
  GazeboRosTurtleBot3();
  ~GazeboRosTurtleBot3();
  bool init();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  bool is_debug_;
  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  double direction_vector[3] = {0.0, 0.0, 0.0};

  std_msgs::String turtlebot3_direction_;

  double turtlebot3_linear_velocity_;
  double turtlebot3_angular_velocity_;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
};
#endif // GAZEBO_ROS_TURTLEBOT3_H_
