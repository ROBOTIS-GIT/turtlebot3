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

/* Authors: Yoonseok Pyo */

#ifndef TURTLEBOT3_FAKE_H_
#define TURTLEBOT3_FAKE_H_

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>

#include "turtlebot3_fake.h"

#define WHEEL_RADIUS                    0.033     // meter

#define LEFT                            0
#define RIGHT                           1

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_STEP                   0.01   // m/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TORQUE_ENABLE                   1       // Value for enabling the torque of motor
#define TORQUE_DISABLE                  0       // Value for disabling the torque of motor

class Turtlebot3Fake
{
 public:
  Turtlebot3Fake();
  ~Turtlebot3Fake();
  bool init();
  bool update();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  // (TODO)

  // ROS Time
  ros::Time last_cmd_vel_time_;
  ros::Time prev_update_time_;

  // ROS Topic Publishers
  ros::Publisher joint_states_pub_;
  ros::Publisher odom_pub_;

  // ROS Topic Subscribers
  ros::Subscriber cmd_vel_sub_;

  sensor_msgs::JointState joint_states_;
  nav_msgs::Odometry odom_;
  tf::TransformBroadcaster tf_broadcaster_;

  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;
  double cmd_vel_timeout_;

  float  odom_pose_[3];
  float  odom_vel_[3];
  double pose_cov_[36];

  std::string joint_states_name_[2];

  double last_position_[2];
  double last_velocity_[2];

  double wheel_seperation_;
  double turning_radius_;
  double robot_radius_;

  // Function prototypes
  void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
  bool updateOdometry(ros::Duration diff_time);
  void updateJoint(void);
  void updateTF(geometry_msgs::TransformStamped& odom_tf);
};

#endif // TURTLEBOT3_FAKE_H_
