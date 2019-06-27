/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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

/* Author: Darby Lim */

#include "odometry.h"
#include <iostream>

using namespace turtlebot3;
constexpr char FRAME_ID_OF_ODOMETRY[] = "odom";
constexpr char CHILD_FRAME_ID_OF_ODOMETRY[] = "base_footprint";

nav_msgs::msg::Odometry Odometry::getOdom(const rclcpp::Time now, const double wheel_radius)
{
  static rclcpp::Time last_time = now;
  rclcpp::Duration duration(now.nanoseconds() - last_time.nanoseconds());    
  
  calcOdometry(duration, wheel_radius);

  auto odom = nav_msgs::msg::Odometry();

  odom.header.frame_id = FRAME_ID_OF_ODOMETRY;
  odom.child_frame_id  = CHILD_FRAME_ID_OF_ODOMETRY;
  odom.header.stamp = now;

  odom.pose.pose.position.x = odom_pose_[0];
  odom.pose.pose.position.y = odom_pose_[1];
  odom.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_pose_[2]);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x  = odom_vel_[0];
  odom.twist.twist.angular.z = odom_vel_[2];

  updateOdomTf(now, odom);
  last_time = now;
  return odom;
}

const geometry_msgs::msg::TransformStamped Odometry::getOdomTf()
{
  std::lock_guard<std::mutex> lock(tf_mutex_);
  return odom_tf_;
}

void Odometry::updateOdomTf(const rclcpp::Time now, const nav_msgs::msg::Odometry odom)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  odom_tf_.transform.translation.x = odom.pose.pose.position.x;
  odom_tf_.transform.translation.y = odom.pose.pose.position.y;
  odom_tf_.transform.translation.z = odom.pose.pose.position.z;
  odom_tf_.transform.rotation      = odom.pose.pose.orientation;

  odom_tf_.header.frame_id = FRAME_ID_OF_ODOMETRY;
  odom_tf_.child_frame_id = CHILD_FRAME_ID_OF_ODOMETRY;
  odom_tf_.header.stamp = now;
}

void Odometry::updateJointState(const sensor_msgs::msg::JointState& joint_state)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);
  static double last_joint_positions[2] = {0.0f, 0.0f};

  diff_mobile_.diff_wheels[0] = joint_state.position[0] - last_joint_positions[0];
  diff_mobile_.diff_wheels[1] = joint_state.position[1] - last_joint_positions[1];

  last_joint_positions[0] = joint_state.position[0];
  last_joint_positions[1] = joint_state.position[1];
}

void Odometry::updateImu(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);
  diff_mobile_.theta = atan2f(imu->orientation.x*imu->orientation.y + imu->orientation.w*imu->orientation.z, 
                       0.5f - imu->orientation.y*imu->orientation.y - imu->orientation.z*imu->orientation.z);
}

bool Odometry::calcOdometry(const rclcpp::Duration duration, const double wheel_radius)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);

  double wheel_l = 0.0f;
  double wheel_r = 0.0f; // rotation value of wheel [rad]

  double delta_s = 0.0f;
  double delta_theta = 0.0f;

  double theta = 0.0f;
  static double last_theta = 0.0f;

  double v = 0.0f;  // v = translational velocity [m/s]
  double w = 0.0f;  // w = rotational velocity [rad/s]
  
  double step_time = duration.seconds();

  if (step_time == 0.0f)
    return false;

  wheel_l = diff_mobile_.diff_wheels[0];
  wheel_r = diff_mobile_.diff_wheels[1];

  if (std::isnan(wheel_l))
    wheel_l = 0.0f;

  if (std::isnan(wheel_r))
    wheel_r = 0.0f;

  delta_s     = wheel_radius * (wheel_r + wheel_l) / 2.0f;
  theta       = diff_mobile_.theta;

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0f));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0f));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel_[0] = v;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = w;

  last_theta = theta;
  return true;
}