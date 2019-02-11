/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
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

#ifndef TURTLEBOT3_ODOMETRY_H
#define TURTLEBOT3_ODOMETRY_H

#include <memory>
#include <array>
#include <mutex>

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace turtlebot3
{
class Odometry
{
 public:
  Odometry()
    :last_theta_(0.0f)
  {
    joint_state_ = std::make_shared<sensor_msgs::msg::JointState>();
    imu_ = std::make_shared<sensor_msgs::msg::Imu>();
  };
  virtual ~Odometry(){};

  nav_msgs::msg::Odometry::SharedPtr getOdom(rclcpp::Time now, double wheel_radius);
  const geometry_msgs::msg::TransformStamped getOdomTf();
  void updateOdomTf(rclcpp::Time now, const nav_msgs::msg::Odometry::SharedPtr odom);
  void updateImu(const sensor_msgs::msg::Imu::SharedPtr imu);
  void updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state);

 private:
  bool calcOdometry(rclcpp::Duration duration, double wheel_radius);

  std::mutex mutex_;
  std::shared_ptr<sensor_msgs::msg::JointState> joint_state_;
  std::shared_ptr<sensor_msgs::msg::Imu> imu_;
  geometry_msgs::msg::TransformStamped odom_tf_;

  double last_theta_;
  rclcpp::Time last_time_;

  std::array<double,3> odom_pose_;
  std::array<double,3> odom_vel_;  
};
}

#endif //TURTLEBOT3_ODOMETRY_H