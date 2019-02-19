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

typedef struct robot
{
  std::array<double,4>  diff_wheels;
  double theta;
}Robot;

class Odometry
{
 public:
  Odometry(){};
  virtual ~Odometry(){};

  nav_msgs::msg::Odometry getOdom(const rclcpp::Time now, const double wheel_radius);
  const geometry_msgs::msg::TransformStamped getOdomTf();
  void updateOdomTf(const rclcpp::Time now, const nav_msgs::msg::Odometry odom);
  void updateImu(const sensor_msgs::msg::Imu::SharedPtr imu);
  void updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state);

 private:
  bool calcOdometry(const rclcpp::Duration duration, const double wheel_radius);

  std::mutex robot_mutex_, tf_mutex_;
  Robot diff_mobile_;

  geometry_msgs::msg::TransformStamped odom_tf_;
  
  std::array<double,3> odom_pose_;
  std::array<double,3> odom_vel_;
};
}

#endif //TURTLEBOT3_ODOMETRY_H