// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#ifndef TURTLEBOT3_NODE__ODOMETRY_HPP_
#define TURTLEBOT3_NODE__ODOMETRY_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


namespace robotis
{
namespace turtlebot3
{
class Odometry
{
public:
  explicit Odometry(
    std::shared_ptr<rclcpp::Node> & nh,
    const double wheels_separation,
    const double wheels_radius);
  virtual ~Odometry() {}

private:
  bool calculate_odometry(const rclcpp::Duration & duration);

  void update_imu(const std::shared_ptr<sensor_msgs::msg::Imu const> & imu);
  void update_joint_state(const std::shared_ptr<sensor_msgs::msg::JointState const> & joint_state);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg);

  void joint_state_and_imu_callback(
    const std::shared_ptr<sensor_msgs::msg::JointState const> & joint_state_msg,
    const std::shared_ptr<sensor_msgs::msg::Imu const> & imu_msg);

  void publish(const rclcpp::Time & now);

  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::shared_ptr<
    message_filters::Subscriber<sensor_msgs::msg::JointState>> msg_ftr_joint_state_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> msg_ftr_imu_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::JointState,
      sensor_msgs::msg::Imu> SyncPolicyJointStateImu;
  typedef message_filters::Synchronizer<SyncPolicyJointStateImu> SynchronizerJointStateImu;

  std::shared_ptr<SynchronizerJointStateImu> joint_state_imu_sync_;

  double wheels_separation_;
  double wheels_radius_;

  std::string frame_id_of_odometry_;
  std::string child_frame_id_of_odometry_;

  bool use_imu_;
  bool publish_tf_;

  std::array<double, 2> diff_joint_positions_;
  double imu_angle_;

  std::array<double, 3> robot_pose_;
  std::array<double, 3> robot_vel_;
};
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__ODOMETRY_HPP_
