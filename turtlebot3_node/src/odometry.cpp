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

#include "turtlebot3_node/odometry.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::Odometry;
using namespace std::chrono_literals;

Odometry::Odometry(
  std::shared_ptr<rclcpp::Node> & nh,
  const double wheels_separation,
  const double wheels_radius)
: nh_(nh),
  wheels_separation_(wheels_separation),
  wheels_radius_(wheels_radius),
  use_imu_(false),
  publish_tf_(false),
  imu_angle_(0.0f)
{
  RCLCPP_INFO(nh_->get_logger(), "Init Odometry");

  nh_->declare_parameter<std::string>("odometry.frame_id");
  nh_->declare_parameter<std::string>("odometry.child_frame_id");

  nh_->declare_parameter<bool>("odometry.use_imu");
  nh_->declare_parameter<bool>("odometry.publish_tf");

  nh_->get_parameter_or<bool>(
    "odometry.use_imu",
    use_imu_,
    false);

  nh_->get_parameter_or<bool>(
    "odometry.publish_tf",
    publish_tf_,
    false);

  nh_->get_parameter_or<std::string>(
    "odometry.frame_id",
    frame_id_of_odometry_,
    std::string("odom"));

  nh_->get_parameter_or<std::string>(
    "odometry.child_frame_id",
    child_frame_id_of_odometry_,
    std::string("base_footprint"));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);

  if (use_imu_) {
    uint32_t queue_size = 10;
    joint_state_imu_sync_ = std::make_shared<SynchronizerJointStateImu>(queue_size);

    msg_ftr_joint_state_sub_ =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::JointState>>(
      nh_,
      "joint_states");

    msg_ftr_imu_sub_ =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
      nh_,
      "imu");

    // connect message filters to synchronizer
    joint_state_imu_sync_->connectInput(*msg_ftr_joint_state_sub_, *msg_ftr_imu_sub_);

    joint_state_imu_sync_->setInterMessageLowerBound(
      0,
      rclcpp::Duration(75ms));

    joint_state_imu_sync_->setInterMessageLowerBound(
      1,
      rclcpp::Duration(15ms));

    joint_state_imu_sync_->registerCallback(
      std::bind(
        &Odometry::joint_state_and_imu_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  } else {
    joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      qos,
      std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));
  }
}

void Odometry::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
{
  static rclcpp::Time last_time = joint_state_msg->header.stamp;
  rclcpp::Duration duration(rclcpp::Duration::from_nanoseconds(
      joint_state_msg->header.stamp.nanosec - last_time.nanoseconds()));

  update_joint_state(joint_state_msg);
  calculate_odometry(duration);
  publish(joint_state_msg->header.stamp);

  last_time = joint_state_msg->header.stamp;
}

void Odometry::joint_state_and_imu_callback(
  const std::shared_ptr<sensor_msgs::msg::JointState const> & joint_state_msg,
  const std::shared_ptr<sensor_msgs::msg::Imu const> & imu_msg)
{
  RCLCPP_DEBUG(
    nh_->get_logger(),
    "[joint_state_msg_] nanosec : %d [imu_msg] nanosec : %d",
    joint_state_msg->header.stamp.nanosec,
    imu_msg->header.stamp.nanosec);

  static rclcpp::Time last_time = joint_state_msg->header.stamp;
  rclcpp::Duration duration(rclcpp::Duration::from_nanoseconds(
      joint_state_msg->header.stamp.nanosec - last_time.nanoseconds()));

  update_joint_state(joint_state_msg);
  update_imu(imu_msg);
  calculate_odometry(duration);
  publish(joint_state_msg->header.stamp);

  last_time = joint_state_msg->header.stamp;
}

void Odometry::publish(const rclcpp::Time & now)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = frame_id_of_odometry_;
  odom_msg->child_frame_id = child_frame_id_of_odometry_;
  odom_msg->header.stamp = now;

  odom_msg->pose.pose.position.x = robot_pose_[0];
  odom_msg->pose.pose.position.y = robot_pose_[1];
  odom_msg->pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose_[2]);

  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = robot_vel_[0];
  odom_msg->twist.twist.angular.z = robot_vel_[2];

  // TODO(Will Son): Find more accurate covariance.
  // odom_msg->pose.covariance[0] = 0.05;
  // odom_msg->pose.covariance[7] = 0.05;
  // odom_msg->pose.covariance[14] = 1.0e-9;
  // odom_msg->pose.covariance[21] = 1.0e-9;
  // odom_msg->pose.covariance[28] = 1.0e-9;
  // odom_msg->pose.covariance[35] = 0.0872665;

  // odom_msg->twist.covariance[0] = 0.001;
  // odom_msg->twist.covariance[7] = 1.0e-9;
  // odom_msg->twist.covariance[14] = 1.0e-9;
  // odom_msg->twist.covariance[21] = 1.0e-9;
  // odom_msg->twist.covariance[28] = 1.0e-9;
  // odom_msg->twist.covariance[35] = 0.001;

  geometry_msgs::msg::TransformStamped odom_tf;

  odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
  odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

  odom_tf.header.frame_id = frame_id_of_odometry_;
  odom_tf.child_frame_id = child_frame_id_of_odometry_;
  odom_tf.header.stamp = now;

  odom_pub_->publish(std::move(odom_msg));

  if (publish_tf_) {
    tf_broadcaster_->sendTransform(odom_tf);
  }
}

void Odometry::update_joint_state(
  const std::shared_ptr<sensor_msgs::msg::JointState const> & joint_state)
{
  static std::array<double, 2> last_joint_positions = {0.0f, 0.0f};

  diff_joint_positions_[0] = joint_state->position[0] - last_joint_positions[0];
  diff_joint_positions_[1] = joint_state->position[1] - last_joint_positions[1];

  last_joint_positions[0] = joint_state->position[0];
  last_joint_positions[1] = joint_state->position[1];
}

void Odometry::update_imu(const std::shared_ptr<sensor_msgs::msg::Imu const> & imu)
{
  imu_angle_ = atan2f(
    imu->orientation.x * imu->orientation.y + imu->orientation.w * imu->orientation.z,
    0.5f - imu->orientation.y * imu->orientation.y - imu->orientation.z * imu->orientation.z);
}

bool Odometry::calculate_odometry(const rclcpp::Duration & duration)
{
  // rotation value of wheel [rad]
  double wheel_l = diff_joint_positions_[0];
  double wheel_r = diff_joint_positions_[1];

  double delta_s = 0.0;
  double delta_theta = 0.0;

  double theta = 0.0;
  static double last_theta = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]
  double v = 0.0;
  double w = 0.0;

  double step_time = duration.seconds();

  if (step_time == 0.0) {
    return false;
  }

  if (std::isnan(wheel_l)) {
    wheel_l = 0.0;
  }

  if (std::isnan(wheel_r)) {
    wheel_r = 0.0;
  }

  delta_s = wheels_radius_ * (wheel_r + wheel_l) / 2.0;

  if (use_imu_) {
    theta = imu_angle_;
    delta_theta = theta - last_theta;
  } else {
    theta = wheels_radius_ * (wheel_r - wheel_l) / wheels_separation_;
    delta_theta = theta;
  }

  // compute odometric pose
  robot_pose_[0] += delta_s * cos(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[1] += delta_s * sin(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[2] += delta_theta;

  RCLCPP_DEBUG(nh_->get_logger(), "x : %f, y : %f", robot_pose_[0], robot_pose_[1]);

  // compute odometric instantaneouse velocity
  v = delta_s / step_time;
  w = delta_theta / step_time;

  robot_vel_[0] = v;
  robot_vel_[1] = 0.0;
  robot_vel_[2] = w;

  last_theta = theta;
  return true;
}
