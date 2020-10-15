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

#include "turtlebot3_node/sensors/imu.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::Imu;

Imu::Imu(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & imu_topic_name,
  const std::string & mag_topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  imu_pub_ = nh->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, this->qos_);
  mag_pub_ = nh->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create imu publisher");
}

void Imu::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

  imu_msg->header.frame_id = this->frame_id_;
  imu_msg->header.stamp = now;

  imu_msg->orientation.w = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_w.addr,
    extern_control_table.imu_orientation_w.length);

  imu_msg->orientation.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_x.addr,
    extern_control_table.imu_orientation_x.length);

  imu_msg->orientation.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_y.addr,
    extern_control_table.imu_orientation_y.length);

  imu_msg->orientation.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_z.addr,
    extern_control_table.imu_orientation_z.length);

  imu_msg->angular_velocity.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_x.addr,
    extern_control_table.imu_angular_velocity_x.length);

  imu_msg->angular_velocity.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_y.addr,
    extern_control_table.imu_angular_velocity_y.length);

  imu_msg->angular_velocity.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_z.addr,
    extern_control_table.imu_angular_velocity_z.length);

  imu_msg->linear_acceleration.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_x.addr,
    extern_control_table.imu_linear_acceleration_x.length);

  imu_msg->linear_acceleration.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_y.addr,
    extern_control_table.imu_linear_acceleration_y.length);

  imu_msg->linear_acceleration.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_z.addr,
    extern_control_table.imu_linear_acceleration_z.length);

  auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();

  mag_msg->header.frame_id = this->frame_id_;
  mag_msg->header.stamp = now;

  mag_msg->magnetic_field.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_x.addr,
    extern_control_table.imu_magnetic_x.length);

  mag_msg->magnetic_field.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_y.addr,
    extern_control_table.imu_magnetic_y.length);

  mag_msg->magnetic_field.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_z.addr,
    extern_control_table.imu_magnetic_z.length);

  imu_pub_->publish(std::move(imu_msg));
  mag_pub_->publish(std::move(mag_msg));
}
