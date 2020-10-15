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

#include "turtlebot3_node/sensors/battery_state.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::BatteryState;

BatteryState::BatteryState(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name)
: Sensors(nh)
{
  pub_ = nh->create_publisher<sensor_msgs::msg::BatteryState>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create battery state publisher");
}

void BatteryState::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto msg = std::make_unique<sensor_msgs::msg::BatteryState>();

  msg->header.stamp = now;

  msg->design_capacity = 1.8f;

  msg->voltage = 0.01f * dxl_sdk_wrapper->get_data_from_device<int32_t>(
    extern_control_table.battery_voltage.addr,
    extern_control_table.battery_voltage.length);

  msg->percentage = 0.01f * dxl_sdk_wrapper->get_data_from_device<int32_t>(
    extern_control_table.battery_percentage.addr,
    extern_control_table.battery_percentage.length);

  msg->voltage <= 7.0 ? msg->present = false : msg->present = true;

  pub_->publish(std::move(msg));
}
