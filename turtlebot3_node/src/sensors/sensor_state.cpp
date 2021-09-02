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

#include "turtlebot3_node/sensors/sensor_state.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::SensorState;

SensorState::SensorState(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name,
  const uint8_t & bumper_forward,
  const uint8_t & bumper_backward,
  const uint8_t & illumination,
  const uint8_t & cliff,
  const uint8_t & sonar)
: Sensors(nh),
  bumper_forward_(bumper_forward),
  bumper_backward_(bumper_backward),
  illumination_(illumination),
  cliff_(cliff),
  sonar_(sonar)
{
  pub_ = nh->create_publisher<turtlebot3_msgs::msg::SensorState>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create sensor state publisher");
}

void SensorState::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto msg = std::make_unique<turtlebot3_msgs::msg::SensorState>();

  msg->header.stamp = now;

  if (bumper_forward_ || bumper_backward_) {
    uint8_t bumper_push_state;
    uint8_t bumper_forward_state;
    uint8_t bumper_backward_state;

    bumper_forward_state = dxl_sdk_wrapper->get_data_from_device<uint8_t>(
      extern_control_table.bumper_1.addr,
      extern_control_table.bumper_1.length);

    bumper_backward_state = dxl_sdk_wrapper->get_data_from_device<uint8_t>(
      extern_control_table.bumper_2.addr,
      extern_control_table.bumper_2.length);

    bumper_push_state = bumper_forward_state << 0;
    bumper_push_state |= bumper_backward_state << 1;

    msg->bumper = bumper_push_state;
  } else if (!bumper_forward_ && !bumper_backward_) {
    msg->bumper = 0;
  }

  if (cliff_) {
    msg->cliff = dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.ir.addr,
      extern_control_table.ir.length);
  } else {
    msg->cliff = 0.0f;
  }

  if (sonar_) {
    msg->sonar = dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.sonar.addr,
      extern_control_table.sonar.length);
  } else {
    msg->sonar = 0.0f;
  }

  if (illumination_) {
    msg->illumination = dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.illumination.addr,
      extern_control_table.illumination.length);
  } else {
    msg->illumination = 0.0f;
  }

  // update button state
  uint8_t button_push_state;
  uint8_t button_0_state;
  uint8_t button_1_state;

  button_0_state = dxl_sdk_wrapper->get_data_from_device<uint8_t>(
    extern_control_table.button_1.addr,
    extern_control_table.button_1.length);

  button_1_state = dxl_sdk_wrapper->get_data_from_device<uint8_t>(
    extern_control_table.button_2.addr,
    extern_control_table.button_2.length);

  button_push_state = button_0_state << 0;
  button_push_state |= button_1_state << 1;

  msg->button = button_push_state;

  // update torque enable state
  msg->torque = dxl_sdk_wrapper->get_data_from_device<bool>(
    extern_control_table.motor_torque_enable.addr,
    extern_control_table.motor_torque_enable.length);

  msg->left_encoder = dxl_sdk_wrapper->get_data_from_device<int32_t>(
    extern_control_table.present_position_left.addr,
    extern_control_table.present_position_left.length);

  msg->right_encoder = dxl_sdk_wrapper->get_data_from_device<int32_t>(
    extern_control_table.present_position_right.addr,
    extern_control_table.present_position_right.length);

  msg->battery = 0.01f * dxl_sdk_wrapper->get_data_from_device<int32_t>(
    extern_control_table.battery_voltage.addr,
    extern_control_table.battery_voltage.length);

  pub_->publish(std::move(msg));
}
