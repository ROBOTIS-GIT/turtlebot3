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

#include "turtlebot3_node/sensors/temperature.hpp"

#include <memory>
#include <string>
#include <utility>

#include "turtlebot3_node/control_table.hpp"
#include "sensor_msgs/msg/temperature.hpp"

using robotis::turtlebot3::sensors::Temperature;

Temperature::Temperature(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name)
: Sensors(nh, "base_link")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  temperature_publisher_ = nh->create_publisher<sensor_msgs::msg::Temperature>(topic_name, qos);

  RCLCPP_INFO(nh->get_logger(), "Succeeded to create temperature publisher");
}

void Temperature::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  try {
    auto temp_msg = std::make_unique<sensor_msgs::msg::Temperature>();
    
    // Set timestamp and frame
    temp_msg->header.stamp = now;
    temp_msg->header.frame_id = frame_id_;
    
    // Read temperature from control table (address 112, 4 bytes)
    temp_msg->temperature = dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.temperature.addr,
      extern_control_table.temperature.length);
    
    // Set variance to 0 (unknown)
    temp_msg->variance = 0.0;
    
    // Publish the message
    temperature_publisher_->publish(std::move(temp_msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("temperature"),
      "Exception in temperature publish: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("temperature"),
      "Unknown exception in temperature publish");
  }
}
