// Copyright 2025 ROBOTIS CO., LTD.
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

#include "turtlebot3_node/sensors/analog_pins.hpp"

#include <memory>
#include <string>
#include <utility>

#include "turtlebot3_node/control_table.hpp"

using robotis::turtlebot3::sensors::AnalogPins;

AnalogPins::AnalogPins(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name)
: Sensors(nh, "")  // Call parent constructor with node handle and empty frame_id
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  analog_publisher_ = nh->create_publisher<std_msgs::msg::UInt16MultiArray>(topic_name, qos);

  // Read analog pin configuration from parameters
  nh->declare_parameter<std::vector<int64_t>>("sensors.analog_pins");
  nh->get_parameter_or<std::vector<int64_t>>("sensors.analog_pins", configured_pins_, {0, 1, 2, 3, 4, 5});

  // Validate pin numbers (must be 0-5)
  for (auto pin : configured_pins_) {
    if (pin < 0 || pin > 5) {
      RCLCPP_WARN(nh->get_logger(), "Invalid analog pin %ld, must be 0-5", pin);
    }
  }

  RCLCPP_INFO(nh->get_logger(), "Analog pins configured for pins: [%s]", 
    [this]() {
      std::string pins_str;
      for (size_t i = 0; i < configured_pins_.size(); ++i) {
        pins_str += std::to_string(configured_pins_[i]);
        if (i < configured_pins_.size() - 1) pins_str += ", ";
      }
      return pins_str;
    }().c_str());

  RCLCPP_INFO(nh->get_logger(), "Succeeded to create analog pins publisher");
}

void AnalogPins::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  (void)now;  // Mark as unused intentionally to suppress warning
  
  try {
    auto analog_msg = std::make_unique<std_msgs::msg::UInt16MultiArray>();
    
    // Set up dimensions for the message based on configured pins
    analog_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    analog_msg->layout.dim[0].label = "analog_pins";
    analog_msg->layout.dim[0].size = configured_pins_.size();
    analog_msg->layout.dim[0].stride = configured_pins_.size();
    analog_msg->layout.data_offset = 0;
    
    // Initialize data array to hold configured pin values
    analog_msg->data.resize(configured_pins_.size());
    
    // Array of control table entries for easy access
    const robotis::turtlebot3::ControlItem* analog_entries[] = {
      &extern_control_table.analog_a0,
      &extern_control_table.analog_a1,
      &extern_control_table.analog_a2,
      &extern_control_table.analog_a3,
      &extern_control_table.analog_a4,
      &extern_control_table.analog_a5
    };
    
    // Read values only from configured pins
    for (size_t i = 0; i < configured_pins_.size(); ++i) {
      int pin = static_cast<int>(configured_pins_[i]);
      if (pin >= 0 && pin <= 5) {
        analog_msg->data[i] = dxl_sdk_wrapper->get_data_from_device<uint16_t>(
          analog_entries[pin]->addr,
          analog_entries[pin]->length);
      } else {
        analog_msg->data[i] = 0;  // Invalid pin, set to 0
      }
    }
    
    // Publish the message
    analog_publisher_->publish(std::move(analog_msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("analog_pins"),
      "Exception in analog_pins publish: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("analog_pins"),
      "Unknown exception in analog_pins publish");
  }
}
