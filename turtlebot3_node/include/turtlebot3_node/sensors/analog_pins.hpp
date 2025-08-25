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

#ifndef TURTLEBOT3_NODE__SENSORS__ANALOG_PINS_HPP_
#define TURTLEBOT3_NODE__SENSORS__ANALOG_PINS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

#include "turtlebot3_node/sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{
class AnalogPins : public Sensors
{
public:
  explicit AnalogPins(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name);
  
  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr analog_publisher_;
  std::vector<int> configured_pins_;  // Which pins to read from
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis

#endif  // TURTLEBOT3_NODE__SENSORS__ANALOG_PINS_HPP_
