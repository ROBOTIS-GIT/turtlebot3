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

#ifndef TURTLEBOT3_NODE__SENSORS__SENSOR_STATE_HPP_
#define TURTLEBOT3_NODE__SENSORS__SENSOR_STATE_HPP_

#include <turtlebot3_msgs/msg/sensor_state.hpp>

#include <memory>
#include <string>

#include "turtlebot3_node/sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{
class SensorState : public Sensors
{
public:
  explicit SensorState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "sensor_state",
    const uint8_t & bumper_forward = 0,
    const uint8_t & bumper_backward = 0,
    const uint8_t & illumination = 0,
    const uint8_t & cliff = 0,
    const uint8_t & sonar = 0);

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<turtlebot3_msgs::msg::SensorState>::SharedPtr pub_;

  uint8_t bumper_forward_;
  uint8_t bumper_backward_;
  uint8_t illumination_;
  uint8_t cliff_;
  uint8_t sonar_;
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__SENSORS__SENSOR_STATE_HPP_
