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

#ifndef TURTLEBOT3_NODE__DEVICES__DEVICES_HPP_
#define TURTLEBOT3_NODE__DEVICES__DEVICES_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "turtlebot3_node/control_table.hpp"
#include "turtlebot3_node/dynamixel_sdk_wrapper.hpp"


namespace robotis
{
namespace turtlebot3
{
extern const ControlTable extern_control_table;
namespace devices
{
class Devices
{
public:
  explicit Devices(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
  : nh_(nh),
    dxl_sdk_wrapper_(dxl_sdk_wrapper)
  {
  }

  virtual void command(const void * request, void * response) = 0;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::ServicesQoS());
};
}  // namespace devices
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__DEVICES__DEVICES_HPP_
