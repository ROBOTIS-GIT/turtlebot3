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

#include "turtlebot3_node/devices/motor_power.hpp"

#include <memory>
#include <string>

using robotis::turtlebot3::devices::MotorPower;

MotorPower::MotorPower(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
  const std::string & server_name)
: Devices(nh, dxl_sdk_wrapper)
{
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create motor power server");
  srv_ = nh_->create_service<std_srvs::srv::SetBool>(
    server_name,
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
    {
      this->command(static_cast<void *>(request.get()), static_cast<void *>(response.get()));
    }
  );
}

void MotorPower::command(const void * request, void * response)
{
  std_srvs::srv::SetBool::Request req = *(std_srvs::srv::SetBool::Request *)request;
  std_srvs::srv::SetBool::Response * res = (std_srvs::srv::SetBool::Response *)response;

  res->success = dxl_sdk_wrapper_->set_data_to_device(
    extern_control_table.motor_torque_enable.addr,
    extern_control_table.motor_torque_enable.length,
    reinterpret_cast<uint8_t *>(&req.data),
    &res->message);
}

void MotorPower::request(
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
  std_srvs::srv::SetBool::Request req)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>(req);
  auto result = client->async_send_request(request);
}
