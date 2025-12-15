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

#include "turtlebot3_node/devices/reset.hpp"

#include <memory>
#include <string>

using robotis::turtlebot3::devices::Reset;

Reset::Reset(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
  const std::string & server_name)
: Devices(nh, dxl_sdk_wrapper)
{
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create reset server");
  srv_ = nh_->create_service<std_srvs::srv::Trigger>(
    server_name,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
    {
      this->command(static_cast<void *>(request.get()), static_cast<void *>(response.get()));
    }
  );

  reset_odom_client_ = nh_->create_client<std_srvs::srv::Trigger>("reset_odometry");
}

void Reset::command(const void * request, void * response)
{
  (void) request;

  std_srvs::srv::Trigger::Response * res = (std_srvs::srv::Trigger::Response *)response;
  std::string result_msg;

  uint8_t reset = 1;
  dxl_sdk_wrapper_->set_data_to_device(
    extern_control_table.imu_re_calibration.addr,
    extern_control_table.imu_re_calibration.length,
    &reset,
    &result_msg);

  RCLCPP_INFO(nh_->get_logger(), "Start Calibration of Gyro");
  rclcpp::sleep_for(std::chrono::seconds(5));
  RCLCPP_INFO(nh_->get_logger(), "Calibration End");
  res->success = true;
  res->message = "Calibration End, Odom reset requested";

  if (!reset_odom_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(nh_->get_logger(), "reset_odometry service not available");
    return;
  }

  auto request_reset_odom = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_odom_client_->async_send_request(
    request_reset_odom,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
    {
      auto response_reset_odom = future.get();
      RCLCPP_INFO(
        nh_->get_logger(),
        "odom reset response: %s",
        response_reset_odom->success ? "success" : "failed");
    });
}

void Reset::request(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
  std_srvs::srv::Trigger::Request req)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>(req);
  auto result = client->async_send_request(request);
}
