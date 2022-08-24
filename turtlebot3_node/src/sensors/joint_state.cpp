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

#include <array>

#include <memory>
#include <string>
#include <utility>

#include "turtlebot3_node/sensors/joint_state.hpp"

using robotis::turtlebot3::sensors::JointState;

JointState::JointState(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  pub_ = nh->create_publisher<sensor_msgs::msg::JointState>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create joint state publisher");
}

void JointState::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();

  static std::array<int32_t, JOINT_NUM> last_diff_position, last_position;

  std::array<int32_t, JOINT_NUM> position =
  {dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_position_left.addr,
      extern_control_table.present_position_left.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_position_right.addr,
      extern_control_table.present_position_right.length)};

  std::array<int32_t, JOINT_NUM> velocity =
  {dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_velocity_left.addr,
      extern_control_table.present_velocity_left.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_velocity_right.addr,
      extern_control_table.present_velocity_right.length)};

  // std::array<int32_t, JOINT_NUM> current =
  //   {dxl_sdk_wrapper->get_data_from_device<int32_t>(
  //     extern_control_table.resent_current_left.addr,
  //     extern_control_table.resent_current_left.length),
  //   dxl_sdk_wrapper->get_data_from_device<int32_t>(
  //     extern_control_table.resent_current_right.addr,
  //     extern_control_table.resent_current_right.length)};

  msg->header.frame_id = this->frame_id_;
  msg->header.stamp = now;

  msg->name.push_back("wheel_left_joint");
  msg->name.push_back("wheel_right_joint");

  msg->position.push_back(TICK_TO_RAD * last_diff_position[0]);
  msg->position.push_back(TICK_TO_RAD * last_diff_position[1]);

  msg->velocity.push_back(RPM_TO_MS * velocity[0]);
  msg->velocity.push_back(RPM_TO_MS * velocity[1]);

  // msg->effort.push_back(current[0]);
  // msg->effort.push_back(current[1]);
  
  // https://stackoverflow.com/questions/65928723/exception-thrown-read-access-violation-it-was-0xfdfdfdfd
  // https://www.reddit.com/r/learnprogramming/comments/1boksy/c_access_violation_reading_location_0xfdfdfdfd/
  // https://social.msdn.microsoft.com/Forums/vstudio/en-US/e6e1d2a0-70e3-4a91-bc84-e634b5bbda98/what-is-this-memory-location?forum=vclanguage
  // TODO: dxl_sdk_wrapper->get_data_from_device returns memory access exception, which is not handled and therefore odometry value is calculated 
  // with exception value (0xFDFDFDFD), which cause huge position change.
  // Visualisation of this issue can be seen here: https://github.com/ros-planning/navigation2/issues/3117
  // This is just a temporary workaround to make it Manwork. Exception should be corectly handled

  static constexpr auto NoMansLand = static_cast<std::int32_t>(0xFDFDFDFD);
  if (position[0] == NoMansLand || position[1] == NoMansLand)
  {
    return;
  }

  last_diff_position[0] += (position[0] - last_position[0]);
  last_diff_position[1] += (position[1] - last_position[1]);

  last_position = position;

  pub_->publish(std::move(msg));
}
