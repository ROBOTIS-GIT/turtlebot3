/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Darby Lim */

#include "joint_state.h"

using namespace turtlebot3;

constexpr double DXL_TICK2RAD = (0.087890625f * 3.14159265359f) / 180.0f;
constexpr char FRAME_ID_OF_JOINT_STATE[] = "base_link";
constexpr char LEFT_WHEEL_JOINT_NAME[] = "wheel_left_joint";
constexpr char RIGHT_WHEEL_JOINT_NAME[] = "wheel_right_joint";

sensor_msgs::msg::JointState::SharedPtr JointState::getJointState(const rclcpp::Time now)
{
  static rclcpp::Time last_time = now;
  rclcpp::Duration duration(now.nanoseconds() - last_time.nanoseconds());

  auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();

  joint_state->header.frame_id = FRAME_ID_OF_JOINT_STATE;
  joint_state->header.stamp = now;

  joint_state->name.push_back(LEFT_WHEEL_JOINT_NAME);
  joint_state->name.push_back(RIGHT_WHEEL_JOINT_NAME);

  std::lock_guard<std::mutex> lock(mutex_);
  joint_state->position.push_back(last_rad_[0]);
  joint_state->position.push_back(last_rad_[1]);

  if (duration > rclcpp::Duration(0,0)) // can't have negative or zero duration
  {
    joint_state->velocity.push_back(DXL_TICK2RAD*last_diff_tick_[0]/(duration.seconds()));
    joint_state->velocity.push_back(DXL_TICK2RAD*last_diff_tick_[1]/(duration.seconds()));
  }

  joint_state->effort.push_back(0.0f);
  joint_state->effort.push_back(0.0f);

  last_time = now;
  return joint_state;
}

void JointState::updateRadianFromTick(const turtlebot3_msgs::msg::SensorState::SharedPtr state)
{
  std::array<int32_t,2> current_tick = {state->left_encoder, state->right_encoder};
  static std::array<int32_t,2> last_tick = current_tick;

  for (uint8_t index = 0; index < current_tick.size(); index++)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_diff_tick_[index] = current_tick[index] - last_tick[index];
    last_rad_[index]      += (DXL_TICK2RAD * static_cast<double>(last_diff_tick_[index]));
    last_tick[index]       = current_tick[index];
  }
}
