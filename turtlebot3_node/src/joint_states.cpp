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
  std::lock_guard<std::mutex> lock(last_rad_mutex_);
  auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();

  joint_state->header.frame_id = FRAME_ID_OF_JOINT_STATE;
  joint_state->header.stamp = now;

  joint_state->name.push_back(LEFT_WHEEL_JOINT_NAME);
  joint_state->name.push_back(RIGHT_WHEEL_JOINT_NAME);

  joint_state->position.push_back(last_rad_[0]);
  joint_state->position.push_back(last_rad_[1]);

  joint_state->velocity.push_back(0.0f);
  joint_state->velocity.push_back(0.0f);

  joint_state->effort.push_back(0.0f);
  joint_state->effort.push_back(0.0f);

  return joint_state;
}

void JointState::updateRadianFromTick(const turtlebot3_msgs::msg::SensorState::SharedPtr state)
{
  std::array<int32_t,2> current_tick = {state->left_encoder, state->right_encoder};
  std::array<int32_t,2> last_diff_tick;

  for (uint8_t index = 0; index < current_tick.size(); index++)
  {
    std::lock_guard<std::mutex> lock(last_rad_mutex_);
    last_diff_tick[index] = current_tick[index] - last_tick_[index];
    last_rad_[index]      += (DXL_TICK2RAD * static_cast<double>(last_diff_tick[index]));
    last_tick_[index]      = current_tick[index];
  }
}

#if 0

#include <chrono>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define TICK2RAD  0.001533981f  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

using namespace std::chrono_literals;

sensor_msgs::msg::JointState joint_state;
float last_rad[2] = {0.0f, 0.0f};  

void sensorStateMsgCallback(const turtlebot3_msgs::msg::SensorState::SharedPtr msg)
{
  int32_t current_tick[2] = {msg->left_encoder, msg->right_encoder};
  static int32_t last_tick[2] = {0, 0};
  int32_t last_diff_tick[2] = {0, 0};
    
  joint_state.header.stamp    = msg->header.stamp;

  for (uint8_t index = 0; index < 2; index++)
  {
    last_diff_tick[index] = current_tick[index] - last_tick[index];
    last_rad[index]      += (TICK2RAD * (double)(last_diff_tick[index]));
    last_tick[index]      = current_tick[index];
  }

  last_tick[0] = msg->left_encoder;
  last_tick[1] = msg->right_encoder;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("joint_states_publisher");

  RCLCPP_INFO(node->get_logger(), "Init joint_states publisher");

  rmw_qos_profile_t sensor_state_qos_profile = rmw_qos_profile_sensor_data;
  sensor_state_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  sensor_state_qos_profile.depth = 1;
  sensor_state_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  sensor_state_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  auto sensor_state_sub = node->create_subscription<turtlebot3_msgs::msg::SensorState>("sensor_state", sensorStateMsgCallback, sensor_state_qos_profile);
  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states");

  rclcpp::WallRate loop_rate(5ms);

  while (rclcpp::ok())
  {
    joint_state.name.clear();
    joint_state.position.clear();
    joint_state.velocity.clear();
    joint_state.effort.clear();

    joint_state.header.frame_id = "base_link";

    joint_state.name.push_back("wheel_left_joint");
    joint_state.name.push_back("wheel_right_joint");

    joint_state.position.push_back(last_rad[0]);
    joint_state.position.push_back(last_rad[1]);

    joint_state.velocity.push_back(0.0f);
    joint_state.velocity.push_back(0.0f);

    joint_state.effort.push_back(0.0f);
    joint_state.effort.push_back(0.0f);

    joint_state_pub->publish(joint_state);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  sensor_state_sub = nullptr;
  joint_state_pub = nullptr;
  node = nullptr;
  return 0;
}
#endif