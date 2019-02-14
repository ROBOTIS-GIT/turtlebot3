/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
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

#ifndef TURTLEBOT3_JOINT_STATE_H
#define TURTLEBOT3_JOINT_STATE_H

#include <memory>
#include <array>
#include <mutex>

#include "rclcpp/time.hpp"

#include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace turtlebot3
{
class JointState
{
 public:
  JointState(){};
  virtual ~JointState(){};

  sensor_msgs::msg::JointState::SharedPtr getJointState(const rclcpp::Time now);
  void updateRadianFromTick(const turtlebot3_msgs::msg::SensorState::SharedPtr state);

 private:
  std::array<int32_t,2> last_tick_;
  std::array<double,2> last_rad_;

  std::mutex last_rad_mutex_;
};
}

#endif //TURTLEBOT3_JOINT_STATE_H