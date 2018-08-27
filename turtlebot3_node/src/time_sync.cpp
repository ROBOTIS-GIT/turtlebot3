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

#include <chrono>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class TimeSync : public rclcpp::Node
{
public:
  TimeSync()
  : Node("time_sync")
  {
    time_pub_ = this->create_publisher<builtin_interfaces::msg::Time>("time_sync");
    auto timer_callback =
      [this]() -> void {
        auto time_msg = builtin_interfaces::msg::Time();
        time_msg = rclcpp::Clock().now();
        this->time_pub_->publish(time_msg);
      };
    timer_ = this->create_wall_timer(5ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeSync>());
  rclcpp::shutdown();
  return 0;
}