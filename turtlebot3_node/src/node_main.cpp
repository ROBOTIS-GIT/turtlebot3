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

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "joint_state.h"

constexpr char SensorStateTopic[] = "sensor_state";
constexpr char JointStateTopic[] = "joint_states";
constexpr char OdomTopic[] = "odom";
constexpr char ScanHalfTopic[] = "scan_half";
constexpr char ScanTopic[] = "scan";
constexpr char ImuTopic[] = "imu";
constexpr char TimeTopic[] = "time_sync";

constexpr double JointStatePublishPeriodSec = 0.03;

namespace turtlebot3
{
class TurtleBot3 : public rclcpp::Node
{
 public:
  explicit TurtleBot3(const std::string &node_name)
   : Node(node_name)
  {
    this->joint_state_ = std::shared_ptr<JointState>();

    RCLCPP_INFO(this->get_logger(), "Init Joint State Publisher");
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(JointStateTopic, rmw_qos_profile_default);

    auto sensor_state_callback = 
      [this](const turtlebot3_msgs::msg::SensorState::SharedPtr sensor_state) -> void
      {
        this->joint_state_->updateRadianFromTick(sensor_state);
      };

    sensor_state_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(SensorStateTopic, sensor_state_callback);

    joint_state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(JointStatePublishPeriodSec * 1000)),
      [this]()
      {
        this->joint_state_pub_->publish(this->joint_state_->getJointState(this->now()));
      }
    );
  }

  virtual ~TurtleBot3(){};

 private:
  std::shared_ptr<JointState> joint_state_;

  rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr sensor_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_half_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub_;

  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr time_timer_;
};
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("turtlebot3_node");

  rclcpp::spin(node);

  return 0;
}