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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include "joint_state.h"
#include "odometry.h"

using namespace std::chrono_literals;

namespace turtlebot3
{
class TurtleBot3 : public rclcpp::Node
{
 public:
  explicit TurtleBot3()
   : Node("turtlebot3_node")
  {
    RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");

    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    joint_state_ = std::make_unique<JointState>();
    odom_ = std::make_unique<Odometry>();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    time_pub_ = this->create_publisher<builtin_interfaces::msg::Time>("time_sync", 10);

    sensor_state_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(
      "sensor_state",
      10,
      [this](const turtlebot3_msgs::msg::SensorState::SharedPtr sensor_state) -> void
      {
        joint_state_->updateRadianFromTick(sensor_state);
      }
    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 
      10, 
      [this](const sensor_msgs::msg::Imu::SharedPtr imu) -> void
      {
        odom_->updateImu(imu);
      }
    );

    joint_state_timer_ = this->create_wall_timer(
      33ms,
      [this]()
      {
        constexpr double WheelRadius = 0.033f;

        sensor_msgs::msg::JointState joint_state_msg = joint_state_->getJointState(now());

        odom_->updateJointState(joint_state_msg);

        odom_pub_->publish(odom_->getOdom(now(), WheelRadius));
        tf_broadcaster_->sendTransform(odom_->getOdomTf());

        joint_state_pub_->publish(joint_state_msg);
      }
    );

    odom_timer_ = this->create_wall_timer(
      33ms,
      [this]()
      {
        // constexpr double WheelRadius = 0.033f;
        // odom_->updateJointState(*joint_state_->getJointState(now()));

        // odom_pub_->publish(odom_->getOdom(now(), WheelRadius));
        // tf_broadcaster_->sendTransform(odom_->getOdomTf());
      }
    );

    time_timer_ = this->create_wall_timer(
      1s,
      [this]()
      {
        auto time_msg = builtin_interfaces::msg::Time();
        time_msg = now();
        time_pub_->publish(time_msg);
      }
    );
  }

  virtual ~TurtleBot3(){};

 private:
  rclcpp::Node::SharedPtr node_handle_;

  std::unique_ptr<JointState> joint_state_;
  std::unique_ptr<Odometry> odom_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr sensor_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub_;

  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr time_timer_;
};
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<turtlebot3::TurtleBot3>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}