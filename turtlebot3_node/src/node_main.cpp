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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "constants.h"
#include "joint_state.h"
#include "lidar.h"
#include "odometry.h"

namespace turtlebot3
{
class TurtleBot3 : public rclcpp::Node
{
 public:
  explicit TurtleBot3(const std::string &node_name)
   : Node(node_name)
  {
    RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
    joint_state_ = std::make_shared<JointState>();
    lidar_ = std::make_shared<Lidar>();
    odom_ = std::make_shared<Odometry>();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(JointStateTopic, rmw_qos_profile_default);
    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(ScanTopic, rmw_qos_profile_default);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(OdomTopic, rmw_qos_profile_default);
    time_pub_ = this->create_publisher<builtin_interfaces::msg::Time>(TimeTopic, rmw_qos_profile_default);

    auto sensor_state_callback = 
      [this](const turtlebot3_msgs::msg::SensorState::SharedPtr sensor_state) -> void
      {
        this->joint_state_->updateRadianFromTick(sensor_state);
      };

    sensor_state_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(SensorStateTopic, sensor_state_callback);

    auto laser_scan_callback = 
      [this](const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) -> void
      {
        this->lidar_->makeFullRange(laser_scan);
      };

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(ScanHalfTopic, laser_scan_callback);

    auto imu_callback = 
      [this](const sensor_msgs::msg::Imu::SharedPtr imu) -> void
      {
        this->odom_->updateImu(imu);
      };

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(ImuTopic, imu_callback);

    joint_state_timer_ = this->create_wall_timer(
      JointStatePublishPeriodMillis,
      [this]()
      {
        this->joint_state_pub_->publish(this->joint_state_->getJointState(this->now()));
      }
    );

    laser_scan_timer_ = this->create_wall_timer(
      ScanPublishPeriodMillis,
      [this]()
      {
        this->laser_scan_pub_->publish(this->lidar_->getLaserScan(this->now()));
      }
    );

    odom_timer_ = this->create_wall_timer(
      OdometryPublishPeriodMillis,
      [this]()
      {
        this->odom_->updateJointState(this->joint_state_->getJointState(this->now()));
        this->odom_pub_->publish(this->odom_->getOdom(this->now(), WheelRadius));
        this->tf_broadcaster_->sendTransform(this->odom_->getOdomTf());
      }
    );

    time_timer_ = this->create_wall_timer(
      TimeSyncPublishPeriodMillis,
      [this]()
      {
        auto time_msg = builtin_interfaces::msg::Time();
        time_msg = this->now();
        this->time_pub_->publish(time_msg);
      }
    );
  }

  virtual ~TurtleBot3(){};

 private:
  std::shared_ptr<JointState> joint_state_;
  std::shared_ptr<Lidar> lidar_;
  std::shared_ptr<Odometry> odom_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr sensor_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub_;

  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr laser_scan_timer_;
  rclcpp::TimerBase::SharedPtr time_timer_;
};
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<turtlebot3::TurtleBot3>("turtlebot3_node");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}