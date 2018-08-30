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
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

// class TransformPublisher : public rclcpp::Node
// {
// public:
//   TransformPublisher()
//       : Node("tf_publisher")
//   {
//     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "odom",
//         [this](nav_msgs::msg::Odometry::UniquePtr msg) {
//           this->odom_tf_transform_.transform.translation.x = msg->pose.pose.position.x;
//           this->odom_tf_transform_.transform.translation.y = msg->pose.pose.position.y;
//           this->odom_tf_transform_.transform.translation.z = msg->pose.pose.position.z;
//           this->odom_tf_transform_.transform.rotation      = msg->pose.pose.orientation;
//         });

//     auto timer_callback =
//       [this]() -> void {
//         auto now = rclcpp::Clock().now();

//         this->odom_tf_transform_.header.frame_id = "odom";
//         this->odom_tf_transform_.child_frame_id = "base_footprint";
//         this->odom_tf_transform_.header.stamp = now;

//         this->tf_broadcaster_->sendTransform(this->odom_tf_transform_);
//       };
//     timer_ = this->create_wall_timer(33ms, timer_callback);
//   }

// private:
//   rclcpp::TimerBase::SharedPtr timer_;

//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

//   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//   geometry_msgs::msg::TransformStamped odom_tf_transform_;
// };

geometry_msgs::msg::TransformStamped odom_tf;

void odomMsgCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_tf.transform.translation.x = msg->pose.pose.position.x;
  odom_tf.transform.translation.y = msg->pose.pose.position.y;
  odom_tf.transform.translation.z = msg->pose.pose.position.z;
  odom_tf.transform.rotation      = msg->pose.pose.orientation;

  // auto now = rclcpp::Clock().now();

  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.header.stamp = msg->header.stamp;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("tf_publisher");

  RCLCPP_INFO(node->get_logger(), "Init tf publisher")

  rmw_qos_profile_t odom_qos_profile = rmw_qos_profile_sensor_data;
  odom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  odom_qos_profile.depth = 50;
  odom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  odom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("odom", odomMsgCallback, odom_qos_profile);

  tf2_ros::TransformBroadcaster tf_broadcaster(node);

  rclcpp::WallRate loop_rate(5ms);

  while (rclcpp::ok())
  {
    tf_broadcaster.sendTransform(odom_tf);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  odom_sub = nullptr;
  node = nullptr;
  return 0;
}