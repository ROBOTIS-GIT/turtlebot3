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
  odom_qos_profile.depth = 1;
  odom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
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