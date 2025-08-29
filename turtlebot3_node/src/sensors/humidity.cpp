// Copyright 2025 EyeZense, Inc.
// Author: Travis Mendoza

#include "turtlebot3_node/sensors/humidity.hpp"

#include <memory>
#include <string>
#include <utility>

#include "turtlebot3_node/control_table.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"

using robotis::turtlebot3::sensors::Humidity;

Humidity::Humidity(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name)
: Sensors(nh, "base_link")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  humidity_publisher_ = nh->create_publisher<sensor_msgs::msg::RelativeHumidity>(topic_name, qos);

  RCLCPP_INFO(nh->get_logger(), "Succeeded to create humidity publisher");
}

void Humidity::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  try {
    auto humidity_msg = std::make_unique<sensor_msgs::msg::RelativeHumidity>();
    
    // Set timestamp and frame
    humidity_msg->header.stamp = now;
    humidity_msg->header.frame_id = frame_id_;
    
    // Read humidity from control table (address 116, 4 bytes)  
    // Firmware stores as float from DHT sensor (percentage)
    float humidity_percent = dxl_sdk_wrapper->get_data_from_device<float>(
      extern_control_table.humidity.addr,
      extern_control_table.humidity.length);
    
    // Convert percentage to ratio (0.0-1.0) for ROS standard compliance
    humidity_msg->relative_humidity = humidity_percent / 100.0;
    
    // Set variance to 0 (unknown)
    humidity_msg->variance = 0.0;
    
    // Publish the message
    humidity_publisher_->publish(std::move(humidity_msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("humidity"),
      "Exception in humidity publish: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("humidity"),
      "Unknown exception in humidity publish");
  }
}
