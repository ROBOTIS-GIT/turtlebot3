// Copyright 2025 EyeZense, Inc.
// Author: Travis Mendoza

#ifndef TURTLEBOT3_NODE__SENSORS__TEMPERATURE_HPP_
#define TURTLEBOT3_NODE__SENSORS__TEMPERATURE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "turtlebot3_node/sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{
class Temperature : public Sensors
{
public:
  explicit Temperature(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name);
  
  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis

#endif  // TURTLEBOT3_NODE__SENSORS__TEMPERATURE_HPP_
