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

#include "lidar.h"

using namespace turtlebot3;

void Lidar::makeFullRange(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(laser_scan_msg_mutex_);
  laser_scan_msg_.header.frame_id = msg->header.frame_id;
  laser_scan_msg_.header.stamp = msg->header.stamp;

  laser_scan_msg_.angle_min = msg->angle_min;
  laser_scan_msg_.angle_max = msg->angle_max;
  laser_scan_msg_.angle_increment = msg->angle_increment;
  laser_scan_msg_.time_increment = msg->time_increment;
  laser_scan_msg_.scan_time = msg->scan_time;
  laser_scan_msg_.range_min = msg->range_min;
  laser_scan_msg_.range_max = msg->range_max; 
  laser_scan_msg_.ranges.resize(360);

  for (uint16_t i=0, j=0; i<180; j=j+2, i++)
  {
    laser_scan_msg_.ranges[j] = msg->ranges[i];
    laser_scan_msg_.ranges[j+1] = msg->ranges[i];
  }
}

sensor_msgs::msg::LaserScan Lidar::getLaserScan(const rclcpp::Time now)
{
  std::lock_guard<std::mutex> lock(laser_scan_msg_mutex_);
  laser_scan_msg_.header.stamp = now;
  return laser_scan_msg_;
}