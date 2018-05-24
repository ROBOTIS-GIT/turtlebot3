/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

#ifndef FLAT_WORLD_IMU_NODE_H_
#define FLAT_WORLD_IMU_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#define GRAVITY 9.8

class FlatWorldImuNode
{
 public:
  FlatWorldImuNode();
  ~FlatWorldImuNode();
  bool init();

 private:
  ros::NodeHandle nh_;
  ros::Time last_published_time_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  void msgCallback(const sensor_msgs::ImuConstPtr imu_in);
};

#endif // FLAT_WORLD_IMU_NODE_H_
