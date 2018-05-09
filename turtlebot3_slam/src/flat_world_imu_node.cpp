/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <turtlebot3_slam/flat_world_imu_node.h>

FlatWorldImuNode::FlatWorldImuNode()
{
  ROS_ASSERT(init());
}

FlatWorldImuNode::~FlatWorldImuNode()
{
}

bool FlatWorldImuNode::init()
{
  publisher_  = nh_.advertise<sensor_msgs::Imu>("imu_out", 10);
  subscriber_ = nh_.subscribe("imu_in", 150, &FlatWorldImuNode::msgCallback, this);

  return true;
}

void FlatWorldImuNode::msgCallback(const sensor_msgs::ImuConstPtr imu_in)
{
  if (last_published_time_.isZero() || imu_in->header.stamp > last_published_time_)
  {
    last_published_time_ = imu_in->header.stamp;
    sensor_msgs::Imu imu_out = *imu_in;
    imu_out.linear_acceleration.x = 0.0;
    imu_out.linear_acceleration.y = 0.0;
    imu_out.linear_acceleration.z = GRAVITY;
    publisher_.publish(imu_out);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "flat_world_imu_node");

  FlatWorldImuNode flat_world_imu_node;

  ros::spin();

  return 0;
}
