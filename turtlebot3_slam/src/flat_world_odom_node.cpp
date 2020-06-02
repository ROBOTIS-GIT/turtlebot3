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

#include <turtlebot3_slam/flat_world_odom_node.h>

FlatWorldOdomNode::FlatWorldOdomNode()
{
  bool init_result = init();
  ROS_ASSERT(init_result);
}

FlatWorldOdomNode::~FlatWorldOdomNode()
{
}

bool FlatWorldOdomNode::init()
{
  publisher_  = nh_.advertise<nav_msgs::Odometry>("odom_out", 10);
  subscriber_ = nh_.subscribe("odom", 150, &FlatWorldOdomNode::msgCallback, this);

  return true;
}

void FlatWorldOdomNode::msgCallback(const nav_msgs::Odometry::ConstPtr odom_in)
{
  if (last_published_time_.isZero() || odom_in->header.stamp > last_published_time_)
  {
    last_published_time_ = odom_in->header.stamp;
    nav_msgs::Odometry odom_out = *odom_in;
    publisher_.publish(odom_out);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "flat_world_odom_node");

  FlatWorldOdomNode flat_world_odom_node;

  ros::spin();

  return 0;
}
