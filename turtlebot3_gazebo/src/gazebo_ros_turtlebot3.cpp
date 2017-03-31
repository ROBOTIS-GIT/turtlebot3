/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Taehoon Lim (Darby) */

#include "turtlebot3_gazebo/gazebo_ros_turtlebot3.h"

GazeboRosTurtleBot3::GazeboRosTurtleBot3()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  ROS_ASSERT(init());
}

GazeboRosTurtleBot3::~GazeboRosTurtleBot3()
{
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool GazeboRosTurtleBot3::init()
{
  // initialize ROS parameter
  nh_.param("is_debug", is_debug_, is_debug_);

  // initialize variables
  turtlebot3_direction_.data = "front";
  turtlebot3_linear_velocity_ = 0.0;
  turtlebot3_linear_velocity_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("/scan", 10, &GazeboRosTurtleBot3::laserScanMsgCallBack, this);
  return true;
}

void GazeboRosTurtleBot3::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      direction_vector[num] = msg->range_max;
    }
    else
    {
      direction_vector[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void GazeboRosTurtleBot3::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool GazeboRosTurtleBot3::controlLoop()
{
  if (direction_vector[CENTER] > 1.0)
  {
    updatecommandVelocity(0.15, 0.0);
  }

  if (direction_vector[LEFT] < 0.5)
  {
    updatecommandVelocity(0.0, 1.5);
  }
  else if (direction_vector[RIGHT] < 0.5)
  {
    updatecommandVelocity(0.0, -1.5);
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gazebo_ros_turtlebot3");
  GazeboRosTurtleBot3 gazeboRosTurtleBot3;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    gazeboRosTurtleBot3.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
