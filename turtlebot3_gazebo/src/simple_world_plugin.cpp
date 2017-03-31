#include <ros/ros.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "simple_world_plugin");

  ROS_INFO("Hello world!");
}
