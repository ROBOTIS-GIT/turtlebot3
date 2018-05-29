################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_example)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  interactive_markers
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  actionlib_msgs
  turtlebot3_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_action_files(
  FILES
  Turtlebot3.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    interactive_markers
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    actionlib_msgs
    turtlebot3_msgs
    message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS 
  nodes/turtlebot3_obstacle
  nodes/turtlebot3_client
  nodes/turtlebot3_server
  nodes/turtlebot3_pointop_key
  nodes/turtlebot3_bumper 
  nodes/turtlebot3_cliff
  nodes/turtlebot3_sonar
  nodes/turtlebot3_illumination
  nodes/turtlebot3_marker_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
