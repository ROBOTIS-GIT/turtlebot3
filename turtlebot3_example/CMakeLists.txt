################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_example)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  message_generation
  visualization_msgs
  interactive_markers
)

find_package(Boost REQUIRED COMPONENTS
    system
    thread
)
################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_action_files(FILES 
    turtlebot3.action
)

generate_messages(DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs  
)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  LIBRARIES turtlebot3_patrol
  CATKIN_DEPENDS
  std_msgs
  message_runtime
  rospy 
  actionlib
  actionlib_msgs
  geometry_msgs
  DEPENDS Boost
)
################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}${Boost_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
install(PROGRAMS 
   scripts/turtlebot3_pointop_key.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/turtlebot3_client.py
  scripts/turtlebot3_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/control_turtlebot3
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/control_turtlebot3
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS 
    nodes/turtlebot3_bumper 
    nodes/turtlebot3_cliff
    nodes/turtlebot3_sonar
    nodes/turtlebot3_illumination
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
################################################################################
# Test
################################################################################
