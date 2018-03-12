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
  visualization_msgs
  actionlib_msgs
  actionlib
  interactive_markers
  message_generation
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_action_files(
  DIRECTORY action
  FILES
  turtlebot3.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs sensor_msgs visualization_msgs actionlib_msgs actionlib interactive_markers message_runtime
  DEPENDS Boost
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(turtlebot3_marker_server src/turtlebot3_marker_server.cpp)
add_dependencies(turtlebot3_marker_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_marker_server ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_marker_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/turtlebot3_pointop_key.py
  scripts/turtlebot3_server.py
  scripts/turtlebot3_client.py
  scripts/turtlebot3_obstacle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
