################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_interactions)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED 
  roscpp
  visualization_msgs
  interactive_markers
)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  #INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot3_marker_server src/turtlebot3_marker_server.cpp)
target_link_libraries(turtlebot3_marker_server ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_marker_server 
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
