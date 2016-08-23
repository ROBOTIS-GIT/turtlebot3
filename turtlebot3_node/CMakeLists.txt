################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_node)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  turtlebot3_msgs
  nodelet
  tf
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp sensor_msgs std_msgs geometry_msgs nav_msgs turtlebot3_msgs nodelet tf
)

################################################################################
# Build
################################################################################
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(turtlebot3_node src/turtlebot3_node.cpp)
add_dependencies(turtlebot3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_node ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
