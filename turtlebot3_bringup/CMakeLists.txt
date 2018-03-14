################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_bringup)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  turtlebot3_msgs
)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs diagnostic_msgs turtlebot3_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot3_diagnostics src/turtlebot3_diagnostics.cpp)
add_dependencies(turtlebot3_diagnostics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_diagnostics ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_diagnostics
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS 
  scripts/create_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES 99-turtlebot3-cdc.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
