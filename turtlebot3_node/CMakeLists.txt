################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs turtlebot3_msgs tf2)

add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher rclcpp std_msgs sensor_msgs nav_msgs tf2_ros)

add_executable(joint_states src/joint_states.cpp)
ament_target_dependencies(joint_states rclcpp std_msgs sensor_msgs turtlebot3_msgs)

add_executable(time_sync src/time_sync.cpp)
ament_target_dependencies(time_sync rclcpp std_msgs builtin_interfaces)

install(TARGETS
  odometry
  tf_publisher
  joint_states
  time_sync
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(turtlebot3_msgs)
ament_export_dependencies(builtin_interfaces)
# ament_export_include_directories(include)
ament_package()