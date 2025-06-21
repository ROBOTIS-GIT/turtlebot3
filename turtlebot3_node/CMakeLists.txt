################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlebot3_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/devices/motor_power.cpp"
  "src/devices/sound.cpp"
  "src/devices/reset.cpp"

  "src/diff_drive_controller.cpp"
  "src/dynamixel_sdk_wrapper.cpp"
  "src/odometry.cpp"
  "src/turtlebot3.cpp"

  "src/sensors/battery_state.cpp"
  "src/sensors/imu.cpp"
  "src/sensors/joint_state.cpp"
  "src/sensors/sensor_state.cpp"
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${dynamixel_sdk_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_lib
  ${dynamixel_sdk_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${turtlebot3_msgs_TARGETS}
  message_filters::message_filters
  rclcpp::rclcpp
  rcutils::rcutils
  tf2::tf2
  tf2_ros::tf2_ros
)

set(EXECUTABLE_NAME "turtlebot3_ros")
add_executable(${EXECUTABLE_NAME} src/node_main.cpp)
target_link_libraries(${EXECUTABLE_NAME}
  ${PROJECT_NAME}_lib
)

################################################################################
# Install
################################################################################
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(
  dynamixel_sdk
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  turtlebot3_msgs
)
ament_export_include_directories(include)
ament_package()
