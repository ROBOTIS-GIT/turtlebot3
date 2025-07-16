# TurtleBot3 ROS 2 Codebase Guide

## Build Commands
- **Build single package**: `colcon build --packages-select turtlebot3_node`
- **Build all**: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
- **Source workspace**: `source install/setup.bash`
- **Launch robot**: Launching the robot is only possible from the turtlebot's raspberry pi. You, the agentic coding tool, only operate on the codebase from the remote PC. Therefore launching is not something you can do.
- **View analog data**: `ros2 topic echo /analog_pins`

## Architecture
- **Main packages**: `turtlebot3_node` (core driver), `turtlebot3_bringup` (launch files), `turtlebot3_description` (URDF), `turtlebot3_navigation2`, `turtlebot3_teleop`
- **Core class**: `robotis::turtlebot3::TurtleBot3` - main node managing sensors, devices, motors
- **New feature**: Analog pins A0-A5 published to `/analog_pins` topic via `AnalogPins` sensor class
- **Communication**: Uses DynamixelSDKWrapper to read from OpenCR control table addresses 350-360 for analog pins
- **Dependencies**: ROS 2 Humble, Dynamixel SDK, custom OpenCR firmware with analog support

## Code Style
- **Standard**: C++17, follows ROS 2 conventions
- **Naming**: snake_case for variables/functions, PascalCase for classes, namespaces: `robotis::turtlebot3::`
- **Includes**: System includes first, then ROS msgs, then local headers with relative paths
- **Error handling**: try-catch blocks with RCLCPP_ERROR logging
- **Memory**: Smart pointers (std::shared_ptr, std::unique_ptr), RAII patterns
- **Formatting**: 2-space indentation, header guards with full path, Apache 2.0 license headers
