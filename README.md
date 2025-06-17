# Overview
This is a fork of [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3). This `publish-analog-pins` branch extends the `turtlebot3_node` to broadcast data collected by the OpenCR analog pins A0-A5 to a ROS 2 topic `/analog_pins`. The `main` branch is kept in sync with the ROBOTIS repository.

This repo is part of the larger project [ez-turtlebot3](https://github.com/ez-turtlebot3/ez-turtlebot3), which combines this repo with analog-enabled [OpenCR](https://github.com/ez-turtlebot3/OpenCR) firmware and a [ROS 2 analog processor package](https://github.com/ez-turtlebot3/ez_analog_processor/branches) to read, process, and publish analog data while operating a TurtleBot3.
 
# Requirements
* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
* [Analog-enabled TurtleBot3 ROS 2 OpenCR firmware](https://github.com/ez-turtlebot3/OpenCR)

# Installation
1. If you haven't already, follow the Humble instructions for the [TurtleBot3 SBC setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)
2. Replace the turtlebot3 repo with the analog-enabled fork
  * `cd ~/turtlebot3_ws/src`
  * `rm -r turtlebot3`
  * `git clone https://github.com/ez-turtlebot3/turtlebot3`
  * `cd turtlebot3`
3. Rebuild the turtlebot3_node
  * `colcon build --symlink-install --packages-select turtlebot3_node --allow-overriding turtlebot3_node`

# Use
1. Launch the TurtleBot with the same bringup command as before
  * `ros2 launch turtlebot3_bringup robot.launch.py`
2. View the /analog_pins topic from your remote pc with
  * `ros2 topic echo /analog_pins`
3. Optionally, install the [ROS 2 analog processor package](https://github.com/ez-turtlebot3/ez_analog_processor/branches) to process the raw data from `/analog_pins`.
4. Recommendation: view real-time data in plots with [PlotJuggler](https://github.com/facontidavide/PlotJuggler)

# ROBOTIS links
## Official Documentation
- ⚙️ **[ROBOTIS DYNAMIXEL](https://dynamixel.com/)**
- 📚 **[ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)**
- 📚 **[ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)**

## Community & Support
- 💬 **[ROBOTIS Community Forum](https://forum.robotis.com/)**
- 💬 **[TurtleBot category from ROS Community](https://discourse.ros.org/c/turtlebot/)**
