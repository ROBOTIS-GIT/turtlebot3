# TurtleBot3

## Setup the Remote PC

### Requirements
- PC or Laptop with Ubuntu 16.04 installed (any distribution based on 16.04)
- ROS Kinetic Kame installed: http://wiki.ros.org/kinetic/Installation/Ubuntu


### Installation
- Install dependencies 
 `
 sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
 `
`
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone git@github.com:alphanote/robo_turtlebot3.git
cd ~/catkin_ws && catkin_make
`
- Modify the bashrc file: `nano ~/.bashrc`
- Modify the address of localhost in the ROS_MASTER_URI and ROS_HOSTNAME with the IP address of the WiFi interface. Use `ifconfig` command to find this IP.
- Source the bashrc with the command below
`source ~/.bashrc`


## Setup the Robot

### Requirements
- Turtlebot3 using Raspberry Pi 3 B+
- OS Linux Raspbian 4.19.36-v7+ 
- ROS Kinetic Kame

### Installation
- Download the OS image using this link: http://www.robotis.com/service/download.php?no=1738, it already contains ROS installed.
- Flash the Raspberry's SD card using the software etcher.io on your PC or Laptop.
- Insert SD card in the Robot's Raspberry 
- Expand filesystem to use a whole SD card.
`
sudo raspi-config
(select 7 Advanced Options > A1 Expand Filesystem)
`
- Synchronize and set computer's date and time by querying a Network Time Protocol (NTP) server
`
sudo apt-get install ntpdate
sudo ntpdate ntp.ubuntu.com
`
- Network configuration for ROS
`
  nano ~/.bashrc
  (modify `localhost` to REMOTE_PC_IP and RASPBERRY_PI_3_IP which you can find with the command "iwconfig")
`
`
  export ROS_MASTER_URI=http://REMOTE_PC_IP:11311
  export ROS_HOSTNAME=RASPBERRY_PI_3_IP
`
`
  source ~/.bashrc
`
- Remove previously installed Turtlebot's ROS node lidar packages
`
cd ~/catkin_ws/src/
rm -r turtlebot3
rm -r hls_lfcd_lds_driver 
`
- Download and compile Innovatech Robot Code
`
cd ~/catkin_ws/src/
git clone git@github.com:alphanote/robo_turtlebot3.git
git clone git@github.com:alphanote/robo_rplidar_ros.git
mv robo_turtlebot3 turtlebot3
mv robo_rplidar_ros rplidar_ros
cd ..
catkin_make_isolated
`

## How to run
- Once you’re done the wireless configuration, you can connect to Raspberry Pi via SSH from your desktop or laptop:
`
ssh pi@192.168.xxx.xxx (The IP 192.168.xxx.xxx is your Raspberry Pi's IP or hostname)
`

- SLAM commands can be found here http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-nodes






<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

## ROS Packages for TurtleBot3
|Version|Kinetic + Ubuntu Xenial|Melodic + Ubuntu Bionic|
|:---:|:---:|:---:|
|[![GitHub version](https://badge.fury.io/gh/ROBOTIS-GIT%2Fturtlebot3.svg)](https://badge.fury.io/gh/ROBOTIS-GIT%2Fturtlebot3)|[![Build Status](https://travis-ci.org/ROBOTIS-GIT/turtlebot3.svg?branch=kinetic-devel)](https://travis-ci.org/ROBOTIS-GIT/turtlebot3)|[![Build Status](https://travis-ci.org/ROBOTIS-GIT/turtlebot3.svg?branch=melodic-devel)](https://travis-ci.org/ROBOTIS-GIT/turtlebot3)|

## ROBOTIS e-Manual for TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)

## Wiki for turtlebot3 Packages
- http://wiki.ros.org/turtlebot3 (metapackage)
- http://wiki.ros.org/turtlebot3_bringup
- http://wiki.ros.org/turtlebot3_description
- http://wiki.ros.org/turtlebot3_example
- http://wiki.ros.org/turtlebot3_navigation
- http://wiki.ros.org/turtlebot3_slam
- http://wiki.ros.org/turtlebot3_teleop

## Open Source related to TurtleBot3
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_applications_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs)
- [turtlebot3_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications)
- [turtlebot3_autorace](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)
- [turtlebot3_deliver](https://github.com/ROBOTIS-GIT/turtlebot3_deliver)
- [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)
- [open_manipulator_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_msgs)
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [open_manipulator_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_simulations)
- [open_manipulator_perceptions](https://github.com/ROBOTIS-GIT/open_manipulator_perceptions)
- [open_manipulator_with_tb3_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_msgs)
- [open_manipulator_with_tb3](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3)
- [open_manipulator_with_tb3_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_simulations)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documents and Videos related to TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)
- [ROBOTIS e-Manual for OpenManipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [ROBOTIS e-Manual for Dynamixel Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)
- [Website for TurtleBot Series](http://www.turtlebot.com/)
- [e-Book for TurtleBot3](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51/)
- [Videos for TurtleBot3 ](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)
