# IDMINER CUSTOM ITEM
## New Model : Mecanum ( lin : 0.4 / ang : 2.0 )

### PC setup

- Install ROS Noetic please follow [Noetic] branch from the [Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

- Changes in [1.1.4. Install TurtelBot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-turtlebot3-packages)

Once ROS Noetic is already installed, we will install some required ROS packages from custom github site. Introduce the following cmomands in your terminal.


```code
$ sudo apt remove ros-noetic-turltebot3-mgs
$ sudo apt remove ros-noetic-trutlebto3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/zhl017/turtlebot3_idminer_custom
$ git clone https://github.com/zhl017/turtlebot3_msgs_idminer_custom
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ echo "export TURTLEBOT3_MODEL=mecanum" >> ~/.bashrc
$ source ~/.bashrc
```

### Network Environment

We uses Raspberry Pi 4B WiFi hostpot, please connect it.
- ssid : TurtleBot3
- password : turtlebot

1. Check the IP address the remote PC is connected to.
```
$ ifconfig
```
you will find an ip similar to ```10.42.0.XXX```. Please, take a note of it.

2. Write the ROS IP inside your [~/.bashrc] file.
```
$ nano ~/.bashrc
```
Move until the last line ```alt+/``` of the document and write down the following lines.
```
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_HOSTNAME=10.42.0.XXX
```
3. Once, it has been already added to the file, press ```ctrl+s``` to save and then, ```ctrl+x``` to go back.
4. Lastly, apply the changes of ROS IP to take them into effect.
```
$ source ~/.bashrc
```

### Bringup the TurtleBot3 Mecanum

1. Open a new terminal from remote PC with ```ctrl+alt+t``` and connect to Raspberry Pi with its IP address.
> default password is turtlebot
```
$ ssh ubuntu@10.42.0.1
```
2. Start TurtleBot3 Mecanum applications.
```
$ robot
```



# TurtleBot3
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

[![kinetic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3/workflows/kinetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3/tree/kinetic-devel)

[![melodic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3/workflows/melodic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3/tree/melodic-devel)

[![noetic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3/workflows/noetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3/tree/noetic-devel)

[![dashing-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3/workflows/dashing-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3/tree/dashing-devel)

[![foxy-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3/workflows/foxy-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel)

[![galactic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3/workflows/galactic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3/tree/galactic-devel)

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
- [turtlebot3_manipulation](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git)
- [turtlebot3_manipulation_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git)
- [open_manipulator_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_msgs)
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [open_manipulator_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_simulations)
- [open_manipulator_perceptions](https://github.com/ROBOTIS-GIT/open_manipulator_perceptions)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documents and Videos related to TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)
- [ROBOTIS e-Manual for OpenManipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [Website for TurtleBot Series](http://www.turtlebot.com/)
- [e-Book for TurtleBot3](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51/)
- [Videos for TurtleBot3 ](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)
