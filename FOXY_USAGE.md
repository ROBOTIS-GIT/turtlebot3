# TurtleBot3 Ubuntu 20.04 Foxy Usage

## Install
```sh
[RemotePC]
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b feature-foxy-update
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b ros2
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b feature-foxy-update

[SBC]
Follow the instruction at this [link](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/)
```

## Run
1) bringup
[SBC]
$ sudo apt install ros-dashing-rmw_cyclonedds_cpp
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$ cd ~/colcon_ws && colcon build && source ~/.bashrc
$ ros2 launch turtlebot3_bringup robot.launch.py

2) simulator
[RemotePC]
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

3) teleop
$ ros2 run turtlebot3_teleop teleop_keyboard

4) slam
$ ros2 launch turtlebot3_cartographer cartographer.launch.py
(issue1: odom and base_footprint are not connected as libgazebo_ros_diff_drive.so does not work for foxy yet(?)) 

5) navigation
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py 
(issue1: nav2 for Foxy has not been released yet)
(issue2: nav2_map_server from the current 'master' branch does not publish /map) 
