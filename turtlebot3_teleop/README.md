Install
---
Install the dependent packages.

Install the joy package.
```sh
sudo apt-get install ros-kinetic-joy
```

Install the Dual Shock 4 driver. 
https://github.com/chrippa/ds4drv

Build this package

for catkin_make user
```sh
cd $HOME/your_catkin_ws
catkin make
```

OR

for catkin tools user
```sh
cd $HOME/your_catkin_ws
catkin build turtlebot3_teleop
```

---

Test
---
How to test the ps4 teleop.

Pair the joy stick with your PC. 
See: https://github.com/chrippa/ds4drv

Launch the teleop node.
```sh
roslaunch turtlebot3_teleop ps4_teleop.launch
```