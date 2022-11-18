# TurtleBot3 Friends : Mecanum
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
