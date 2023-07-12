# TurtleBot3 Friends : Mecanum  
model name according to XM430's type.

- **Mecanum W210**
  
  linear : 0.40 | angular : 2.0

- **Mecanum W350**
  
  linear : 0.24 | angular : 1.2

## Setup Manual ( Quick Start Guide )
### 1. Environment Setup
Few configurations from TurtleBot3 are required in order to use and test the **mecanum** model from the Turtlebot3 Friends family. To do so, please follow the official TurtleBot3 e-Manual
「[Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」.
However, take note of the following changes that are pointed out.
- About [1.1. PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) mecanum model uses Raspberry Pi 4B as SBS. Right now, it is only supported in **ROS Noetic** environment. Please, **Select the 「Noetic」 branch** from the 「[Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」.
- Changes in [1.1.4. Install TurtelBot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-turtlebot3-packages)
Once ROS Noetic is already installed, we will install some required ROS packages from the custom github site. Please, introduce the following commands in your terminal.
```code
$ sudo apt remove ros-noetic-turltebot3-mgs
$ sudo apt remove ros-noetic-trutlebto3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/zhl017/turtlebot3_idminer_custom
$ git clone https://github.com/zhl017/turtlebot3_msgs_idminer_custom
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Set the default TURTLEBOT3_MODEL name to your model. Enter the below command to a terminal.
- In case of `mecanum w210`
```code
$ echo "export TURTLEBOT3_MODEL=mecanum" >> ~/.bashrc
$ echo "export MECANUM_TYPE=w210 >> ~/.bashrc
$ source ~/.bashrc
```
- In case of `mecanum w350`
```code
$ echo "export TURTLEBOT3_MODEL=mecanum" >> ~/.bashrc
$ echo "export MECANUM_TYPE=w350 >> ~/.bashrc
$ source ~/.bashrc
```


- Changes in [3.2.2. Download TurtleBot3 SBC Image](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#download-turtlebot3-sbc-image-2)
We provide custom image for mecanum.
> [**Download** `Raspberry Pi 4B (2GB of 4GB)` ROS Noetic image](https://mega.nz/file/MI0HXSjS#9mXlbcwk5lk_4uTEhls1XlHFqCEaI_y4SBJ7SBCc1x8)
>   
> Please note that this image may not compatible with Raspberry Pi 4B with 8GM RAM.
- Changes in [3.3. OpenCR Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/)
> comming soon...
### 2. Network Environment
We uses Raspberry Pi 4B WiFi hostpot. Please, connect with your PC.
> ssid : **TurtleBot3**  
> password : **turtlebot**
1. Check the IP address the remote PC is connected to.
```
$ ifconfig
```
you will find an ip similar to ```10.42.0.XXX```. Please, take a note of it.
2. Write the ROS IP inside your 「~/.bashrc」 file.
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
## Let's Move
- **Bringup**  
1. In **remote PC**, connect to Raspberry Pi with its IP address.
> default password : **turtlebot**
```
$ ssh ubuntu@10.42.0.1
$ robot
```
- **Basic Operation**
1. In the **remote PC**, launch the teleopeartion.
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

- **SLAM (mapping)**
1. In the **remote PC**, launch the slam.
```
$ roslaunch turtlebot3_slam turtlebot3_slam.launch
```

- **Navigation**
1. In the **remote PC**, launch the navigation.
```
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

In other to use other packages, please refer to the official [e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). The bringup steps are the same as previous models.
