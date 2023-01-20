[日本語](/turtlebot3/documentation/README_tb3_pizza_jp.md) | [English](/turtlebot3/documentation/README_tb3_pizza_en.md)

# TurtleBot3 Friends: Pizza
![TB3 Pizza](/turtlebot3/documentation/tb3_pizza_bg.png)

# In Development Process

## Navigation Demo in real-world environment

| Real-world | Rviz |
|:---:|:---:|
| ![TB3 Pizza GO](/turtlebot3/documentation/gif/tb3_pizza_go_top_x2.gif) | ![TB3 Pizza GO rv](/turtlebot3/documentation/gif/tb3_pizza_go_rv_x2.gif) | 

| Real-world | Rviz |
|:---:|:---:|
|![TB3 Pizza BACK](/turtlebot3/documentation/gif/tb3_pizza_back_top_x2.gif) | ![TB3 Pizza BACK rv](/turtlebot3/documentation/gif/tb3_pizza_back_rv_x2.gif) |

## Navigation Demo in Gazebo environment

| Gazebo + Rviz | 
|:---:|
|![TB3 Pizza BACK](/turtlebot3/documentation/gif/tb3_pizza_nav_1_x5.gif) |

| Gazebo + Rviz | 
|:---:|
|![TB3 Pizza BACK](/turtlebot3/documentation/gif/tb3_pizza_nav_2_x5.gif) |

## Setup Manual（Quick Start Guide）
### 1. Environment Setup
Few configurations from TurtleBot3 are required in order to use and test the new Pizza model from the TurtleBot3 Friends family. To do so, please follow the official TurtleBot3 e-Manual 「[Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」. However, take note of the following changes that are pointed out.

- About [1.1. PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

Pizza model is right now only supported in ROS Noetic enviroment. Please, select the 「Noetic」 branch from the 「[Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」.

- Changes in [1.1.4. Install TurtleBot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-turtlebot3-packages)

Once ROS Noetic is already installed, we will install some required ROS packages from the ROBOTIS Japan official GitHub site. Please, introduce the following commands in your terminal.

```code
$ sudo apt remove ros-noetic-dynamixel-sdk
$ sudo apt remove ros-noetic-turtlebot3-msgs
$ sudo apt remove ros-noetic-turtlebot3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b noetic-jp-devel https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_jp_custom
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

- About [3.2. SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)

In the new Pizza model, NUC11 is used as main processing computer, instead the Raspbery Pi 4B that has being used in `Buger` or `Waffle Pi` models. We will omit the usual SBC setup setup. Please follow the next steps in your NUC.

1. Install Ubuntu 20.04.

2. Install ROS Noetic.

3. Install TurtleBot3 required packages as it follows

```code
$ sudo apt remove ros-noetic-dynamixel-sdk
$ sudo apt remove ros-noetic-turtlebot3-msgs
$ sudo apt remove ros-noetic-turtlebot3
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b noetic-jp-devel https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_jp_custom
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

- Changes in [3.3. OpenCR Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup)

The setup of the OpenCR is done through the NUC.

1. Connect the [OpenCR](https://emanual.robotis.com/docs/en/parts/controller/opencr10/) to te NUC through a micro USB cable.

2. Skip the installation of arm Debian packages (in NUC are not required).

3. Select ```pizza_noetic``` as the `OPENCR_MODEL`.
```code
$ export OPENCR_PORT=/dev/ttyACM0
$ export OPENCR_MODEL=pizza_noetic
$ rm -rf ./opencr_update.tar.bz2
```

4. Download the firmware and uncompress it.
```code
$ wget https://github.com/ROBOTIS-JAPAN-GIT/OpenCR_jp_custom/releases/download/v1.0.0/opencr_update_jp_custom.tar.bz2
$ tar -xvf opencr_update_jp_custom.tar.bz2 
```

5. Burn the firmware into the OpenCR.
```code
$ cd ./opencr_update
$ ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

6. If the firmware for the TurtleBot3 Pizza has been successfully burnt into the OpenCR, the following message will appear.
![TB3 Pizza OpenCR Success Output](/turtlebot3/documentation/tb3_pizza_opencr.png)


### 2. Network Environment
In usual TurtleBot3 models, Raspberri Pi has being used to distribute the main processing to the `master pc` which counts with higher processing performance. In the case of the new Pizza model, NUC is set up as the `master pc`. To connect with it, we will use ssh through a remote PC, and visualize the NUC-processed topic in Rviz. Please, follow the next steps.
> **Note**
> As NUC counts with a better processing performance than Raspberry Pi, main processes (SLAM and Navigation) are hold inside te NUC.

- **In the NUC（master PC）**
1. Check the IP address the NUC is connected to.

```code
$ ifconfig
```

Inside the 「wlp2s0」>「inet addr」, you will find an ip similar to `192.168.X.XXX`. Please, ntake a note of it.
> **Note**
> The 「X」 from the ip `192.168.X.XXX` may change depending on your network.

2. Write the ROS IP inside your 「~/.bashrc」 file.

```code ~/.bashrc
$ nano ~/.bashrc
```

Move until the last line of the document and write down the following lines. (Do not forget to change the 「X」 depending on your IP)

```
export ROS_MASTER_URI=http://192.168.X.XXX:11311
export ROS_HOSTNAME=192.168.X.XXX
```

3. Once, it has been already added to the file, press `Ctrl+s` to save and then, `Ctrl+x` to go back.

4. Lastly, apply the changes of ROS IP to take them into effect.

```code
$ source ~/.bashrc
```

- **In the Remote PC**

1. Do not forget the IP address of your NUC. (192.168.X.XXX)
1. Check the IP address of your `remote PC` using the `ifconfig` command and take note of it. (192.168.X.YYY)
1. Write the ROS IP inside your 「~/.bashrc」 file.

```code ~/.bashrc
$ nano ~/.bashrc
```

Move until the last line of the document and write down the following lines. (Do not forget to change the 「X」 and the 「Y」 depending on your IP)。

```
export ROS_MASTER_URI=http://192.168.X.XXX:11311
export ROS_HOSTNAME=192.168.X.YYY
```

4. Once, it has been already added to the file, press `Ctrl+s` to save and then, `Ctrl+x` to go back.

5. Lastly, apply the changes of ROS IP to take them into effect.

```code
$ source ~/.bashrc
```

### 3. Additional repositories setup
In the new TurtleBot3 Pizza model, 「Realsense D435」 is used as defaut camera and 「Sick Tim571」 as default LiDAR sensor. Here, we will explain about its setup.

> **Warning**
> The following steps are conducted in the NUC.

- **Camera setup**

First of all, please install the requiered packages.
- ROS Wrapper for Intel® RealSense™ Devices (Based on official website)

```code
$ sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-JAPAN-GIT/realsense-ros_jp_custom
$ cd ~/catkin_ws
$ catkin_make
```

> **Note**
> If you want to know more in detail about the [realsense-ros](https://github.com/IntelRealSense/realsense-ros), please refer to the official GitHub repository.

Now, in order to use the Realsense D435 in the Gazebo simulation as well, download the next repository.

```code
$ cd ~/catkin_ws/src
$ git clone https://github.com/pal-robotics/realsense_gazebo_plugin
$ cd ~/catkin_ws
$ catkin_make
```


- **LiDAR setup**

Download the necessary Sick Tim repository, add `udev rules` to the NUC and compile it.

```code
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-JAPAN-GIT/sick_tim_jp_custom
$ cd sick_tim
$ sudo cp debian/udev /etc/udev/rules.d/81-sick-tim3xx.rules
$ sudo udevadm control --reload-rules
$ cd ~/catkin_ws
$ cakin_make
```

> **Note**
> The main difference between the official [sick_tim](https://github.com/uos/sick_tim) is that it is also supported for Gazebo simulation.

Then, as TiM is connected to the NUC through LAN cable, IP setup is required.

First, we need to configure the IP in the NUC.
1. Take note of LiDAR IP.
> **Note**
> The default IP is set to「192.168.0.1」. If different IP is required, please refer to the SICK official website about how to update the LiDAR IP.
2. Open up 「Settings」 from Ubuntu.
1. Move onto 「Network」, and check the 「Wired」 list of connections.
1. Click on the knob button.
1. Move to IPv4. Inside 「IPv4 Method」by default「Automatic (DHCP)」is selected. Change it to「Manual」.
1. Inside 「Address」, change the 「Address」to「192.168.X.XXX」, set the 「Netmask」 to 「255.255.255.0」, and the Gateway to 「192.168.X.1」.
> **Note**
> Depending on the LiDAR IP address, 「X」 value needs to be changed. Moreover, it is important that the last 「XXX」 does not coincide with the LiDAR IP address. For example, if the LiDAR IP address is 「192.168.0.1」, you might need to choose a number between 「0~255」 apart from 「1」.

Next, we need to match the `hostname` to the LiDAR IP address in the 「sick_tim571_2050101.launch」(~/catkin_ws/src/sick_tim) launcher file.

- **Before**

```code
    <!-- Uncomment this to enable TCP instead of USB connection; 'hostname' is the host name or IP address of the laser scanner
    In cases where a race condition exists and the computer boots up before the TIM is ready, increase 'timelimit.'
         <param name="hostname" type="string" value="192.168.0.1" />
         <param name="port" type="string" value="2112" />
         <param name="timelimit" type="int" value="5" />
    -->
```

- **After**
```code
    <!-- Uncomment this to enable TCP instead of USB connection; 'hostname' is the host name or IP address of the laser scanner
    In cases where a race condition exists and the computer boots up before the TIM is ready, increase 'timelimit.' -->
         <param name="hostname" type="string" value="192.168.X.XXX" />
         <param name="port" type="string" value="2112" />
         <param name="timelimit" type="int" value="5" />
```
> **Note**
> Inside `value="192.168.X.XXX`, 「X」 value needs to be changed depending on the LiDAR IP address. Fo example, if the LiDAR IP address is 「192.168.0.1」, set the value as `value="192.168.0.1"`.（If you want to set a different IP addres, please refer to SICK official website about its process.）


## Simulation Environment（Gazebo）
TurtleBot3 Pizza counts with Gazebo support, a simulation environment which takes into account the physics of its virtual environmet. By just dowloading the following package, you will be able to use it with no problem.

```code
$ cd ~/catkin_ws/src/
$ git clone -b noetic-jp-devel https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_simulations_jp_custom
$ git clone https://github.com/robotics-upo/lightsfm
$ cd lightsfm
$ make
$ sudo make install
$ cd ~/catkin_ws && catkin_make
```


## Let's move it!

### Real robot
Congratulations! You have been able to setup your developing environment. Now, let's move the TurtleBot3 Pizza. To do so, please followg the next steps.

> **Note**
> When connecting to the NUC thorugh ssh, do not forget to use the NUC IP address (192.168.X.XXX).

0. Select the TurtleBot3 model.
```code 
$ export TURTLEBOT3_MODEL=pizza
$ export TURTLEBOT3_PLAT=false
```
> **Note**
> Everytime you open a new terminal, it is required to select the TurtleBot3 model, you would use. A part from `pizza`, you can slo try other models such as `burger`, `waffle_pi`, `big_wheel`.

1. In the **NUC**, run roscore.
```code
$ roscore
```
2. In the **NUC**, launch the Turtlebot3 Pizza.
```code
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
3. If required, in the **remote PC** launch the TeleOperation.
```code
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

- **SLAM (mapping) + Navigation**

In other to use those packages, please refer to the official e-Manual 「[SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/)」 and 「[Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/)」. The bring-up steps are the same as previous models.


### Simulation
In other to use the Gazebo simulation environment, please refer to the official e-Manual「[1.1.2. Launch Simulation World](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#launch-simulation-world)」. The bring-up stes are the same as previous models.

```code 
$ export TURTLEBOT3_MODEL=pizza
$ export TURTLEBOT3_PLAT=false
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

> **Note**
> A part from `empty_world`, you can also try different environments such as `house`, `simulation`, `stage_1`. Moreover, as ROBOTIS Japan, we have prepared a few new environments. Right now, we have `turtlebot3_jp_world_empty`, `turtlebot3_jp_world_static`, `turtlebot3_jp_world_dynamic`, `turtlebot3_jp_world_corridor_pedestrian` 4 different worlds ready to use.


| World Name | Image |
|:---:|:---:|
| turtlebot3_jp_world_static | ![TB3 static](/turtlebot3/documentation/gif/turtlebot3_jp_world_static.png) | 
| turtlebot3_jp_world_dynamic | ![TB3 dynamic](/turtlebot3/documentation/gif/turtlebot3_jp_world_dynamic.gif) |
| turtlebot3_jp_world_corridor_pedestrian | ![TB3 corridor](/turtlebot3/documentation/gif/turtlebot3_jp_world_corridor.gif) |
| turtlebot3_jp_world_corridor_pedestrian (rgbd) | ![TB3 corridor rgbd](/turtlebot3/documentation/gif/turtlebot3_jp_world_corridor_rgbd.gif) |


## Hardware Related
### Bill Of Materials（BOM）
| Item | Reference no. | Quantity | Link |
|---|---|---|---|
| Dynamixel xm540-w150-r | 902-0134-000 | 2 | [here](https://e-shop.robotis.co.jp/product.php?id=42) |
| OpenCR1.0 | 903-0257-000 | 1 | [here](https://e-shop.robotis.co.jp/product.php?id=155) |
| NUC 11 Pro Kit NUC11TNHv7 | BNUC11TNHV70000 | 1 | [here](https://www.ark-pc.co.jp/i/31400996/) |
| TiM571-2050101 | 1075091 | 1 | [here](https://www.sick.com/jp/ja/detection-and-ranging-solutions/2d-lidar/tim/tim571-2050101/p/p412444) |
| Realsense d435 | --- | 1 | [here](https://www.intelrealsense.com/depth-camera-d435/) |
| Wheel (5inch) | --- | 2 | [here]() |
| Smooth omniwheel（Φ55mm） | 4571398310089 | 2 | [here](https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=4394) |
| Aluminum frame | CAF5-2020-400 | 4 | [here](https://jp.misumi-ec.com/vona2/detail/110302683830/?PNSearch=CAF5-2020-400&HissuCode=CAF5-2020-400&searchFlow=suggest2products&Keyword=CAF5-2020-400) |
| Aluminum frame | CAF5-2020-360 | 6 | [here](https://jp.misumi-ec.com/vona2/detail/110302683830/?PNSearch=CAF5-2020-360&HissuCode=CAF5-2020-360&searchFlow=suggest2products&Keyword=CAF5-2020-360) |
| Aluminum frame | CAF5-2020-170 | 4 | [here](https://jp.misumi-ec.com/vona2/detail/110302683830/?PNSearch=CAF5-2020-170&HissuCode=CAF5-2020-170&searchFlow=suggest2products&Keyword=CAF5-2020-170) |
| Aluminum frame | CAF5-2020-100 | 5 | [here](https://jp.misumi-ec.com/vona2/detail/110302683830/?PNSearch=CAF5-2020-100&HissuCode=CAF5-2020-100&searchFlow=suggest2products&Keyword=CAF5-2020-100) |
| Hard bracket SS with turnstile | SFK-N58T | 52 | [here](https://jp.misumi-ec.com/vona2/detail/221005427845/?PNSearch=SFK-N58T&HissuCode=SFK-N58T&searchFlow=suggest2products&Keyword=SFK-N58T) |
| Makita-compatible battery BL1490 14.4V 9.0Ah 129.6Wh| BL1490 | 1 | [here](https://www.amazon.co.jp/dp/B08MHWMZ7C) |
| 18V Battery Dock | B08X73Z7RP | 1 | [here](https://www.amazon.co.jp/dp/B08X73Z7RP) |
| Needle roller thrust bearing | BA0821 | 4 | [here](https://jp.misumi-ec.com/vona2/detail/110300117970/?PNSearch=BA0821&HissuCode=BA0821&searchFlow=suggest2products&Keyword=BA0821) |
| Drawn-Cup Needle Roller Bearing | TLA810Z | 2 | [here](https://jp.misumi-ec.com/vona2/detail/221005155382/?PNSearch=TLA810Z&HissuCode=TLA810Z&searchFlow=suggest2products&Keyword=TLA810Z) |
| Metal Washer | TWSSS16-4-1  | 4 | [here](https://jp.misumi-ec.com/vona2/detail/110302677010/?PNSearch=TWSSS16-4-1&HissuCode=TWSSS16-4-1&searchFlow=suggest2products&Keyword=TWSSS16-4-1) |
| Brass Spacer | BRB-435CE | 2 | [here](https://jp.misumi-ec.com/vona2/detail/221006202724/?PNSearch=BRB-435CE&HissuCode=BRB-435CE&searchFlow=suggest2products&Keyword=BRB-435CE) |
| Hexagon socket head cap screws | CSH-SUS-M5-10 | 6 | [here](https://jp.misumi-ec.com/vona2/detail/221000551286/?PNSearch=CSH-SUS-M5-10&HissuCode=CSH-SUS-M5-10&searchFlow=suggest2products&Keyword=CSH-SUS-M5-10) |
| Hexagon socket head cap screws | CSH-SUS-M4-16 | 2 | [here](https://jp.misumi-ec.com/vona2/detail/221000551286/?PNSearch=CSH-SUS-M4-16&HissuCode=CSH-SUS-M4-16&searchFlow=suggest2products&Keyword=CSH-SUS-M4-16) |
| Hexagon socket head cap screws | SBCB3-8 | 2 | [here](https://jp.misumi-ec.com/vona2/detail/110302280450/?PNSearch=SBCB3-8&HissuCode=SBCB3-8&searchFlow=suggest2products&Keyword=SBCB3-8) |
| Hexagon socket head cap screws | CSH-SUS-M2.5-20 | 8 | [here](https://jp.misumi-ec.com/vona2/detail/221000551286/?PNSearch=CSH-SUS-M2.5-20&HissuCode=CSH-SUS-M2.5-20&searchFlow=suggest2products&Keyword=CSH-SUS-M2.5-20) |
| Hexagon socket head cap screws | CSH-SUS-M2.5-12 | 8 | [here](https://jp.misumi-ec.com/vona2/detail/221000551286/?PNSearch=CSH-SUS-M2.5-12&HissuCode=CSH-SUS-M2.5-12&searchFlow=suggest2products&Keyword=CSH-SUS-M2.5-12) |
| Hexagon socket head cap screws UNC | CSH-SUS-UNC1/4-7/16 | 8 | [here](https://jp.misumi-ec.com/vona2/detail/221000551343/?PNSearch=CSH-SUS-UNC1%2F4-7%2F16&HissuCode=CSH-SUS-UNC1%2F4-7%2F16&searchFlow=suggest2products&Keyword=CSH-SUS-UNC1%2F4-7%2F16) |
| Hexagon socket countersunk bolt | CSHCS-BR-M4-10 | 21 | [here](https://jp.misumi-ec.com/vona2/detail/221000551376/?PNSearch=CSHCS-BR-M4-10&HissuCode=CSHCS-BR-M4-10&searchFlow=suggest2products&Keyword=CSHCS-BR-M4-10) |
| Hexagon socket countersunk bolt | CSHCS-BR-M4-8 | 5 | [here](https://jp.misumi-ec.com/vona2/detail/221000551376/?PNSearch=CSHCS-BR-M4-8&HissuCode=CSHCS-BR-M4-8&searchFlow=suggest2products&Keyword=CSHCS-BR-M4-8) |
| Hexagon socket countersunk bolt | CSHCS-ST-M2.5-8 | 16 | [here](https://jp.misumi-ec.com/vona2/detail/221000551376/?PNSearch=CSHCS-ST-M2.5-8&HissuCode=CSHCS-ST-M2.5-8&searchFlow=suggest2products&Keyword=CSHCS-ST-M2.5-8) |
| Hexagon socket countersunk bolt | SHFBS3-10 | 2 | [here](https://jp.misumi-ec.com/vona2/detail/110300463610/?PNSearch=SHFBS3-10&HissuCode=SHFBS3-10&searchFlow=suggest2products&Keyword=SHFBS3-10) |
| Nut | LBNR4 | 4 | [here](https://jp.misumi-ec.com/vona2/detail/110300250540/?PNSearch=LBNR4&HissuCode=LBNR4&searchFlow=suggest2products&Keyword=LBNR4) |
| Nut | LBNR2.5 | 8 | [here](https://jp.misumi-ec.com/vona2/detail/110300250540/?PNSearch=LBNR2.5&HissuCode=LBNR2.5&searchFlow=suggest2products&Keyword=LBNR2.5) |
| Brass Knurled knob | NB-310EA-N | 4 | [here](https://jp.misumi-ec.com/vona2/detail/221006307382/?PNSearch=NB-310EA-N&HissuCode=NB-310EA-N&searchFlow=suggest2products&Keyword=NB-310EA-N) |
| SF20 T Nut SS | SFB-012  | 26 | [here](https://jp.misumi-ec.com/vona2/detail/221005423144/?PNSearch=SFB-012&HissuCode=SFB-012&searchFlow=suggest2products&Keyword=SFB-012) |
| Cushion Corner Guard | EA983FE-72 | 4 | [here](https://jp.misumi-ec.com/vona2/detail/223007688859/?PNSearch=EA983FE-72&HissuCode=EA983FE-72&searchFlow=suggest2products&Keyword=EA983FE-72) |
| --- | --- | --- | [here]() |
| --- | --- | --- | [here]() |


### 3D Model reference
If you want to know more in detail about the 3D model parts, please have a look in the [TurtleBot3 Friends: Pizza](https://cad.onshape.com/documents/9d6a0395dffef67a3c72e937/w/9be45b8110f1eab640d1cdfd/e/9a65db04c65f7adb980422c1) OnShape document.

### Model Characteristics
| Items | Pizza | Big Wheel |
|---|---|---|
| Maximum translational velocity | 0.35 m/s | 0.50 m/s |
| Maximum rotational velocity | 1.49 rad/s | 3.41 rad/s |
| Maximum payload | ? | 30kg |
| Size (L x W x H) | 434.94mm x 489.10mm x 261.54mm | 281mm x 306mm x 170.30mm |
| Weight |  |  |
| Threshold of climbing |  |  |
| Expected operating time |  |  |
| Expected charging time |  |  |
| Cumputer | NUC10i7FNHN | Raspberry Pi |
| MCP |  Intel® Core™ i7-10710U Processor (12M Cache, up to 4.70 GHz)  | 32-bit ARM Cortex®-M7 with FPU (216 MHz, 462 DMIPS) |
| Remote Controller | - | - |
| Actuator | XM540-W150 | XM430-W210 |
| LiDAR | SICK TiM571 | 360 Laser Distance Sensor LDS-01 or LDS-02 |
| Camera | Realsense D435 | Realsense D435 |
| IMU | GyroscSeveral programmable beep sequencesope 3 Axis | Gyroscope 3 Axis |
|     | Accelerometer 3 Axis | Accelerometer 3 Axis |
| Power connectors |  | 3.3V / 800mA, 5V / 4A, 12V / 1A |
| Expansion pins |  | GPIO 18 pins, Arduino 32 pin |
| Peripheral |  | UART x3, CAN x1, SPI x1, I2C x1, ADC x5, 5pin OLLO x4 |
| DYNAMIXEL ports | RS485 x 3, TTL x 3 | RS485 x 3, TTL x 3 |
| Audio | Several programmable beep sequences | Several programmable beep sequences |
| Programmable LEDs | User LED x 4 | User LED x 4 |
| Status LEDs | Board status LED x 1, Arduino LED x 1, Power LED x 1 | Board status LED x 1, Arduino LED x 1, Power LED x 1 |
| Buttons and Switches | Push buttons x 2, Reset button x 1, Dip switch x 2 | Push buttons x 2, Reset button x 1, Dip switch x 2 |
| Battery | マキタ互換バッテリー BL1490 14.4v 9Ah | Lithium polymer 11.1V 1800mAh / 19.98Wh 5C |
| PC connection | USB | USB |
| Firmware upgrade | via USB, via JTAG | via USB, via JTAG |
| Power adapter (SMPS) | Input : 100-240V, AC 50/60Hz, 1.5A @max, Output : 12V DC, 5A | Input : 100-240V, AC 50/60Hz, 1.5A @max, Output : 12V DC, 5A |


## ROBOTIS e-Manual for TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)

## Open Source related to TurtleBot3
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_jp_custom](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_jp_custom)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_simulations_jp_custom](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_simulations_jp_custom)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documents and Videos related to TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [Website for TurtleBot Series](http://www.turtlebot.com/)
- [e-Book for TurtleBot3](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51/)
- [Videos for TurtleBot3](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)
