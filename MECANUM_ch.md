# TurtleBot3 Friends : Mecanum  

**Mecanum W210 :** linear : 0.40 | angular : 2.0

**Mecanum W350 :** linear : 0.24 | angular : 1.2

## 快速安裝手冊

### 1. 環境設定
為了測試TurtleBot Friends系列的mecanum模型，需要一些配置。  
因此，請遵照TurtleBot3官方電子手冊「[快速入門指南](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」進行安裝並留意下列步驟更改事項。

- [1.1. PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)，mecanum使用Raspberry Pi 4B作為車體主機，目前僅於ROS Noetic版本運行，請於「[快速入門指南](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)」中上排工具列選擇「**Noetic分支**」進行安裝。

- 更改步驟 [1.1.4. Install TurtelBot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-turtlebot3-packages)

安裝完ROS Noetic後，請輸入下列指令安裝mecanum相關ROS packages。
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

- 更改步驟 [3.2.2. Download TurtleBot3 SBC Image](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#download-turtlebot3-sbc-image-2)

使用我們為mecanum提供的映象檔。
> [**Download** `Raspberry Pi 4B (2GB of 4GB)` ROS Noetic image](https://mega.nz/file/MI0HXSjS#9mXlbcwk5lk_4uTEhls1XlHFqCEaI_y4SBJ7SBCc1x8)
>   
> 請注意，此映像檔可能與 Raspberry Pi 4B with 8GM RAM 不相容。

- 更改步驟 [3.3. OpenCR Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/)

> comming soon...

### 2. 網路設定

我們使用Raspberry Pi 作為WiFi熱點。請使用PC進行連接。
> ssid : **TurtleBot3**  
> password : **turtlebot**

1. PC連接後，檢查IP位址。
```
$ ifconfig
```
可以看到類似 ```10.42.0.XXX``` 的IP型態，請記錄下來。

2. 修改PC的「~/.bashrc」檔案。
```
$ nano ~/.bashrc
```
透過使用快捷鍵 ```alt+/``` 幫助您移動到文件最底部。並寫下下列訊息
```
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_HOSTNAME=10.42.0.XXX
```
3. 確認修改完畢後使用快捷鍵 ```ctrl+s``` 儲存以及快捷鍵 ```ctrl+x``` 離開。
4. 最後，輸入指令重新載入配置。
```
$ source ~/.bashrc
```

## 如何運作
- **開機**  
1. 於**PC端**，使用指令遠端至Raspberry Pi。
> default password : **turtlebot**
```
$ ssh ubuntu@10.42.0.1
$ robot
```

- **基本遙控**
1. 於**PC端**，執行遙控範例。
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

- **SLAM (mapping)**
1. 於**PC端**，執行SLAM建圖。
```
$ roslaunch turtlebot3_slam turtlebot3_slam.launch
```

- **Navigation**
1. 於**PC端**，執行Navigation。
```
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```
  
其他相關應用請參閱 [官方電子手冊](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)。
