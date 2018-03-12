^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2018-03-12)
-----------
* added turtlebot3_rpicamera.launch for raspberry pi camera
* added waffle pi model (urdf and gazebo)
* added verion check function
* added diagnostics node
* added scripts for reload rules
* added example package
* modified firmware version
* modified param config
* modified topic of gazebo plugin
* modified r200 tf tree
* modified gazebo imu link
* removed the large bag file and added download command from other site
* refactoring for release
* Contributors: Darby Lim, Gilbert, Leon Jung, Pyo

0.1.6 (2017-08-14)
-----------
* fixed typo
* fixed xacro.py deprecation
* modified file location
* updated nav param
* updated SLAM param
* updated model.launch
* updated IMU link
* updated gazebo config
* Contributors: Darby Lim, Hunter L. Allen

0.1.5 (2017-05-25)
-----------
* updated turtlebot3 waffle URDF
* changed the node name from hlds_laser_publisher to turtlebot3_lds
* modified bag and map files
* added SLAM bag file
* Contributors: Darby Lim, Pyo

0.1.4 (2017-05-23)
------------------
* modified launch file name
* added teleop package
* Contributors: Darby Lim

0.1.3 (2017-04-24)
-----------
* Detached turtlebot3_msgs package from turtlebot3 package for uploading to rosdistro
* modified the package information for release
* modified SLAM param
* modified the description, authors, depend option and delete the core package
* modified the turtlebot bringup files
* modified pkg setting for turtlebot3_core
* modified the navigation package and turtlebot3 node for demo
* modified the wheel speed gain
* added Intel RealSense R200
* added LDS sensor
* Contributors: Darby Lim, Leon Jung, Pyo
