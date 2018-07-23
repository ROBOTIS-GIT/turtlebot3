^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2018-07-23)
------------------
* added bringup to load multiple robot simply #251
* added arguments for multiple robot
* added odometrySource
* modified camera topic name
* modified base_scan update_rate and add param on diff_drive #258
* modified the laser scanner update_rate in the gazebo xacro files #258
* modified origin of collision in Waffle URDF
* updated turtlebot3_diagnostic node
* updated firmware version from 1.2.0 to 1.2.2
* updated get firmware version
* updated version check function
* updated warn msg for version check
* deleted unused get_scan function #227
* Contributors: Darby Lim, Gilbert, Eduardo Avelar, shtseng, Pyo

1.0.0 (2018-05-29)
------------------
* added cartographer
* added hector mapping
* added karto SLAM
* added frontier_exploration
* added launch files to run various SLAMs
* added robot model for OpenManipulator and turtlebot3_autorace
* added exec python nodes like marker_server in catkin_install_python
* added frameName for imu on gazebo (however, there is no effect.)
* added variable to check version only once (turtlebot3_bringup)
* modified global names `#211 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/211>`_ from FurqanHabibi/fix_global_topic_name
* modified gmapping parameters
* modified navigation parameters
* modified version check and firmware version (turtlebot3_bringup)
* modified robot names
* modified range of lidar, lidar position, scan param
* modified camera position and fixed slip bug
* modified waffle_pi stl files
* modified initial value, profile function, limit velocity msg (teleop)
* merged pull request `#154 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/154>`_ `#153 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/153>`_ `#148 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/148>`_ `#147 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/147>`_ `#146 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/146>`_ `#145 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/145>`_
* Contributors: Darby Lim, Leon Jung, Gilbert, KurtE, ncnynl, FurqanHabibi, skasperski, ihadzic, Pyo

0.2.1 (2018-03-14)
------------------
* added install directory
* refactoring for release
* Contributors: Pyo

0.2.0 (2018-03-12)
------------------
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
------------------
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
------------------
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
------------------
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
