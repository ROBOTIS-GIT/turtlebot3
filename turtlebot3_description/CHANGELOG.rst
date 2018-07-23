^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2018-07-23)
------------------
* added bringup to load multiple robot simply #251
* added odometrySource
* modified camera topic name
* modified base_scan update_rate and add param on diff_drive #258
* modified the laser scanner update_rate in the gazebo xacro files #258
* modified origin of collision in Waffle URDF
* Contributors: Darby Lim, Gilbert, shtseng, Pyo

1.0.0 (2018-05-29)
------------------
* added frameName for imu on gazebo (however, there is no effect.)
* modified robot names
* modified range of lidar, lidar position, scan param
* modified camera position and fixed slip bug
* modified waffle_pi stl files
* merged pull request `#220 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/220>`_ `#212 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/212>`_ `#200 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/200>`_ `#155 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/155>`_ `#154 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/154>`_ `#153 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/153>`_ `#147 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/147>`_ `#146 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/146>`_
* Contributors: Darby Lim, Pyo

0.2.1 (2018-03-14)
------------------
* refactoring for release
* Contributors: Pyo

0.2.0 (2018-03-12)
------------------
* added waffle pi model (urdf and gazebo)
* modified topic of gazebo plugin 
* refactoring for release
* modified r200 tf tree
* modified gazebo imu link
* Contributors: Darby Lim, Pyo

0.1.6 (2017-08-14)
------------------
* modified models
* fixed xacro.py deprecation
* updated IMU link
* updated gazebo config
* Contributors: Darby Lim, Hunter L. Allen

0.1.5 (2017-05-25)
------------------
* updated turtlebot3 waffle URDF
* Contributors: Darby Lim

0.1.4 (2017-05-23)
------------------
* modified launch file name
* added teleop package
* Contributors: Darby Lim

0.1.3 (2017-04-24)
------------------
* modified the package information for release
* modified SLAM param
* modified the description, authors, depend option and delete the core package
* modified the turtlebot bringup files
* modified pkg setting for turtlebot3_core
* modified the navigation package and turtlebot3 node for demo
* modified the wheel speed gain
* added Intel RealSense R200
* added LDS sensor
* Contributors: Darby Lim, Pyo
