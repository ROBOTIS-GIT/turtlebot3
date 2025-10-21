^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.3 (2025-07-11)
------------------
* None

2.3.2 (2025-07-08)
------------------
* None

2.3.1 (2025-05-30)
------------------
* None

2.3.0 (2025-05-23)
------------------
* None

2.2.9 (2025-04-15)
------------------
* fixed typo error in urdf
* Contributors: Hyungyu Kim

2.2.8 (2025-04-11)
------------------
* Support flexible configuration of the frame_id used when publishing the topic
* Contributors: Hyungyu Kim

2.2.7 (2025-03-27)
------------------
* None

2.2.6 (2025-03-24)
------------------
* None

2.2.5 (2025-02-26)
------------------
* None

2.2.4 (2025-02-21)
------------------
* None

2.2.3 (2025-02-19)
------------------
* None

2.2.2 (2025-02-19)
------------------
* None

2.2.1 (2025-02-18)
------------------
* None

2.2.0 (2025-02-13)
------------------
* None

2.1.5 (2022-05-26)
------------------
* ROS 2 Humble Hawksbill supported
* Contributors: Will Son

2.1.4 (2022-02-08)
------------------
* None

2.1.3 (2021-09-02)
------------------
* ROS 2 Rolling Ridley supported
* Contributors: Will Son

2.1.2 (2021-04-07)
------------------
* None

2.1.1 (2021-01-06)
------------------
* turtlebot3.repos updated to target correct distro
* galactic-devel branch created
* Eloquent EOL
* Contributors: Ashe Kim, Will Son

2.1.0 (2020-06-22)
------------------
* ROS 2 Foxy Fitzroy supported
* ROS 2 Eloquent Elusor supported
* Contributors: Ryan, Ashe

2.0.1 (2019-09-05)
------------------
* Updated the CHANGELOG and version to release binary packages
* Modified dependency packages
* Contributors: Darby Lim, Pyo

2.0.0 (2019-08-20)
------------------
* Supported ROS 2 Dashing Diademata
* Updated the CHANGELOG and version to release binary packages
* Removed ament_export_dependency(xacro) `#462 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/462>`_
* Contributors: Ross Desmond, Darby Lim, Pyo

1.2.2 (2019-08-20)
------------------
* none

1.2.1 (2019-08-20)
------------------
* none

1.2.0 (2019-01-22)
------------------
* none

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
