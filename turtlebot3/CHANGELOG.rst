^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.5 (2022-05-26)
------------------
* ROS2 Humble Hawksbill supported
* revise ROS2 Cartographer excutable & name

2.1.4 (2022-02-08)
------------------
* add LDS-02 support

2.1.3 (2021-09-02)
------------------
* ROS 2 Rolling Ridley supported
* fix ambiguous ParameterValue calls (#748, #736)
* fix RViz2 Cartographer default config
* Contributors: m2-farzan, Will Son

2.1.2 (2021-04-07)
------------------
* use static param types for Galactic
* fix SensorState msg
* rename and update nav2 params
* modify robot_state_publisher
* fix odometry bug
* Contributors: jhbirdchoi, David Park, Ashe Kim, Will Son

2.1.1 (2021-01-06)
------------------
* Nav2 prefix filename removed
* turtlebot3.repos updated to target correct distro
* galactic-devel branch created
* Eloquent EOL
* Contributors: Ashe Kim, Will Son

2.1.0 (2020-06-22)
------------------
* ROS 2 Foxy Fitzroy supported
* ROS 2 Eloquent Elusor supported
* Enable Windows port
* Contributors: Ryan, Ashe, Sean Yen

2.0.1 (2019-09-05)
------------------
* Updated the CHANGELOG and version to release binary packages
* Added missing exec_depend for nav2_bringup
* Modified dependency packages
* Contributors: Ruffin, Darby Lim, Pyo

2.0.0 (2019-08-20)
------------------
* Supported ROS 2 Dashing Diademata
* Updated the CHANGELOG and version to release binary packages
* Removed ament_export_dependency(xacro) `#462 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/462>`_
* Added use_sim_time parameter for rviz2 `#456 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/456>`_
* Fixed ROS2 dependencies and library install `#454 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/454>`_
* Fixed turtlebot3.repos `#447 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/447>`_ `#432 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/432>`_
* Fixed scan rate to 5hz `#418 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/418>`_
* Initialized joint states variables `#411 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/411>`_
* Updated map.yaml `#386 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/386>`_
* Contributors: Matt Hansen, Alberto Soragna, Carl Delsey, sousou1, Emerson Knapp, Mikael Arguedas, Ross Desmond, Darby Lim, Pyo

1.2.2 (2019-08-20)
------------------
* Fixed `dwa local planner params` for dwa_local_planner 1.16.2 `#415 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/415>`_
* This patch only applies to ROS 1 Melodic.
* Contributors: atinfinity, Kayman

1.2.1 (2019-08-20)
------------------
* Fixed typo `#436 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/436>`_
* Fixed ROS_ASSERT bug `#416 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/416>`_
* Deleted '/' to sync tf2 `#402 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/402>`_
* Added turtlebot3_remote.launch to turtlebot3_model.launch `#389 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/389>`_
* Contributors: Jonathan Hechtbauer, Pallav Bhalla, ant, Ryan Shim, Kayman, Darby Lim, Gilbert, Pyo

1.2.0 (2019-01-22)
------------------
* changed math.ceil() operation `#373 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/373>`_
* fixed TypeError integers
* fixed read of scanned samples when there isn't 360
* updated map.yaml `#348 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/348>`_
* added an additional argument move_forward_only to prohibit backward locomotion in navigation `#339 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/339>`_
* fixed typo `#280 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/280>`_
* added windows port `#358 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/358>`_
* Contributors: Gilbert, Darby Lim, linzhibo, oiz5201618, yoshimalucky, Steven Macenski, Eduardo Avelar, Sean Yen, Pyo

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
