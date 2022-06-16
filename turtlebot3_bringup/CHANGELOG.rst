^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.5 (2022-05-26)
------------------
* ROS2 Humble Hawksbill supported

2.1.4 (2022-02-08)
------------------
* add LDS-02 support

2.1.3 (2021-09-02)
------------------
* ROS 2 Rolling Ridley supported
* fix ambiguous ParameterValue calls (#748, #736)
* rename and update nav2 params
* modify robot_state_publisher
* Contributors: m2-farzan, David Park, Ashe Kim, Will Son

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
* Fixed ROS2 dependencies and library install `#454 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/454>`_
* Contributors: Emerson Knapp, Darby Lim, Pyo

1.2.2 (2019-08-20)
------------------
* none

1.2.1 (2019-08-20)
------------------
* Added turtlebot3_remote.launch to turtlebot3_model.launch `#389 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/389>`_
* Contributors: Jonathan Hechtbauer, Gilbert

1.2.0 (2019-01-22)
------------------
* none

1.1.0 (2018-07-23)
------------------
* added bringup to load multiple robot simply #251
* added argument about namespace
* updated turtlebot3_diagnostic node
* updated firmware version from 1.2.0 to 1.2.2
* updated get firmware version
* updated version check function
* updated warn msg for version check
* Contributors: Darby Lim, Gilbert, Pyo

1.0.0 (2018-05-29)
------------------
* added variable to check version only once
* modified firmware version
* modified global names `#211 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/211>`_ from FurqanHabibi/fix_global_topic_name
* modified turtlebot3_rpicamera.yaml gets camera_info_url value from the initialized calibration file
* deleted count version info msg
* merged pull request `#220 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/220>`_ `#212 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/212>`_ `#200 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/200>`_ `#156 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/156>`_ `#154 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/154>`_ `#153 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/153>`_ `#150 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/150>`_ `#147 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/147>`_ `#146 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/146>`_
* Contributors: Darby Lim, Leon Jung, Muhammad Furqan Habibi, Pyo

0.2.1 (2018-03-12)
------------------
* refactoring for release
* Contributors: Pyo

0.2.0 (2018-03-12)
------------------
* refactoring for release
* updated firmware version
* modified param config
* added turtlebot3_rpicamera.launch for raspberry pi camera
* added waffle pi model
* added verion check function
* added diagnostics node
* added scripts for reload rules
* Contributors: Darby Lim, Gilbert, Leon Jung, Pyo

0.1.6 (2017-08-14)
------------------
* updated model.launch
* fixed typo
* fixed xacro.py deprecation
* modified file location
* Contributors: Darby Lim, Hunter L. Allen

0.1.5 (2017-05-25)
------------------
* changed the node name from hlds_laser_publisher to turtlebot3_lds
* Contributors: Pyo

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
