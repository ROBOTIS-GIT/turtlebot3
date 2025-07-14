^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.3 (2025-07-11)
------------------
* None

2.3.2 (2025-07-08)
------------------
* Added class member variable initialization statement in the odometry node
* Contributors: Hyungyu Kim

2.3.1 (2025-05-30)
------------------
* Deprecate ament_include_dependency usage in CMakeLists.txt
* Contributors: Hyungyu Kim

2.3.0 (2025-05-23)
------------------
* None

2.2.9 (2025-04-15)
------------------
* None

2.2.8 (2025-04-11)
------------------
* Support flexible configuration of the frame_id used when publishing the topic
* Contributors: Hyungyu Kim

2.2.7 (2025-03-27)
------------------
* None

2.2.6 (2025-03-24)
------------------
* Made it possible to choose between using the Twist type or the TwistStamped type in cmd_vel through a parameter
* Contributors: Hyungyu Kim

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
* Fixed a bug where the frame_id in the header of the odom_msg starts with a slash
* Contributors: Hyungyu Kim

2.2.1 (2025-02-18)
------------------
* Fixed a bug where the orientation in the odometry was incorrect
* Contributors: Hyungyu Kim

2.2.0 (2025-02-13)
------------------
* Fixed a bug where odometry was not resetting and an offset issue
* Contributors: Hyungyu Kim

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
* fix ambiguous ParameterValue calls (#748, #736)
* fix RViz2 Cartographer default config
* Contributors: m2-farzan, Will Son

2.1.2 (2021-04-07)
------------------
* use static param types for Galactic
* fix SensorState msg
* fix odometry bug
* Contributors: jhbirdchoi, Ashe Kim, Will Son

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
* Fixed ROS 2 dependencies and library install `#454 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/454>`_
* Fixed scan rate to 5hz `#418 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/418>`_
* Initialized joint states variables `#411 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/411>`_
* Contributors: Matt Hansen, Emerson Knapp, Darby Lim, Pyo
