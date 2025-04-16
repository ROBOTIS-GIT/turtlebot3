^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_navigation2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.9 (2025-04-15)
------------------
* None

2.2.8 (2025-04-11)
------------------
* None

2.2.7 (2025-03-27)
------------------
* None

2.2.6 (2025-03-24)
------------------
* Added Nav2 parameters for Jazzy
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
* rename and update nav2 params
* Contributors: Ashe Kim, Will Son

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
* Added use_sim_time parameter for rviz2 `#456 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/456>`_
* Updated map.yaml `#386 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/386>`_
* Changed package name to turtlebot3_navigation2, it supoorts Nav2 of ROS 2
* Contributors: Mikael Arguedas, Alberto Soragna, Darby Lim, Pyo

1.2.2 (2019-08-20)
------------------
* Fixed `dwa local planner params` for dwa_local_planner 1.16.2 `#415 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/415>`_
* This patch only applies to ROS 1 Melodic.
* Contributors: atinfinity, Kayman

1.2.1 (2019-08-20)
------------------
* Deleted '/' to sync tf2 `#402 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/402>`_
* Contributors: Ryan Shim, Kayman, Darby Lim

1.2.0 (2019-01-22)
------------------
* updated map.yaml `#348 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/348>`_
* added an additional argument move_forward_only to prohibit backward locomotion in navigation `#339 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/339>`_
* fixed typo `#280 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/280>`_
* Contributors: Gilbert, Darby Lim, linzhibo, oiz5201618, yoshimalucky, Pyo

1.1.0 (2018-07-23)
------------------
* added bringup to load multiple robot simply #251
* added arguments for multiple robot
* Contributors: Darby Lim, Gilbert, Pyo

1.0.0 (2018-05-29)
------------------
* modified navigation parameters
* modified control frequency and inflation radius
* modified param after experiments
* merged pull request `#202 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/202>`_ from ROBOTIS-GIT/feature-cartographer
* merged pull request `#220 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/220>`_ `#212 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/212>`_ `#200 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/200>`_ `#154 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/154>`_ `#153 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/153>`_ `#147 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/147>`_ `#146 <https://github.com/ROBOTIS-GIT/turtlebot3/issues/146>`_
* Contributors: Darby Lim, Pyo

0.2.1 (2018-03-14)
------------------
* refactoring for release
* Contributors: Pyo

0.2.0 (2018-03-12)
------------------
* added waffle pi model (urdf and gazebo)
* refactoring for release
* Contributors: Darby Lim, Pyo

0.1.6 (2017-08-14)
------------------
* updated nav param
* Contributors: Darby Lim

0.1.5 (2017-05-25)
------------------
* modified bag and map files
* Contributors: Darby Lim, Pyo

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
* Contributors: Darby Lim, Leon Jung, Pyo
