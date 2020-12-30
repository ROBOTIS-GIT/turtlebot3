^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot3_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.5 (2020-12-30)
------------------
* No Changes

1.2.4 (2020-09-29)
------------------
* Package info updated
* Contributors: Will Son

1.2.3 (2020-03-03)
------------------
* none

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
