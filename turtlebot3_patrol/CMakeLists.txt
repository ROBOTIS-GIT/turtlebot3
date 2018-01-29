cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_patrol)

find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib
    actionlib_msgs
    message_generation
    geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)



add_action_files(FILES turtlebot3.action)

generate_messages(DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs  
)

catkin_package(
  LIBRARIES turtlebot3_patrol
  CATKIN_DEPENDS
  std_msgs
  message_runtime
  rospy 
  actionlib
  actionlib_msgs
  geometry_msgs
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

install(PROGRAMS
  scripts/turtlebot3_client.py
  scripts/turtlebot3_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

