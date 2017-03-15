cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_panorama)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp std_msgs std_srvs nav_msgs sensor_msgs geometry_msgs turtlebot3_msgs image_transport cv_bridge)
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# What other packages will need to use this package
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    DEPENDS Boost Eigen3 OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_executable(panorama src/panorama.cpp)
add_dependencies(panorama turtlebot3_msgs_gencpp)
target_link_libraries(panorama ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS panorama
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS scripts/loop.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark anything (useful) else for installation
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
