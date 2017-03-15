/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /include/turtlebot_panorama/panorama.h
 *
 * @brief Panorama app class definition
 *
 * @date 08/01/2013
 *
 * @author Younghun Ju, Jihoon Lee and Marcus Liebhardt
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef PANORAMA_H_
#define PANORAMA_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>	
#include <sensor_msgs/Image.h>		
#include <std_msgs/Empty.h> 		
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>		
#include <geometry_msgs/Twist.h>
#include <turtlebot3_msgs/TakePanorama.h>
#include <std_srvs/Empty.h>

#include "geometry.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

namespace turtlebot_panorama
{

/**
 * The PanoApp utilises pano_ros for creating panorama pictures.
 */
class PanoApp
{
public:
  PanoApp();
  ~PanoApp();

  void init();
  void spin();

  /**
   * Additionally sends out logging information on a ROS topic
   * @param msg logging information
   */
  void log(std::string msg);

private:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  std::map<std::string, std::string> params;

  geometry_msgs::Twist cmd_vel, zero_cmd_vel;
  double snap_interval;
  double angle, last_angle, given_angle, ang_vel_cur;

  bool continuous;

  image_transport::Publisher pub_stitched;
  image_transport::Subscriber sub_camera;

  ros::ServiceServer srv_start_pano;

  // for turning the robot
  ros::Publisher pub_cmd_vel;
  // for retrieving the odometry of robot
  ros::Subscriber sub_odom;

  std::vector<cv::Mat> images_;

  /**
   * turns true, when the pano_ros action goal goes active
   */
  bool is_active;
  /**
   * Tells the pano_ros feedback callback to set is_active to true (starts rotating the robot)
   * This is necessary in order to capture the first picture at the start,
   * since it takes a while to get the first pciture from the Kinect.
   */
  bool go_active;
  /**
   * Default panorama mode used for interaction via rostopic
   */
  int default_mode;
  /**
   * Default panorama angle used for interaction via rostopic
   */
  double default_pano_angle;
  /**
   * Default snap interval used for interaction via rostopic
   */
  double default_snap_interval;
  /**
   * Default rotation velocity used for interaction via rostopic
   */
  double default_rotation_velocity;

  bool store_image;
  /**
   * Starts the creation of a panorama picture via a ROS service
   * @param request specify the details for panorama creation
   * @param response the current state of the app (started, in progress, stopped)
   * @return true, if service call was successful
   */
  bool takePanoServiceCb(turtlebot3_msgs::TakePanorama::Request& request,
                         turtlebot3_msgs::TakePanorama::Response& response);

  void snap();

  void rotate();

  bool hasReachedAngle();

  void odomCb(const nav_msgs::OdometryConstPtr& msg);

  void startPanoAction();

  void cameraImageCb(const sensor_msgs::ImageConstPtr& msg);
};

} //namespace turtlebot_panorama

#endif /* PANORAMA_H_ */
