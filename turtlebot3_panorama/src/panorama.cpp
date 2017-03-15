/*
 * Copyright (c) 2016, Yujin Robot, Rohan Agrawal
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
 * @file /include/turtlebot_panorama/panorama.cpp
 *
 * @brief Panorama app class and ROS node implementation
 *
 * @date 08/01/2013
 *
 * @author Younghun Ju, Jihoon Lee, Marcus Liebhardt and Rohan Agrawal
 **/

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <turtlebot_panorama/panorama.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace turtlebot_panorama
{

PanoApp::PanoApp() : nh(), priv_nh("~")
{

}

PanoApp::~PanoApp()
{

}

void PanoApp::init()
{
  //***************************
  // public API for the app
  //***************************
  srv_start_pano = priv_nh.advertiseService("take_pano", &PanoApp::takePanoServiceCb, this);

  image_transport::ImageTransport it_priv(priv_nh);
  pub_stitched = it_priv.advertise("panorama", 1, true);

  image_transport::ImageTransport it(nh);
  sub_camera = it.subscribe("/camera/rgb/image_raw", 1, &PanoApp::cameraImageCb, this);

  //***************************
  // Robot control
  //***************************
  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  sub_odom = nh.subscribe("odom", 100, &PanoApp::odomCb, this);

  cmd_vel.linear.x = 0.0f;
  cmd_vel.linear.y = 0.0f;
  cmd_vel.linear.z = 0.0f;
  cmd_vel.angular.x = 0.0f;
  cmd_vel.angular.y = 0.0f;
  cmd_vel.angular.z = 0.0f;
  zero_cmd_vel = cmd_vel;
  is_active = false;
  continuous = false;
  ang_vel_cur = 0.0;
  given_angle = 0.0;
  angle = 0.0;
  last_angle = 0.0;
}

void PanoApp::spin()
{
  ros::Rate loop_rate(10);
  double start_time;
  start_time = 0.0;
  bool take_snapshot = false;

  while (ros::ok())
  {
    if (is_active)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Degrees to go: " << radians_to_degrees(std::abs(given_angle - angle)));
      if ((given_angle - angle) <= 0.0174) // check, if target angle is reached (< 1 degree)
      {
        snap();

        pub_cmd_vel.publish(zero_cmd_vel);

        ROS_INFO("Stiching %lu images", images_.size());

        cv::Mat pano;
        cv::Stitcher stitcher = cv::Stitcher::createDefault(false);
        cv::Stitcher::Status status = stitcher.stitch(images_, pano);
        log("Finished Stiching");

        cv_bridge::CvImage cv_img;
        cv_img.image = pano;
        cv_img.encoding = "bgr8";
        cv_img.header.stamp = ros::Time::now();
        pub_stitched.publish(cv_img.toImageMsg());
        log("Publishing Completed Panorama");
        ROS_INFO("Angle: %f", angle); 
        ROS_INFO("Last Angle: %f", last_angle); 
	angle=0.0;
        last_angle=0.0;
	images_.clear();
 //       imwrite("pano.jpg", pano);
        is_active = false;
      }
      else
      {
        if (continuous) // then snap_interval is a duration
        {
	    log("Continuous Mode panorama");
            rotate();
            ros::Duration(snap_interval).sleep();
            snap();
            ROS_INFO("Angle Continuous: %f", angle); 
            ROS_INFO("Angle Given: %f", given_angle); 
        }
        else
        {
          if (hasReachedAngle())
          {
            pub_cmd_vel.publish(zero_cmd_vel); // stop before taking a snapshot
            take_snapshot = true;

          }
          if (take_snapshot)
          {
            if (std::abs(ang_vel_cur) <= 0.000001) // wait until robot has stopped
            {
              snap();
              take_snapshot = false;
            }
            else
            {
              std::stringstream ss;
              std::string str;
              ss << "Waiting for robot to stop ... (speed = " << ang_vel_cur << ")";
              str = ss.str();
              log(str);
            }
          }
          else
          {
            rotate();
          }
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}


void PanoApp::snap()
{
  log("snap");
  store_image = true;
  ros::spinOnce();
  ros::Duration(1.0).sleep();
}

void PanoApp::rotate()
{
  log("rotate");
  pub_cmd_vel.publish(cmd_vel); // rotate a bit
}

bool PanoApp::hasReachedAngle()
{
  if (angle > last_angle + degrees_to_radians(snap_interval))
  {
    last_angle = angle;
    return true;
  }
  else
  {
    return false;
  }
}

void PanoApp::odomCb(const nav_msgs::OdometryConstPtr& msg)
{
  static double heading_last = 0.0f;
  double heading = 0.0f;

  Eigen::AngleAxisf angle_axis(Eigen::Quaternionf(msg->pose.pose.orientation.w,
                                                  msg->pose.pose.orientation.x,
                                                  msg->pose.pose.orientation.y,
                                                  msg->pose.pose.orientation.z));
  Eigen::Vector3f axis = angle_axis.axis();

  if (axis(2) > 0.0)
  {
    heading = angle_axis.angle();
  }
  else if (axis(2) < 0.0)
  {
    heading = -1.0 * angle_axis.angle();
  }

  angle += std::abs(wrap_angle(heading - heading_last));
  heading_last = heading;
  ang_vel_cur = msg->twist.twist.angular.z;
}

//*************************
// Public interface
//*************************
bool PanoApp::takePanoServiceCb(turtlebot3_msgs::TakePanorama::Request& request,
                                turtlebot3_msgs::TakePanorama::Response& response)
{
  if (is_active && (request.mode == request.CONTINUOUS || request.mode == request.SNAPANDROTATE))
  {
    log("Panorama creation already in progress.");
    response.status = response.IN_PROGRESS;
  }
  else if (is_active && (request.mode == request.STOP))
  {
    is_active = false;
    log("Panorama creation stopped.");
    response.status = response.STOPPED;
    return true;
  }
  else if (!is_active && (request.mode == request.STOP))
  {
    log("No panorama creation in progress.");
    response.status = response.STOPPED;
    return true;
  }
  else
  {
    if (request.pano_angle <= 0.0)
    {
      log("Specified panorama angle is zero or negative! Panorama creation aborted.");
      return true;
    }
    else if (request.snap_interval <= 0.0)
    {
      log("Specified snapshot interval is zero or negative! Panorama creation aborted.");
      return true;
    }
    else if (request.rot_vel == 0.0)
    {
      log("Specified rotating speed is zero! Panorama creation aborted.");
      return true;
    }
    else
    {
      given_angle = degrees_to_radians(request.pano_angle);
      snap_interval = request.snap_interval;
      cmd_vel.angular.z = request.rot_vel;
    }
    if (request.mode == turtlebot3_msgs::TakePanoramaRequest::CONTINUOUS)
    {
      continuous = true;
    }
    else
    {
      continuous = false;
    }
    log("Starting panorama creation.");
    // startPanoAction();
    is_active = true;
    response.status = response.STARTED;
  }
  return true;
}


void PanoApp::cameraImageCb(const sensor_msgs::ImageConstPtr& msg)
{

  if (store_image)
  {
    std::cout << "encoding: " << msg->encoding << std::endl;
    std::cout << "is_bigendian: " << msg->is_bigendian << std::endl;


    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    images_.push_back(cv_ptr->image);
    store_image = false;
  }
  else {
    //pub_stitched.publish(msg);
  }

}

//*************
// Logging
//*************
void PanoApp::log(std::string log)
{
  std_msgs::String msg;
  msg.data = log;
  ROS_INFO_STREAM(log);
}

} //namespace turtlebot_panorama

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_panorama");

  turtlebot_panorama::PanoApp pano;
  pano.log("Panorama app starting...");
  pano.init();
  pano.log("Panorama application initialised.");
  pano.spin();
  pano.log("Bye, bye!");

  return 0;
}
