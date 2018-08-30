/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Darby Lim */

#include <chrono>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "tf2/LinearMath/Quaternion.h"

#define LEFT  0
#define RIGHT 1

#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.160           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              0.22   // m/s   (BURGER : 0.22, WAFFLE : 0.25)
#define MAX_ANGULAR_VELOCITY             2.84   // rad/s (BURGER : 2.84, WAFFLE : 1.82)

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#define TICK2RAD  0.001533981f  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
      : Node("odometry"), init_encoder_(true)
  {
    RCLCPP_INFO(this->get_logger(), "Init Odometry publisher")

    sensor_state_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(
        "sensor_state",
        [this](turtlebot3_msgs::msg::SensorState::UniquePtr msg) {
          this->updateMotorInfo(msg->left_encoder, msg->right_encoder);
        });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        [this](sensor_msgs::msg::Imu::UniquePtr msg) {
          this->orientation_[0] = msg->orientation.w;
          this->orientation_[1] = msg->orientation.x;
          this->orientation_[2] = msg->orientation.y;
          this->orientation_[3] = msg->orientation.z;
        });

    rmw_qos_profile_t odom_qos_profile = rmw_qos_profile_sensor_data;
    
    odom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    odom_qos_profile.depth = 50;
    odom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    odom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", odom_qos_profile);

    time_priv_ = std::chrono::system_clock::now();

    auto timer_callback =
        [this]() -> void {
      std::chrono::system_clock::time_point time_now = std::chrono::system_clock::now();
      std::chrono::milliseconds diff_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - this->time_priv_);

      this->time_priv_ = time_now;

      auto odom_msg = nav_msgs::msg::Odometry();
      this->calcOdometry(odom_msg, (double)(diff_time.count()) * 0.001f);
      this->odom_pub_->publish(odom_msg);
    };
    timer_ = this->create_wall_timer(10ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr sensor_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  bool init_encoder_;

  double odom_pose_[3];
  double odom_vel_[3];

  double orientation_[4];

  int32_t last_tick_[2];
  int32_t last_diff_tick_[2];
  double last_rad_[2];
  double last_velocity_[2];

  std::chrono::system_clock::time_point time_priv_;

  void updateMotorInfo(int32_t left_tick, int32_t right_tick)
  {
    int32_t current_tick[2] = {left_tick, right_tick};
    
    if (init_encoder_)
    {
      for (uint8_t index = 0; index < 2; index++)
      {
        last_diff_tick_[index] = 0;
        last_tick_[index]      = 0;
        last_rad_[index]       = 0.0;

        last_velocity_[index]  = 0.0;
      }  

      last_tick_[LEFT] = left_tick;
      last_tick_[RIGHT] = right_tick;

      init_encoder_ = false;
      return;
    }

    for (uint8_t index = 0; index < 2; index++)
    {
      last_diff_tick_[index] = current_tick[index] - last_tick_[index];
      last_rad_[index]      += (TICK2RAD * (double)(last_diff_tick_[index]));
      last_tick_[index]      = current_tick[index];
    }
  }

  bool calcOdometry(nav_msgs::msg::Odometry &odom, double diff_time)
  {
    // float* orientation;
    double wheel_l, wheel_r;      // rotation value of wheel [rad]
    double delta_s, theta, delta_theta;
    static double last_theta = 0.0;
    double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
    double step_time;

    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = theta = 0.0;
    v = w = 0.0;
    step_time = 0.0;

    step_time = diff_time;

    if (step_time == 0)
      return false;

    wheel_l = TICK2RAD * (double)(last_diff_tick_[LEFT]);
    wheel_r = TICK2RAD * (double)(last_diff_tick_[RIGHT]);

    if (std::isnan(wheel_l))
      wheel_l = 0.0;

    if (std::isnan(wheel_r))
      wheel_r = 0.0;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
    // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
    // orientation = &orientation_;
    theta       = atan2f(orientation_[1]*orientation_[2] + orientation_[0]*orientation_[3], 
                  0.5f - orientation_[2]*orientation_[2] - orientation_[3]*orientation_[3]);

    delta_theta = theta - last_theta;

    // compute odometric pose
    odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
    odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
    odom_pose_[2] += delta_theta;

    // compute odometric instantaneouse velocity

    v = delta_s / step_time;
    w = delta_theta / step_time;

    odom_vel_[0] = v;
    odom_vel_[1] = 0.0;
    odom_vel_[2] = w;

    last_velocity_[LEFT]  = wheel_l / step_time;
    last_velocity_[RIGHT] = wheel_r / step_time;
    last_theta = theta;


    auto now = rclcpp::Clock().now();

    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";
    odom.header.stamp = now;

    odom.pose.pose.position.x = odom_pose_[0];
    odom.pose.pose.position.y = odom_pose_[1];
    odom.pose.pose.position.z = 0;

    // Ref : https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code

    // double cy = cos(odom_pose_[2] * 0.5);
    // double sy = sin(odom_pose_[2] * 0.5);
    // double cr = cos(0.0 * 0.5);
    // double sr = sin(0.0 * 0.5);
    // double cp = cos(0.0 * 0.5);
    // double sp = sin(0.0 * 0.5);

    // odom.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    // odom.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    // odom.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    // odom.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;

    // odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_pose_[2]);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x  = odom_vel_[0];
    odom.twist.twist.angular.z = odom_vel_[2];

    return true;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}