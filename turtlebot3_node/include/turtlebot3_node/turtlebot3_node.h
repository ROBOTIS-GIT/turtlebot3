#ifndef _TURTLEBOT3_NODE_H_
#define _TURTLEBOT3_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <turtlebot3_msgs/DynamixelFeedback.h>

namespace turtlebot3
{
enum
{
  LEFT=0,
  RIGHT=1
};

class Turtlebot3
{
 public:
  Turtlebot3();
  ~Turtlebot3();
  bool initTurtlebot3(ros::NodeHandle& nh);
  void update(void);

  // variables
  sensor_msgs::JointState joint_states;
  nav_msgs::Odometry odom;
  geometry_msgs::Twist wheel_speed[2];
  float odom_pose[3];
  float odom_vel[3];
  double pose_cov[36];
  std::string wheel_joint_name[2];
  double wheel_speed_cmd[2];
  double wheel_separation;
  double wheel_radius_;
  bool motor_enabled;
  double cmd_vel_timeout;
  int32_t encoder_min_;
  int32_t encoder_max_;
  int32_t encoder_low_;
  int32_t encoder_high_;
  int32_t prev_left_encoder_;
  int32_t prev_right_encoder_;
  int32_t left_multiplication_;
  int32_t right_multiplication_;
  int32_t left_encoder_;
  int32_t right_encoder_;
  const double tick_to_rad_;
  bool init_left_encoder_;
  bool init_right_encoder_;
  int32_t last_diff_tick_left_;
  int32_t last_diff_tick_right_;
  int32_t last_tick_left_;
  int32_t last_tick_right_;
  int32_t last_realtime_tick_left_;
  int32_t last_realtime_tick_right_;
  int32_t last_diff_realtime_tick_left_;
  int32_t last_diff_realtime_tick_right_;
  double last_rad_left_;
  double last_rad_right_;
  double last_velocity_left_;
  double last_velocity_right_;

 private:
  bool shutdownTurtlebot3(void);
  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg);
  void subscribeEncoder(const turtlebot3_msgs::DynamixelFeedbackConstPtr encoder);
  bool updateOdometry(double diff_time);
  void updateJoint(void);
  void updateTF(geometry_msgs::TransformStamped& odom_tf);

  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  std::string name;
  ros::Time last_cmd_vel_time;
  ros::Time prev_update_time;
  // ROS TF
  tf::TransformBroadcaster tf_broadcaster;
  // ROS publishers
  std::map<std::string,ros::Publisher> publisher;
  // ROS subscribers
  std::map<std::string,ros::Subscriber> subscriber;
};
}
#endif
