#ifndef _TURTLEBOT3_NODE_H_
#define _TURTLEBOT3_NODE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

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
  bool update();


  // variables
  sensor_msgs::JointState         joint_states;
  nav_msgs::Odometry              odom;
  geometry_msgs::Twist            wheel_speed[2];
  float odom_pose[3];
  float odom_vel[3];
  double pose_cov[36];

  std::string wheel_joint_name[2];
  float wheel_speed_cmd[2];
  float wheel_separation;
  float wheel_diameter;

  bool motor_enabled;
  double cmd_vel_timeout;

//      sensor_msgs::Imu                imu_data;

 private:
  bool shutdownTurtlebot3();
  // private functions
  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);
//  void publishVersionInfoOnce();
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg);

  void updateJoint(unsigned int index,double& w, ros::Duration step_time);
  void updateOdometry(double w_left,double w_right, ros::Duration step_time);
  void updateTF(geometry_msgs::TransformStamped& odom_tf);

  ///////////////////////////
  // Variables
  //////////////////////////
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  std::string name;
  ros::Time last_cmd_vel_time;
  ros::Time prev_update_time;

  // version_info, joint_states
  std::map<std::string,ros::Publisher> publisher;
  // button, bumper, cliff, wheel_drop, power_system, digital_input, robot_state
//      std::map<std::string,ros::Publisher> event_publisher;
  // sensor_core, dock_ir, imu_data
//      std::map<std::string,ros::Publisher> sensor_publisher;
  // no debug publisher
  tf::TransformBroadcaster tf_broadcaster;

  // command subscribers
  std::map<std::string,ros::Subscriber> subscriber;
};
}
#endif
