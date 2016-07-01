#include <turtlebot3_node/turtlebot3_node.h>

using namespace turtlebot3;

Turtlebot3::Turtlebot3()
: nh_priv_("~"),
  wheel_radius_(0.033),
  wheel_separation(0.15),
  encoder_min_(-2147483648),
  encoder_max_(2147483648),
  tick_to_rad_(0.00153589f),
  init_left_encoder_(false),
  init_right_encoder_(false),
  last_rad_left_(0),
  last_rad_right_(0),
  last_velocity_left_(0),
  last_velocity_right_(0),
  last_tick_left_(0),
  last_tick_right_(0),
  last_diff_tick_left_(0),
  last_diff_tick_right_(0),
  is_debug_(false)
{
  //Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);

  this->wheel_speed_cmd[LEFT] = 0.0;
  this->wheel_speed_cmd[RIGHT] = 0.0;

  encoder_low_  = ((encoder_max_ - encoder_min_) * 0.3) + encoder_min_;
  encoder_high_ = ((encoder_max_ - encoder_min_) * 0.7) + encoder_min_;

  double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
  memcpy(&(this->odom.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(this->odom.twist.covariance),pcov,sizeof(double)*36);

  // joint states
  nh_.param("wheel_left_joint_name",this->wheel_joint_name[LEFT], std::string("wheel_left_joint"));
  nh_.param("wheel_right_joint_name",this->wheel_joint_name[RIGHT], std::string("wheel_right_joint"));
  nh_.param("cmd_vel_timeout",this->cmd_vel_timeout, 1.0);
  this->cmd_vel_timeout = 1.0;

//  this->motor_enabled = true;

  this->joint_states.header.frame_id = "Joint States";
  this->joint_states.name.push_back(wheel_joint_name[LEFT]);
  this->joint_states.name.push_back(wheel_joint_name[RIGHT]);
  this->joint_states.position.resize(2,0.0);
  this->joint_states.velocity.resize(2,0.0);
  this->joint_states.effort.resize(2,0.0);

  // odometry
  nh_.param("odom_frame",this->odom.header.frame_id,std::string("odom"));
  nh_.param("base_frame",this->odom.child_frame_id,std::string("base_footprint"));

  this->odom_pose[0] = 0;
  this->odom_pose[1] = 0;
  this->odom_pose[2] = 0;

  this->prev_update_time = ros::Time::now();

  //Init target name
  ROS_ASSERT(initTurtlebot3(nh_));
}

Turtlebot3::~Turtlebot3()
{
  ROS_ASSERT(shutdownTurtlebot3());
}

bool Turtlebot3::shutdownTurtlebot3(void)
{
  return true;
}

bool Turtlebot3::initTurtlebot3(ros::NodeHandle& nh)
{
  // initialize publishers
  advertiseTopics(nh);

  // initialize subscribers
  subscribeTopics(nh);

  this->prev_update_time = ros::Time::now();
  return true;
}


void Turtlebot3::advertiseTopics(ros::NodeHandle& nh)
{
  // turtlebot required
  this->publisher["joint_states"]  = nh.advertise<sensor_msgs::JointState>("joint_states",100);

  // odometry
  this->publisher["odom"] = nh.advertise<nav_msgs::Odometry>("odom",100);

  // dxl speed
  this->publisher["left_wheel_speed"]  = nh.advertise<geometry_msgs::Twist>("left_wheel_speed",100);
  this->publisher["right_wheel_speed"] = nh.advertise<geometry_msgs::Twist>("right_wheel_speed",100);
}

void Turtlebot3::subscribeTopics(ros::NodeHandle& nh)
{
  this->subscriber["velocity"] = nh.subscribe("cmd_vel", 10, &Turtlebot3::subscribeVelocityCommand, this);
  this->subscriber["left_encoder"]  = nh.subscribe("left_wheel_position", 10, &Turtlebot3::subscribeLeftEncoder, this);
  this->subscriber["right_encoder"] = nh.subscribe("right_wheel_position", 10, &Turtlebot3::subscribeRightEncoder, this);
}

void Turtlebot3::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
  this->last_cmd_vel_time = ros::Time::now();
  this->wheel_speed_cmd[LEFT]  = msg->linear.x - (msg->angular.z * this->wheel_separation / 2);
  this->wheel_speed_cmd[RIGHT] = msg->linear.x + (msg->angular.z * this->wheel_separation / 2);
}

void Turtlebot3::subscribeLeftEncoder(const std_msgs::Int32ConstPtr left_encoder)
{
  int32_t current_tick = left_encoder->data;

  if (!init_left_encoder_)
  {
    last_tick_left_    = current_tick;
    init_left_encoder_ = true;
  }

  last_diff_tick_left_ = current_tick - last_tick_left_;
  last_tick_left_ = current_tick;
  last_rad_left_ += tick_to_rad_ * (double)last_diff_tick_left_;

  ROS_INFO_STREAM("Left : " << last_tick_left_ << "," << last_diff_tick_left_ << "," << last_rad_left_);
}

void Turtlebot3::subscribeRightEncoder(const std_msgs::Int32ConstPtr right_encoder)
{
  int32_t current_tick = right_encoder->data;

  if (!init_right_encoder_)
  {
    last_tick_right_    = current_tick;
    init_right_encoder_ = true;
  }

  last_diff_tick_right_ = last_tick_right_ - current_tick;
  last_tick_right_ = current_tick;
  last_rad_right_ += tick_to_rad_ * (double)last_diff_tick_right_;

  ROS_INFO_STREAM("Right : " << last_tick_right_ << "," << last_diff_tick_right_ << "," << last_rad_right_);
}

void Turtlebot3::updateOdometry(ros::Duration step_time)
{
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double v, w;             // v = translational velocity [m/s], w = rotational velocity [rad/s]
  wheel_l = wheel_r = 0.0;
  v = w = 0.0;

  wheel_l = tick_to_rad_ * (double)last_diff_tick_left_ / 3;
  wheel_r = tick_to_rad_ * (double)last_diff_tick_right_ / 3;

  ROS_INFO_STREAM("wheel_l = " << wheel_l);
  ROS_INFO_STREAM("wheel_r = " << wheel_r);

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  v = this->wheel_radius_ * (wheel_r + wheel_l) / 2 / step_time.toSec();
  w = this->wheel_radius_ * (wheel_r - wheel_l) / this->wheel_separation / step_time.toSec();

  ROS_INFO_STREAM("step_time = " << step_time.toSec());
  ROS_INFO_STREAM("v = " << v);
  ROS_INFO_STREAM("w = " << w);

  if (step_time.toSec() != this->prev_update_time.toSec())
  {
    last_velocity_left_  = wheel_l / step_time.toSec();
    last_velocity_right_ = wheel_r / step_time.toSec();
  }

  // compute odometric pose
  this->odom_pose[0] += v * step_time.toSec() * cos(this->odom_pose[2] + (w * step_time.toSec() / 2));
  this->odom_pose[1] += v * step_time.toSec() * sin(this->odom_pose[2] + (w * step_time.toSec() / 2));
  this->odom_pose[2] += w * step_time.toSec();

  // compute odometric instantaneouse velocity
  this->odom_vel[0] = v;
  this->odom_vel[1] = 0.0;
  this->odom_vel[2] = w;

  this->odom.pose.pose.position.x = this->odom_pose[0];
  this->odom.pose.pose.position.y = this->odom_pose[1];
  this->odom.pose.pose.position.z = 0;
  this->odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->odom_pose[2]);

  // We should update the twist of the odometry
  this->odom.twist.twist.linear.x  = this->odom_vel[0];
  this->odom.twist.twist.angular.z = this->odom_vel[2];
}

void Turtlebot3::updateJoint(ros::Duration step_time)
{
  this->joint_states.position[LEFT]  = last_rad_left_;
  this->joint_states.position[RIGHT] = last_rad_right_;

  this->joint_states.velocity[LEFT]  = last_velocity_left_;
  this->joint_states.velocity[RIGHT] = last_velocity_right_;
//  double v,w;
//  v = this->wheel_speed_cmd[LEFT];
//  w = v / this->wheel_radius_;
//  this->joint_states.velocity[LEFT] = w;
//  this->joint_states.position[LEFT]= this->joint_states.position[LEFT] + w * step_time.toSec();

//  v = this->wheel_speed_cmd[RIGHT];
//  w = v / this->wheel_radius_;
//  this->joint_states.velocity[RIGHT] = w;
//  this->joint_states.position[RIGHT]= this->joint_states.position[RIGHT] + w * step_time.toSec();
}

void Turtlebot3::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = this->odom.header;
  odom_tf.child_frame_id = this->odom.child_frame_id;
  odom_tf.transform.translation.x = this->odom.pose.pose.position.x;
  odom_tf.transform.translation.y = this->odom.pose.pose.position.y;
  odom_tf.transform.translation.z = this->odom.pose.pose.position.z;
  odom_tf.transform.rotation = this->odom.pose.pose.orientation;
}

void Turtlebot3::update(void)
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - this->prev_update_time;
  this->prev_update_time = time_now;

  // zero-ing after timeout
  if(((time_now - this->last_cmd_vel_time).toSec() > this->cmd_vel_timeout))
  {
    this->wheel_speed_cmd[LEFT]  = 0.0;
    this->wheel_speed_cmd[RIGHT] = 0.0;
  }

  // publish speed cmd
  this->wheel_speed[LEFT].linear.x  = this->wheel_speed_cmd[LEFT];
  this->wheel_speed[RIGHT].linear.x = this->wheel_speed_cmd[RIGHT];
  this->publisher["left_wheel_speed"].publish(this->wheel_speed[LEFT]);
  this->publisher["right_wheel_speed"].publish(this->wheel_speed[RIGHT]);

  // odom
  updateOdometry(step_time);
  this->odom.header.stamp = time_now;
  this->publisher["odom"].publish(this->odom);

  // joint_states
  updateJoint(step_time);
  this->joint_states.header.stamp = time_now;
  this->publisher["joint_states"].publish(this->joint_states);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  this->tf_broadcaster.sendTransform(odom_tf);
}


int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "turtlebot3_node");
  turtlebot3::Turtlebot3 tb3;
  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    tb3.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
