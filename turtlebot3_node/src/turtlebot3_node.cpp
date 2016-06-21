#include <turtlebot3_node/turtlebot3_node.h>
#include <tf/transform_datatypes.h>

using namespace turtlebot3;

Turtlebot3::Turtlebot3()
: nh_priv_("~"),
  wheel_diameter(0.066),
  wheel_separation(0.15),
  is_debug_(false)
{
  //Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);

  this->wheel_speed_cmd[LEFT] = 0.0;
  this->wheel_speed_cmd[RIGHT] = 0.0;

  // using the same values as in kobuki_node
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
  nh_.param("cmd_vel_timeout",this->cmd_vel_timeout, 0.6);
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

bool Turtlebot3::shutdownTurtlebot3()
{
  return true;
}

bool Turtlebot3::initTurtlebot3(ros::NodeHandle& nh)
{
  // initialize publishers
  advertiseTopics(nh);

  // initialize subscribers
  subscribeTopics(nh);

//  publishVersionInfoOnce();

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
  this->publisher["left_wheel_speed"] = nh.advertise<geometry_msgs::Twist>("left_wheel_speed",100);
  this->publisher["right_wheel_speed"] = nh.advertise<geometry_msgs::Twist>("right_wheel_speed",100);
}

void Turtlebot3::subscribeTopics(ros::NodeHandle& nh)
{
  this->subscriber["velocity"] = nh.subscribe("cmd_vel", 10, &Turtlebot3::subscribeVelocityCommand, this);
}

void Turtlebot3::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
  this->last_cmd_vel_time = ros::Time::now();
  this->wheel_speed_cmd[LEFT]  = msg->linear.x - (msg->angular.z * this->wheel_separation / 2);
  this->wheel_speed_cmd[RIGHT] = msg->linear.x + (msg->angular.z * this->wheel_separation / 2);
}

void Turtlebot3::updateJoint(unsigned int index,double& w,ros::Duration step_time)
{
  double v;
  v = this->wheel_speed_cmd[index];
  w = v / (this->wheel_diameter / 2);
  this->joint_states.velocity[index] = w;
  this->joint_states.position[index]= this->joint_states.position[index] + w * step_time.toSec();
}

void Turtlebot3::updateOdometry(double w_left,double w_right,ros::Duration step_time)
{
  double d1,d2;
  double dr,da;
  d1 = d2 = 0;
  dr = da = 0;

  d1 = step_time.toSec() * (this->wheel_diameter / 2) * w_left;
  d2 = step_time.toSec() * (this->wheel_diameter / 2) * w_right;

  if(isnan(d1))
  {
    d1 = 0;
  }
  if(isnan(d2))
  {
    d2 = 0;
  }

  dr = (d1 + d2) / 2;
  da = (d2 - d1) / this->wheel_separation;

  // compute odometric pose
  this->odom_pose[0] += dr * cos(this->odom_pose[2]);
  this->odom_pose[1] += dr * sin(this->odom_pose[2]);
  this->odom_pose[2] += da;

  // compute odometric instantaneouse velocity
  this->odom_vel[0] = dr / step_time.toSec();
  this->odom_vel[1] = 0.0;
  this->odom_vel[2] = da / step_time.toSec();

  this->odom.pose.pose.position.x = this->odom_pose[0];
  this->odom.pose.pose.position.y = this->odom_pose[1];
  this->odom.pose.pose.position.z = 0;
  this->odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->odom_pose[2]);

  // We should update the twist of the odometry
  this->odom.twist.twist.linear.x = this->odom_vel[0];
  this->odom.twist.twist.angular.z = this->odom_vel[2];
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


bool Turtlebot3::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - this->prev_update_time;
  this->prev_update_time = time_now;

  // zero-ing after timeout
  if(((time_now - this->last_cmd_vel_time).toSec() > this->cmd_vel_timeout))
  {
    this->wheel_speed_cmd[LEFT] = 0.0;
    this->wheel_speed_cmd[RIGHT] = 0.0;
  }

  this->wheel_speed[LEFT].linear.x  = this->wheel_speed_cmd[LEFT];
  this->wheel_speed[RIGHT].linear.x = this->wheel_speed_cmd[RIGHT];
  this->publisher["left_wheel_speed"].publish(this->wheel_speed[LEFT]);
  this->publisher["right_wheel_speed"].publish(this->wheel_speed[RIGHT]);

  // joint_states
  double w_left, w_right;
  updateJoint(LEFT, w_left, step_time);
  updateJoint(RIGHT, w_right, step_time);
  this->joint_states.header.stamp = time_now;
  this->publisher["joint_states"].publish(this->joint_states);

  // odom
  updateOdometry(w_left, w_right, step_time);
  this->odom.header.stamp = time_now;
  this->publisher["odom"].publish(this->odom);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  this->tf_broadcaster.sendTransform(odom_tf);

  return true;
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
