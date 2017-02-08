/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim */

#include "turtlebot3_core_config.h"

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Turtlebot3 using RC100 remocon
geometry_msgs::Twist cmd_vel_rc100_msg;
ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tfbroadcaster;

/*******************************************************************************
* HardwareTimer for imu, encoder, cmd_vel of Turtlebot3
*******************************************************************************/
HardwareTimer timer_sensors_state(TIMER_CH1);
HardwareTimer timer_imu(TIMER_CH2);

/*******************************************************************************
* SoftwareTimer for * of Turtlebot3
*******************************************************************************/
static uint32_t tTime[3];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;
bool init_encoder_[2]  = {false, false};
int32_t last_diff_tick_[2];
int32_t last_tick_[2];
double last_rad_[2];
double last_velocity_[2];
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
cIMU imu;

/*******************************************************************************
* Declaration for Remocon (RC100)
*******************************************************************************/
RC100 remote_controller;
double const_cmd_vel    = 0.2;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
float joint_states_pos[2] = {0.0, 0.0};
float joint_states_vel[2] = {0.0, 0.0};
float joint_states_eff[2] = {0.0, 0.0};

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200); // TODO: Testing
  nh.subscribe(cmd_vel_sub);
  nh.advertise(sensor_state_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  tfbroadcaster.init(nh);

  nh.loginfo("Connected to OpenCR board!");

  // Setting for Dynamixel motors
  motor_driver.init();

  timer_sensors_state.pause();
  timer_sensors_state.setPeriod(1000000/SENSOR_STATE_PUBLISH_PERIOD); // 0.1 sec
  timer_sensors_state.attachInterrupt(publish_sensor_state_msg);
  timer_sensors_state.refresh();
  timer_sensors_state.resume();

  // Setting for IMU
  imu.begin();

  timer_imu.pause();
  timer_imu.setPeriod(1000000/IMU_PUBLISH_PERIOD); // 0.1 sec
  timer_imu.attachInterrupt(publish_imu_msg);
  timer_imu.refresh();
  timer_imu.resume();

  // Setting for remocon(ROBOTIS RC100) and cmd_vel
  remote_controller.begin(1);  //57600bps for RC100

  cmd_vel_rc100_msg.linear.x  = 0.0;
  cmd_vel_rc100_msg.angular.z = 0.0;

  // Setting for SLAM and navigation (odometry, joint states, TF)
  odom_pose[0] = 0.0;
  odom_pose[1] = 0.0;
  odom_pose[2] = 0.0;

  joint_states.header.frame_id = "base_footprint";

  joint_states.name_length     = 2;
  joint_states.position_length = 2;
  joint_states.velocity_length = 2;
  joint_states.effort_length   = 2;

  joint_states.name     = joint_states_name;

  prev_update_time = millis();

  SerialBT2.begin(57600);
}

/*******************************************************************************
* loop function
*******************************************************************************/
void loop()
{
  receive_remocon_data();

  if ((millis()-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    control_motor_speed();
    tTime[0] = millis();
  }

  if ((millis()-tTime[1]) >= (1000 / CMD_VEL_PUBLISH_PERIOD))
  {
    cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
    tTime[1] = millis();
  }

  if ((millis()-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
  {
    publish_drive_information();
    tTime[2] = millis();
  }

  // Call all the callbacks waiting to be called at that point in time.
  nh.spinOnce();
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publish_imu_msg(void)
{
  /*
    Range   : Roll  : +/- 180 deg/sec
              Pitch : +/- 180 deg/sec
              Yaw   : +/- 180 deg/sec
    Scale   : Roll  : 10 = 1 deg/sec
              Pitch : 10 = 1 deg/sec
              Yaw   :  1 = 1 deg/sec
   */

  imu.update();

  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = imu.SEN.gyroADC[0];
  imu_msg.angular_velocity.y = imu.SEN.gyroADC[1];
  imu_msg.angular_velocity.z = imu.SEN.gyroADC[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = imu.SEN.accADC[0];
  imu_msg.linear_acceleration.y = imu.SEN.accADC[1];
  imu_msg.linear_acceleration.z = imu.SEN.accADC[2];
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = imu.quat[0];
  imu_msg.orientation.x = imu.quat[1];
  imu_msg.orientation.y = imu.quat[2];
  imu_msg.orientation.z = imu.quat[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(&imu_msg);

  tfs_msg.header.stamp    = nh.now();
  tfs_msg.header.frame_id = "base_footprint";
  tfs_msg.child_frame_id  = "imu_link";
  tfs_msg.transform.rotation.w = imu.quat[0];
  tfs_msg.transform.rotation.x = imu.quat[1];
  tfs_msg.transform.rotation.y = imu.quat[2];
  tfs_msg.transform.rotation.z = imu.quat[3];
  tfbroadcaster.sendTransform(tfs_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publish_sensor_state_msg(void)
{
  sensor_state_msg.stamp = nh.now();
  sensor_state_msg.battery = check_voltage();

  bool dxl_comm_result = false;

  dxl_comm_result = motor_driver.readEncoder(ADDR_X_PRESENT_POSITION, 4, sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
  {
    sensor_state_pub.publish(&sensor_state_msg);
  }
  else
  {
    // ROS_ERROR("syncReadDynamixelRegister failed");
  }

  int32_t current_tick = sensor_state_msg.left_encoder;

  if (!init_encoder_[LEFT])
  {
    last_tick_[LEFT] = current_tick;
    init_encoder_[LEFT] = true;
  }

  last_diff_tick_[LEFT] = current_tick - last_tick_[LEFT];
  last_tick_[LEFT] = current_tick;
  last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];

  current_tick = sensor_state_msg.right_encoder;

  if (!init_encoder_[RIGHT])
  {
    last_tick_[RIGHT] = current_tick;
    init_encoder_[RIGHT] = true;
  }

  last_diff_tick_[RIGHT] = current_tick - last_tick_[RIGHT];
  last_tick_[RIGHT] = current_tick;
  last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publish_drive_information(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = nh.now();

  // odom
  updateOdometry((double)(step_time * 0.001));
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // joint_states
  updateJoint();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);

  // tf
  updateTF(odom_tf);
  tfbroadcaster.sendTransform(odom_tf);
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool updateOdometry(double diff_time)
{
  float odom_vel[3];

  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double v, w;             // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;
  wheel_l = wheel_r = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if(step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick_[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick_[RIGHT];

  //ROS_INFO_STREAM("wheel_l = " << wheel_l);
  //ROS_INFO_STREAM("wheel_r = " << wheel_r);

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  v = WHEEL_RADIUS * (wheel_r + wheel_l) / 2 / step_time;
  w = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION / step_time;

  //ROS_INFO_STREAM("step_time = " << ((double)last_diff_realtime_tick_left_ + (double)last_diff_realtime_tick_right_) / 2.0 / 1000.0);
  //ROS_INFO_STREAM("step_time = " << step_time);
  //ROS_INFO_STREAM("v = " << v);
  //ROS_INFO_STREAM("w = " << w);

  last_velocity_[LEFT]  = wheel_l / step_time;
  last_velocity_[RIGHT] = wheel_r / step_time;

  //ROS_INFO_STREAM("last_velocity_left_ = " << last_velocity_left_);
  //ROS_INFO_STREAM("last_velocity_right_ = " << last_velocity_right_);

  // compute odometric pose
  odom_pose[0] += v * step_time * cos(odom_pose[2] + (w * step_time / 2));
  odom_pose[1] += v * step_time * sin(odom_pose[2] + (w * step_time / 2));
  odom_pose[2] += w * step_time;

  //ROS_INFO_STREAM(" x = " << odom_pose[0] << " y = " << odom_pose[1] << " r = " << odom_pose[2]);

  // compute odometric instantaneouse velocity
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  // We should update the twist of the odometry
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  return true;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void updateJoint(void)
{
  joint_states_pos[LEFT]  = last_rad_[LEFT];
  joint_states_pos[RIGHT] = last_rad_[RIGHT];

  joint_states_vel[LEFT]  = last_velocity_[LEFT];
  joint_states_vel[RIGHT] = last_velocity_[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom.header.frame_id = "odom";
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

/*******************************************************************************
* Receive remocon (RC100) data
*******************************************************************************/
void receive_remocon_data(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if(received_data & RC100_BTN_U)
    {
      cmd_vel_rc100_msg.linear.x  += VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }
    else if(received_data & RC100_BTN_D)
    {
      cmd_vel_rc100_msg.linear.x  -= VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }
    else if(received_data & RC100_BTN_L)
    {
      cmd_vel_rc100_msg.angular.z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }
    else if(received_data & RC100_BTN_R)
    {
      cmd_vel_rc100_msg.angular.z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }
    else if(received_data & RC100_BTN_1)
    {
      const_cmd_vel += VELOCITY_STEP;
    }
    else if(received_data & RC100_BTN_3)
    {
      const_cmd_vel -= VELOCITY_STEP;
    }
    else if(received_data & RC100_BTN_6)
    {
      cmd_vel_rc100_msg.linear.x  = const_cmd_vel;
      cmd_vel_rc100_msg.angular.z = 0.0;
    }
    else if(received_data & RC100_BTN_5)
    {
      cmd_vel_rc100_msg.linear.x  = 0.0;
      cmd_vel_rc100_msg.angular.z = 0.0;
    }

    goal_linear_velocity  = cmd_vel_rc100_msg.linear.x;
    goal_angular_velocity = cmd_vel_rc100_msg.angular.z;
  }
}

/*******************************************************************************
* Control motor speed
*******************************************************************************/
void control_motor_speed(void)
{
  double wheel_speed_cmd[2];
  double lin_vel1;
  double lin_vel2;

  wheel_speed_cmd[LEFT]  = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION / 2);
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION / 2);

  lin_vel1 = wheel_speed_cmd[LEFT] * VELOCITY_CONSTANT_VAULE;

  if (lin_vel1 > LIMIT_X_MAX_VELOCITY)       lin_vel1 =  LIMIT_X_MAX_VELOCITY;
  else if (lin_vel1 < -LIMIT_X_MAX_VELOCITY) lin_vel1 = -LIMIT_X_MAX_VELOCITY;

  lin_vel2 = wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VAULE;

  if (lin_vel2 > LIMIT_X_MAX_VELOCITY)       lin_vel2 =  LIMIT_X_MAX_VELOCITY;
  else if (lin_vel2 < -LIMIT_X_MAX_VELOCITY) lin_vel2 = -LIMIT_X_MAX_VELOCITY;

  #ifdef DEBUG_MODE
    char log_msg[30];
    sprintf(log_msg, "linear_velocity L = %d", (int)lin_vel1);
    nh.loginfo(log_msg);
    sprintf(log_msg, "linear_velocity R = %d", (int)lin_vel2);
    nh.loginfo(log_msg);
  #endif

  bool dxl_comm_result = false;

  dxl_comm_result = motor_driver.speedControl(ADDR_X_GOAL_VELOCITY, 4, (int64_t)lin_vel1, (int64_t)lin_vel2);

  if (dxl_comm_result == false)
  {
    // ROS_ERROR("syncWriteDynamixelRegister failed");
  }
}

/*******************************************************************************
* Check voltage
*******************************************************************************/
float check_voltage(void)
{
  int adc_value;
  float vol_value;

  adc_value = analogRead(BDPIN_BAT_PWR_ADC);
  vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value/100;

  return vol_value;
}

// EOF
