#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <turtlebot3_msgs/DynamixelFeedback.h>

#include "dynamixel_sdk/dynamixel_sdk.h"                                   // Uses Dynamixel SDK library

// Control table address (Dynamixel XM series)
#define ADDR_XM_OPERATING_MODE           11
#define ADDR_XM_TORQUE_ENABLE            64
#define ADDR_XM_GOAL_VELOCITY           104
#define ADDR_XM_GOAL_POSITION           116
#define ADDR_XM_REALTIME_TICK           120
#define ADDR_XM_PRESENT_VELOCITY        128
#define ADDR_XM_PRESENT_POSITION        132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_CONTROL_MODE                 0
#define VELOCITY_CONTROL_MODE               1
#define POSITION_CONTROL_MODE               3 //(Default)
#define EXTENDED_POSITION_CONTROL_MODE      4
#define CURRENT_BASED_POSITION_CONTROL_MODE 5
#define PWM_CONTROL_MODE                    16
#define TORQUE_ENABLE                       1               // Value for enabling the torque
#define TORQUE_DISABLE                      0               // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE          100             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE          4000            // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD         20              // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                     0x1b
#define SMALL_U_ASCII_VALUE                 0x75
#define SMALL_D_ASCII_VALUE                 0x64

namespace turtlebot3_diff_driver
{
class Turtlebot3DiffDriver
{
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  std::string device_name_;
  // ROS Topic Subscriber
  ros::Subscriber velocity_sub1_;
  ros::Subscriber velocity_sub2_;
  // ROS Topic Publisher
  ros::Publisher position_pub_;
//  ros::Publisher position_pub1_;
//  ros::Publisher position_pub2_;
  //parameters
  double lin_vel1_;
  double lin_vel2_;
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  uint8_t dxl_left_id_;                 // left wheel
  uint8_t dxl_right_id_;                // right wheel

 public:
  Turtlebot3DiffDriver();
  ~Turtlebot3DiffDriver();
  void checkLoop(void);
  void closeDynamixel(void);

 private:
  bool initTurtlebot3DiffDriver(void);
  bool shutdownTurtlebot3DiffDriver(void);
  void velocityCallback1(const geometry_msgs::Twist::ConstPtr& vel);
  void velocityCallback2(const geometry_msgs::Twist::ConstPtr& vel);
  bool setTorque(uint8_t id, bool onoff);
  bool readPosition(uint8_t id, int32_t &position, int32_t &realtime_tick);
  void writeDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length, int32_t value);
  void readDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length);
  bool syncWriteDynamixelRegister(uint16_t addr, uint16_t length, int64_t left_wheel_value, int64_t right_wheel_value);
  bool syncReadDynamixelRegister(uint16_t addr, uint16_t length, int32_t &left_value, int32_t &right_value);
};
}

#endif // eof DYNAMIXEL_CONTROLLER_H
