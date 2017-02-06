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

#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY           104
#define ADDR_X_GOAL_POSITION           116
#define ADDR_X_REALTIME_TICK           120
#define ADDR_X_PRESENT_VELOCITY        128
#define ADDR_X_PRESENT_POSITION        132

// Limit values (XM430-W350-T)
#define LIMIT_X_MAX_VELOCITY           415 // 2000(XL430-W?-T), 415(XL430-W350-T), 480(XM430-W210-T), 350(XM430-W350-T)

// Data Byte Length
#define LEN_X_TORQUE_ENABLE            1
#define LEN_X_GOAL_VELOCITY            4
#define LEN_X_GOAL_POSITION            4
#define LEN_X_REALTIME_TICK            2
#define LEN_X_PRESENT_VELOCITY         4
#define LEN_X_PRESENT_POSITION         4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXL_LEFT_ID                     1       // ID of left motor
#define DXL_RIGHT_ID                    2       // ID of right motor
#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool readEncoder(uint16_t addr, uint16_t length, int32_t &left_value, int32_t &right_value);
  bool speedControl(uint16_t addr, uint16_t length, int64_t left_wheel_value, int64_t right_wheel_value);

 private:
  // char device_name_[];
  int8_t baudrate_;
  float  protocol_version_;
  uint8_t left_wheel_id_;
  uint8_t right_wheel_id_;
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
};

#endif // DYNAMIXEL_CONTROLLER_H_
