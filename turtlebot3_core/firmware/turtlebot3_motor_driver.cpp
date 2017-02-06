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

#include "turtlebot3_motor_driver.h"

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  // device_name_(DEVICENAME),
  protocol_version_(PROTOCOL_VERSION),
  left_wheel_id_(DXL_LEFT_ID),
  right_wheel_id_(DXL_RIGHT_ID)
{
  // int dxl_comm_result      = false;               // Communication result
  // bool dxl_addparam_result = false;               // addParam result
  // bool dxl_getdata_result  = false;               // GetParam result
  // uint8_t dxl_error = 0;                          // Dynamixel error
  // int32_t dxl1_present_position = 0;              // Present position
  // int32_t dxl2_present_position = 0;              // Present position


  // dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  // dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  closeDynamixel();
}

bool Turtlebot3MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    // nh.loginfo("Succeeded to open the port!");
  }
  else
  {
    // nh.loginfo("Failed to open the port!");
    // nh.loginfo("Press any key to terminate...");
    // return;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    // nh.loginfo("Succeeded to change the baudrate!");
  }
  else
  {
    // nh.loginfo("Failed to change the baudrate!");
    // nh.loginfo("Press any key to terminate...");
    // return;
  }

  // Enable Dynamixel Torque
  setTorque(left_wheel_id_, true);
  setTorque(right_wheel_id_, true);
}

bool Turtlebot3MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);

  if(dxl_comm_result != COMM_SUCCESS)
      packetHandler_->printTxRxResult(dxl_comm_result);
  else if(dxl_error != 0)
      packetHandler_->printRxPacketError(dxl_error);
}

void Turtlebot3MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(left_wheel_id_, false);
  setTorque(right_wheel_id_, false);

  // Close port
  portHandler_->closePort();
}

bool Turtlebot3MotorDriver::readEncoder(uint16_t addr, uint16_t length, int32_t &left_value, int32_t &right_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, addr, length);

  // Set parameter
  dxl_addparam_result = groupSyncRead.addParam(left_wheel_id_);
  if (dxl_addparam_result != true)
  {
    // fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", left_wheel_id_);
    return false;
  }

  dxl_addparam_result = groupSyncRead.addParam(right_wheel_id_);
  if (dxl_addparam_result != true)
  {
    // fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", right_wheel_id_);
    return false;
  }

  // Syncread present position
  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler_->printTxRxResult(dxl_comm_result);

  // Check if groupsyncread data of Dynamixels are available
  dxl_getdata_result = groupSyncRead.isAvailable(left_wheel_id_, addr, length);
  if (dxl_getdata_result != true)
  {
    // fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", left_wheel_id_);
    return false;
  }

  dxl_getdata_result = groupSyncRead.isAvailable(right_wheel_id_, addr, length);
  if (dxl_getdata_result != true)
  {
    // fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", right_wheel_id_);
    return false;
  }

  // Get data
  left_value  = groupSyncRead.getData(left_wheel_id_,  addr, length);
  right_value = groupSyncRead.getData(right_wheel_id_, addr, length);

  return true;
}

bool Turtlebot3MotorDriver::speedControl(uint16_t addr, uint16_t length, int64_t left_wheel_value, int64_t right_wheel_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, addr, length);

  dxl_addparam_result_ = groupSyncWrite.addParam(left_wheel_id_, (uint8_t*)&left_wheel_value);
  if (dxl_addparam_result_ != true)
  {
    // logerror("[ID:%03d] groupSyncWrite addparam failed", left_wheel_id_);
    return false;
  }

  dxl_addparam_result_ = groupSyncWrite.addParam(right_wheel_id_, (uint8_t*)&right_wheel_value);
  if (dxl_addparam_result_ != true)
  {
    // logerror("[ID:%03d] groupSyncWrite addparam failed", right_wheel_id_);
    return false;
  }

  dxl_comm_result_ = groupSyncWrite.txPacket();

  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWrite.clearParam();
  return true;
}
