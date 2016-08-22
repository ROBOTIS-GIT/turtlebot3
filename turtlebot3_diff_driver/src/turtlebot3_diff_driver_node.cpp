/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Yoonseok Pyo, leon */

#include "turtlebot3_diff_driver/turtlebot3_diff_driver.h"

using namespace turtlebot3_diff_driver;

Turtlebot3DiffDriver::Turtlebot3DiffDriver()
: nh_priv_("~"),
  lin_vel1_(0),
  lin_vel2_(0),
  dxl_left_id_(1),
  dxl_right_id_(2),
  device_name_(DEVICENAME),
  is_debug_(false)
{
  //Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.param("device_name", device_name_, device_name_);

  //Init target name
  ROS_ASSERT(initTurtlebot3DiffDriver());
  velocity_sub1_ = nh_.subscribe("/left_wheel_speed", 1, &Turtlebot3DiffDriver::velocityCallback1, this);
  velocity_sub2_ = nh_.subscribe("/right_wheel_speed", 1, &Turtlebot3DiffDriver::velocityCallback2, this);
  position_pub1_ = nh_.advertise<turtlebot3_msgs::DynamixelFeedback>("/left_wheel_position", 10);
  position_pub2_ = nh_.advertise<turtlebot3_msgs::DynamixelFeedback>("/right_wheel_position", 10);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize Packethandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if( portHandler_->openPort() )
  {
      ROS_INFO("Succeeded to open the port!");
  }
  else
  {
      ROS_ERROR("Failed to open the port!");
      shutdownTurtlebot3DiffDriver();
  }

  // Set port baudrate
  if( portHandler_->setBaudRate(BAUDRATE) )
  {
      ROS_INFO("Succeeded to change the baudrate!");
  }
  else
  {
      ROS_ERROR("Failed to change the baudrate!");
      shutdownTurtlebot3DiffDriver();
  }

  // Enable Dynamixel Torque
  setTorque(dxl_left_id_, true);
  setTorque(dxl_right_id_, true);
}

Turtlebot3DiffDriver::~Turtlebot3DiffDriver()
{
  ROS_ASSERT(shutdownTurtlebot3DiffDriver());
}

bool Turtlebot3DiffDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XM_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
      packetHandler_->printTxRxResult(dxl_comm_result);
  else if(dxl_error != 0)
      packetHandler_->printRxPacketError(dxl_error);
}

bool Turtlebot3DiffDriver::readPosition(uint8_t id, int32_t &position, int32_t &realtime_tick)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int32_t dxl_present_position = 0;
  int32_t dxl_realtime_tick = 0;

  dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_XM_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);

  if(dxl_comm_result != COMM_SUCCESS)
  {
      packetHandler_->printTxRxResult(dxl_comm_result);
      return false;
  }
  else if(dxl_error != 0)
  {
      packetHandler_->printRxPacketError(dxl_error);
      return false;
  }

  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_XM_REALTIME_TICK, (uint16_t*)&dxl_realtime_tick, &dxl_error);

  if(dxl_comm_result != COMM_SUCCESS)
  {
      packetHandler_->printTxRxResult(dxl_comm_result);
      return false;
  }
  else if(dxl_error != 0)
  {
      packetHandler_->printRxPacketError(dxl_error);
      return false;
  }

  position = dxl_present_position;
  realtime_tick = dxl_realtime_tick;

  return true;
}

void Turtlebot3DiffDriver::writeDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length, int32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (int16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (int32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
      packetHandler_->printRxPacketError(dxl_error);
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!",id);
  }
}

void Turtlebot3DiffDriver::readDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int     dxl_comm_result = COMM_TX_FAIL;

  int8_t  value8    = 0;
  int16_t value16   = 0;
  int32_t value32   = 0;


  if (length == 1)
  {
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value8, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value16, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value32, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) packetHandler_->printRxPacketError(dxl_error);

    if (length == 1)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value8);
    }
    else if (length == 2)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value16);
    }
    else if (length == 4)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value32);
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to read!", id);
  }
}

void Turtlebot3DiffDriver::checkLoop(void)
{
  turtlebot3_msgs::DynamixelFeedback feedback;

  writeDynamixelRegister(dxl_left_id_, ADDR_XM_GOAL_VELOCITY, 4, (int32_t)lin_vel1_);
  ROS_INFO("[ID] %u, [Goal Value] %d", dxl_left_id_, (int32_t)lin_vel1_);
  writeDynamixelRegister(dxl_right_id_, ADDR_XM_GOAL_VELOCITY, 4, -(int32_t)lin_vel2_);
  ROS_INFO("[ID] %u, [Goal Value] %d", dxl_right_id_, -(int32_t)lin_vel2_);

  readDynamixelRegister(dxl_left_id_, ADDR_XM_PRESENT_VELOCITY, 4);
  readDynamixelRegister(dxl_right_id_, ADDR_XM_PRESENT_VELOCITY, 4);

  readPosition(dxl_left_id_, feedback.position, feedback.realtime_tick);
  position_pub1_.publish(feedback);

  readPosition(dxl_right_id_, feedback.position, feedback.realtime_tick);
  position_pub2_.publish(feedback);

}

void Turtlebot3DiffDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(dxl_left_id_, false);
  setTorque(dxl_right_id_, false);

  // Close port
  portHandler_->closePort();
}

bool Turtlebot3DiffDriver::initTurtlebot3DiffDriver()
{
  ROS_INFO("turtlebot3_diff_driver_node : Init OK!");
  return true;
}

bool Turtlebot3DiffDriver::shutdownTurtlebot3DiffDriver()
{
  return true;
}

void Turtlebot3DiffDriver::velocityCallback1(const geometry_msgs::Twist::ConstPtr& vel)
{
  lin_vel1_ = vel->linear.x * 1263.632956882;

  if (lin_vel1_ > 450)
  {
    lin_vel1_ = 450;
  }
  else if (lin_vel1_ < -450)
  {
    lin_vel1_ = -450;
  }
}

void Turtlebot3DiffDriver::velocityCallback2(const geometry_msgs::Twist::ConstPtr& vel)
{
  lin_vel2_ = vel->linear.x * 1263.632956882;

  if (lin_vel2_ > 450)
  {
    lin_vel2_ = 450;
  }
  else if (lin_vel2_ < -450)
  {
    lin_vel2_ = -450;
  }
}

int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "turtlebot3_diff_driver_node");
  Turtlebot3DiffDriver dc;
  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    dc.checkLoop();

    ros::spinOnce();
    loop_rate.sleep();
  }

  dc.closeDynamixel();

  return 0;
}
