// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "turtlebot3_node/turtlebot3.hpp"

#include <memory>
#include <string>

// serial用の追加分
#include "turtlebot3_node/Serial.hpp"
#include <vector>
// #include <iostream>
#include <unistd.h>

using robotis::turtlebot3::TurtleBot3;
using namespace std::chrono_literals;

TurtleBot3::TurtleBot3(const std::string & usb_port)
: Node("turtlebot3_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  init_dynamixel_sdk_wrapper(usb_port);
  check_device_status();

  add_motors();
  add_wheels();
  add_sensors();
  add_devices();

  run();
}

TurtleBot3::Wheels * TurtleBot3::get_wheels()
{
  return &wheels_;
}

TurtleBot3::Motors * TurtleBot3::get_motors()
{
  return &motors_;
}

void TurtleBot3::init_dynamixel_sdk_wrapper(const std::string & usb_port)
{
  DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

  this->declare_parameter("opencr.id");
  this->declare_parameter("opencr.baud_rate");
  this->declare_parameter("opencr.protocol_version");

  this->get_parameter_or<uint8_t>("opencr.id", opencr.id, 200);
  this->get_parameter_or<uint32_t>("opencr.baud_rate", opencr.baud_rate, 1000000);
  this->get_parameter_or<float>("opencr.protocol_version", opencr.protocol_version, 2.0f);

  RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

  dxl_sdk_wrapper_->init_read_memory(
    extern_control_table.millis.addr,
    (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) +
    extern_control_table.profile_acceleration_right.length
  );
}

void TurtleBot3::check_device_status()
{
  if (dxl_sdk_wrapper_->is_connected_to_device()) {
    std::string sdk_msg;
    uint8_t reset = 1;

    dxl_sdk_wrapper_->set_data_to_device(
      extern_control_table.imu_re_calibration.addr,
      extern_control_table.imu_re_calibration.length,
      &reset,
      &sdk_msg);

    RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Calibration End");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed connection with Devices");
    rclcpp::shutdown();
    return;
  }

  const int8_t NOT_CONNECTED_MOTOR = -1;

  int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
    extern_control_table.device_status.addr,
    extern_control_table.device_status.length);

  switch (device_status) {
    case NOT_CONNECTED_MOTOR:
      RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and Power");
      break;

    default:
      break;
  }
}

void TurtleBot3::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter("motors.profile_acceleration_constant");
  this->declare_parameter("motors.profile_acceleration");

  this->get_parameter_or<float>(
    "motors.profile_acceleration_constant",
    motors_.profile_acceleration_constant,
    214.577);

  this->get_parameter_or<float>(
    "motors.profile_acceleration",
    motors_.profile_acceleration,
    0.0);
}

void TurtleBot3::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter("wheels.separation");
  this->declare_parameter("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.160);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.033);
}

void TurtleBot3::add_sensors()
{
  RCLCPP_INFO(this->get_logger(), "Add Sensors");

  this->declare_parameter("sensors.bumper_1");
  this->declare_parameter("sensors.bumper_2");

  this->declare_parameter("sensors.illumination");

  this->declare_parameter("sensors.ir");

  this->declare_parameter("sensors.sonar");

  bool is_connected_bumper_1 = this->get_parameter("sensors.bumper_1").as_bool();
  bool is_connected_bumper_2 = this->get_parameter("sensors.bumper_2").as_bool();

  bool is_connected_illumination = this->get_parameter("sensors.illumination").as_bool();

  bool is_connected_ir = this->get_parameter("sensors.ir").as_bool();

  bool is_connected_sonar = this->get_parameter("sensors.sonar").as_bool();

  sensors_.push_back(
    new sensors::BatteryState(
      node_handle_,
      "battery_state"));

  sensors_.push_back(
    new sensors::Imu(
      node_handle_,
      "imu",
      "magnetic_field",
      "imu_link"));

  sensors_.push_back(
    new sensors::SensorState(
      node_handle_,
      "sensor_state",
      is_connected_bumper_1,
      is_connected_bumper_2,
      is_connected_illumination,
      is_connected_ir,
      is_connected_sonar));

  sensors_.push_back(new sensors::JointState(node_handle_, "joint_states", "base_link"));
}

void TurtleBot3::add_devices()
{
  RCLCPP_INFO(this->get_logger(), "Add Devices");
  devices_["motor_power"] =
    new devices::MotorPower(node_handle_, dxl_sdk_wrapper_, "motor_power");
  devices_["reset"] =
    new devices::Reset(node_handle_, dxl_sdk_wrapper_, "reset");
  devices_["sound"] =
    new devices::Sound(node_handle_, dxl_sdk_wrapper_, "sound");
}

void TurtleBot3::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));
  heartbeat_timer(std::chrono::milliseconds(100));

  parameter_event_callback();
  cmd_vel_callback();
}

void TurtleBot3::publish_timer(const std::chrono::milliseconds timeout)
{
  publish_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time now = this->now();

      dxl_sdk_wrapper_->read_data_set();

      for (const auto & sensor : sensors_) {
        sensor->publish(now, dxl_sdk_wrapper_);
      }
    }
  );
}

void TurtleBot3::heartbeat_timer(const std::chrono::milliseconds timeout)
{
  heartbeat_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      static uint8_t count = 0;
      std::string msg;

      dxl_sdk_wrapper_->set_data_to_device(
        extern_control_table.heartbeat.addr,
        extern_control_table.heartbeat.length,
        &count,
        &msg);

      RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

      count++;
    }
  );
}

void TurtleBot3::parameter_event_callback()
{
  priv_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  while (!priv_parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
  }

  auto param_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      for (const auto & changed_parameter : event->changed_parameters) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "changed parameter name : %s",
          changed_parameter.name.c_str());

        if (changed_parameter.name == "motors.profile_acceleration") {
          std::string sdk_msg;

          motors_.profile_acceleration =
            rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();

          motors_.profile_acceleration =
            motors_.profile_acceleration / motors_.profile_acceleration_constant;

          union Data {
            int32_t dword[2];
            uint8_t byte[4 * 2];
          } data;

          data.dword[0] = static_cast<int32_t>(motors_.profile_acceleration);
          data.dword[1] = static_cast<int32_t>(motors_.profile_acceleration);

          uint16_t start_addr = extern_control_table.profile_acceleration_left.addr;
          uint16_t addr_length =
            (extern_control_table.profile_acceleration_right.addr -
            extern_control_table.profile_acceleration_left.addr) +
            extern_control_table.profile_acceleration_right.length;

          uint8_t * p_data = &data.byte[0];

          dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

          RCLCPP_INFO(
            this->get_logger(),
            "changed parameter value : %f [rev/min2] sdk_msg : %s",
            motors_.profile_acceleration,
            sdk_msg.c_str());
        }
      }
    };

  parameter_event_sub_ = priv_parameters_client_->on_parameter_event(param_event_callback);
}

void TurtleBot3::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
      std::string sdk_msg;

      union Data {
        int32_t dword[6];
        uint8_t byte[4 * 6];
      } data;

      data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
      data.dword[1] = 0;
      data.dword[2] = 0;
      data.dword[3] = 0;
      data.dword[4] = 0;
      data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

      uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
      uint16_t addr_length =
      (extern_control_table.cmd_velocity_angular_z.addr -
      extern_control_table.cmd_velocity_linear_x.addr) +
      extern_control_table.cmd_velocity_angular_z.length;

      uint8_t * p_data = &data.byte[0];

      dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

      // ここから追加

      Serial serial;
      if (serial.open("/dev/ttyACM1", 115200)){
        RCLCPP_INFO(this->get_logger(), "Successfully open serial");
        // return -1; // return -1 をしたときにどうなるのか？？
      } else {
        RCLCPP_INFO(this->get_logger(), "Failed to open serial");
      }

      float x_lim = 0.0;
      float y_lim = 0.0;
      float linear_x = 0.1
      float angular_z = 0.1

      if(linear_x > 1.0){
        x_lim = 1.0;
      }else if(linear_x < -1.0){
        x_lim = -1.0;
      }else{
        x_lim = linear_x * 5; // 0.1から0.5にする
      }

      if(angular_z > 1.0){
        y_lim = 1.0;
      }else if(angular_z < -1.0){
        y_lim = -1.0;
      }else{
        y_lim = angular_z;
      }
      

      int int_x = x_lim * (-63);
      int int_y = y_lim * (-63);

      //Serialへの書き込み
      int_x = int_x + 63 + 128;
      
      RCLCPP_INFO(this->get_logger(),"int_x:%d",int_x);
      unsigned char data_x = int_x;
      serial.write(&data_x, sizeof(data_x));
      

      int_y  = int_y + 63;
      RCLCPP_INFO(this->get_logger(),"int_y:%d",int_y);

      unsigned char data_y = int_y;
      serial.write(&data_y, sizeof(data_y));
      serial.close();

      //ここまで追加

      RCLCPP_DEBUG(
        this->get_logger(),
        "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
    }
  );
}
