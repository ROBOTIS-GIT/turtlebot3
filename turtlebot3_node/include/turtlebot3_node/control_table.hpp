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

#ifndef TURTLEBOT3_NODE__CONTROL_TABLE_HPP_
#define TURTLEBOT3_NODE__CONTROL_TABLE_HPP_

#include <stdlib.h>

namespace robotis
{
namespace turtlebot3
{
constexpr uint8_t EEPROM = 1;
constexpr uint8_t RAM = 2;

constexpr uint8_t READ = 1;
constexpr uint8_t READ_WRITE = 3;

typedef struct
{
  uint16_t addr;
  uint8_t memory;
  uint16_t length;
  uint8_t rw;
} ControlItem;

typedef struct
{
  ControlItem model_number = {0, EEPROM, 2, READ};
  ControlItem model_information = {2, EEPROM, 4, READ};
  ControlItem firmware_version = {6, EEPROM, 1, READ};
  ControlItem id = {7, EEPROM, 1, READ};
  ControlItem baud_rate = {8, EEPROM, 1, READ};

  ControlItem millis = {10, RAM, 4, READ};
  ControlItem micros = {14, RAM, 4, READ};

  ControlItem device_status = {18, RAM, 1, READ};
  ControlItem heartbeat = {19, RAM, 1, READ_WRITE};

  ControlItem external_led_1 = {20, RAM, 1, READ_WRITE};
  ControlItem external_led_2 = {21, RAM, 1, READ_WRITE};
  ControlItem external_led_3 = {22, RAM, 1, READ_WRITE};
  ControlItem external_led_4 = {23, RAM, 1, READ_WRITE};

  ControlItem button_1 = {26, RAM, 1, READ};
  ControlItem button_2 = {27, RAM, 1, READ};

  ControlItem bumper_1 = {28, RAM, 1, READ};
  ControlItem bumper_2 = {29, RAM, 1, READ};

  ControlItem illumination = {30, RAM, 4, READ};
  ControlItem ir = {34, RAM, 4, READ};
  ControlItem sonar = {38, RAM, 4, READ};

  ControlItem battery_voltage = {42, RAM, 4, READ};
  ControlItem battery_percentage = {46, RAM, 4, READ};

  ControlItem sound = {50, RAM, 1, READ_WRITE};

  ControlItem imu_re_calibration = {59, RAM, 1, READ_WRITE};

  ControlItem imu_angular_velocity_x = {60, RAM, 4, READ};
  ControlItem imu_angular_velocity_y = {64, RAM, 4, READ};
  ControlItem imu_angular_velocity_z = {68, RAM, 4, READ};
  ControlItem imu_linear_acceleration_x = {72, RAM, 4, READ};
  ControlItem imu_linear_acceleration_y = {76, RAM, 4, READ};
  ControlItem imu_linear_acceleration_z = {80, RAM, 4, READ};
  ControlItem imu_magnetic_x = {84, RAM, 4, READ};
  ControlItem imu_magnetic_y = {88, RAM, 4, READ};
  ControlItem imu_magnetic_z = {92, RAM, 4, READ};
  ControlItem imu_orientation_w = {96, RAM, 4, READ};
  ControlItem imu_orientation_x = {100, RAM, 4, READ};
  ControlItem imu_orientation_y = {104, RAM, 4, READ};
  ControlItem imu_orientation_z = {108, RAM, 4, READ};

  ControlItem present_current_left = {120, RAM, 4, READ};
  ControlItem present_current_right = {124, RAM, 4, READ};
  ControlItem present_velocity_left = {128, RAM, 4, READ};
  ControlItem present_velocity_right = {132, RAM, 4, READ};
  ControlItem present_position_left = {136, RAM, 4, READ};
  ControlItem present_position_right = {140, RAM, 4, READ};

  ControlItem motor_torque_enable = {149, RAM, 1, READ_WRITE};

  ControlItem cmd_velocity_linear_x = {150, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_linear_y = {154, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_linear_z = {158, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_angular_x = {162, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_angular_y = {166, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_angular_z = {170, RAM, 4, READ_WRITE};

  ControlItem profile_acceleration_left = {174, RAM, 4, READ_WRITE};
  ControlItem profile_acceleration_right = {178, RAM, 4, READ_WRITE};
} ControlTable;

const ControlTable extern_control_table;
}  // namespace turtlebot3
}  // namespace robotis

#endif  // TURTLEBOT3_NODE__CONTROL_TABLE_HPP_
