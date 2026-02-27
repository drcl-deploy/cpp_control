/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _MOTOR_CRC_HG_H_
#define _MOTOR_CRC_HG_H_

#include <stdint.h>

#include <array>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"

typedef struct {
  uint8_t mode;  // desired working mode
  float q;       // desired angle (unit: radian)
  float dq;      // desired velocity (unit: radian/second)
  float tau;     // desired output torque (unit: N.m)
  float Kp;      // desired position stiffness (unit: N.m/rad )
  float Kd;      // desired velocity stiffness (unit: N.m/(rad/s) )
  uint32_t reserve = 0;
} MotorCmd;  // motor control

typedef struct {
  uint8_t modePr;
  uint8_t modeMachine;
  std::array<MotorCmd, 35> motorCmd;
  std::array<uint32_t, 4> reserve;
  uint32_t crc;
} LowCmd;

uint32_t crc32_core(uint32_t *ptr, uint32_t len);
void get_crc(unitree_hg::msg::LowCmd &msg);

namespace almi_ctrl {
inline void init_cmd_hg(unitree_hg::msg::LowCmd &cmd, uint8_t mode_machine, uint8_t mode_pr)
{
    cmd.mode_machine = mode_machine;
    cmd.mode_pr = mode_pr;
    for (int i = 0; i < 35; ++i) {
        cmd.motor_cmd[i].mode = 1;
        cmd.motor_cmd[i].q = 0.0f;
        cmd.motor_cmd[i].dq = 0.0f;
        cmd.motor_cmd[i].tau = 0.0f;
        cmd.motor_cmd[i].kp = 0.0f;
        cmd.motor_cmd[i].kd = 0.0f;
    }
}
}  // namespace almi_ctrl

#endif
