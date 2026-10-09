#include <cmath>
#include <cstring>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::DBus remote;
extern volatile float imu_yaw_unwrapped;

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(2, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(4, sp::RM_Motors::GM6020);

namespace
{
constexpr float kPositionKp = 1.0f;
constexpr float kPositionKd = 0.05f;
constexpr float kMaxTorque = 0.5f;

float ratio_from_left_switch(sp::DBusSwitchMode mode)
{
  switch (mode) {
    case sp::DBusSwitchMode::DOWN:
      return 0.5f;
    case sp::DBusSwitchMode::MID:
      return -1.0f;
    case sp::DBusSwitchMode::UP:
      return 3.0f;
  }
  return 0.5f;
}

float position_torque(const sp::RM_Motor & motor, float target)
{
  float torque = kPositionKp * (target - motor.angle) - kPositionKd * motor.speed;
  if (torque > kMaxTorque) torque = kMaxTorque;
  if (torque < -kMaxTorque) torque = -kMaxTorque;
  return torque;
}

void send_motor_commands()
{
  std::memset(can1.tx_data, 0, sp::CAN_DATA_LEN);
  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);
  can1.send(motor_a.tx_id);
}

}  // namespace

extern "C" void motor_task(void const * argument)
{
  (void)argument;
  osDelay(500);
  can1.config();
  can1.start();

  bool reference_initialized = false;
  bool last_reset_mode = false;
  float yaw_reference = 0.0f;
  float motor_a_reference = 0.0f;
  float motor_b_reference = 0.0f;

  for (;;) {
    const auto right_mode = remote.sw_r;
    const bool reset_mode = right_mode == sp::DBusSwitchMode::UP;

    if (
      right_mode == sp::DBusSwitchMode::DOWN || !motor_a.is_alive(osKernelSysTick()) ||
      !motor_b.is_alive(osKernelSysTick())) {
      motor_a.cmd(0.0f);
      motor_b.cmd(0.0f);
      send_motor_commands();
      reference_initialized = false;
      last_reset_mode = false;
      osDelay(2);
      continue;
    }

    if (!reference_initialized) {
      yaw_reference = imu_yaw_unwrapped;
      motor_a_reference = motor_a.angle;
      motor_b_reference = motor_b.angle;
      reference_initialized = true;
    }

    if (reset_mode && !last_reset_mode) {
      // 进入复位档前，应先将三个机械 R 标对齐；此处记录该位置为零点。
      yaw_reference = imu_yaw_unwrapped;
      motor_a_reference = motor_a.angle;
      motor_b_reference = motor_b.angle;
    }

    const float yaw_delta = imu_yaw_unwrapped - yaw_reference;
    const float ratio = ratio_from_left_switch(remote.sw_l);
    const float target_a = motor_a_reference + yaw_delta;
    const float target_b = motor_b_reference + ratio * yaw_delta;

    motor_a.cmd(position_torque(motor_a, target_a));
    motor_b.cmd(position_torque(motor_b, target_b));
    send_motor_commands();

    last_reset_mode = reset_mode;
    osDelay(2);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();
      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}