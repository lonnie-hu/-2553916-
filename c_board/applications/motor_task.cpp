// 本文件集中放置：拨杆模式、姿态联动、手动拖动、R 标复位、双环 PID 和 CAN 发送。
// 初学时先搜索 motor_task，按主循环中的“第 1 步”到“第 6 步”阅读。
// 先理解三件事：目标角度是多少 -> PID 算出多少力矩 -> 把力矩发给电机。
// 位置外环 P 算目标速度；速度内环 PI 算力矩；A/B 的积分各自独立。
// 角度用 rad，速度用 rad/s，GM6020 电流模式下输出用 N*m。

#include <cmath>
#include <cstdint>

// MOTOR_TASK_TEST 只供电脑上的测试使用；STM32 正常编译时会包含下面的硬件代码。
#ifndef MOTOR_TASK_TEST
#include <cstring>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::DBus remote;
extern volatile float imu_yaw_unwrapped;
extern volatile float imu_yaw_speed;
extern volatile uint32_t imu_last_update_ms;
extern volatile bool imu_ready;

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(2, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(1, sp::RM_Motors::GM6020);
#endif

// ---------- 一、PID 的参数类型和可调整参数 ----------
namespace motor_control
{
struct Parameters
{
  float position_kp;           // 位置环比例系数，单位 (rad/s)/rad
  float speed_kp;              // 速度环比例系数，单位 N*m/(rad/s)
  float speed_ki;              // 速度环积分系数，单位 N*m/rad
  float max_speed;             // 目标速度限幅，单位 rad/s
  float max_torque;            // 输出力矩限幅，单位 N*m
  float max_integral;          // I 项输出限幅，单位 N*m
  float velocity_filter_time;  // 目标速度前馈的滤波时间，单位秒
};

}  // namespace motor_control

#ifndef MOTOR_TASK_TEST
namespace
{
// 正方向设置：从同一侧观察，yaw 和电机角度增加应表示同一转动方向。
// 某台电机方向相反时，把对应 Direction 改为 -1。
constexpr float kYawDirection = 1.0f;
constexpr float kMotorADirection = 1.0f;
constexpr float kMotorBDirection = 1.0f;
// 当前装置实测：速度 Ki=0.05 时，C 板联动能稳定运行。
// 保留位置 P + 速度 PI 双环；手动识别的灵敏度在后面的联动参数中调整。
constexpr float kPositionKp = 2.0f;                 // 位置外环 Kp：角度误差乘它，得到目标速度
constexpr float kSpeedKp = 0.04f;                   // 速度内环 Kp：速度误差乘它，得到 P 项力矩
constexpr float kSpeedKi = 0.05f;                   // 速度内环积分系数，保留当前实测值
constexpr float kMaxSpeed = 1.5f;                   // 起调目标速度最多 +/-1.5 rad/s
constexpr float kMaxTorque = 0.15f;                 // 起调总输出最多 +/-0.15 N*m
constexpr float kMaxIntegral = 0.05f;               // 开启积分后的 I 项力矩限幅
constexpr float kVelocityFilterTime = 0.02f;        // 目标速度前馈滤波时间，单位秒
constexpr bool kEnableVelocityFeedforward = false;  // 暂关前馈，先排除 yaw 差分的影响
constexpr motor_control::Parameters kPIDParameters{
  kPositionKp, kSpeedKp, kSpeedKi, kMaxSpeed, kMaxTorque, kMaxIntegral, kVelocityFilterTime};

}  // namespace
#endif

// ---------- 二、双环 PID：输入目标角度，输出电机力矩 ----------
namespace motor_control
{
struct PIDData
{
  float angle_error = 0.0f;  // 目标角度 - 实际角度
  float speed_set = 0.0f;    // 位置环算出来的目标速度
  float speed_error = 0.0f;  // 目标速度 - 实际速度
  float pout = 0.0f;         // 速度环 P 项力矩
  float iout = 0.0f;         // 速度环 I 项力矩，随控制循环累积
  float torque = 0.0f;       // 最后发给电机的力矩
};

// 双环结构：位置 P -> 目标速度；速度 PI -> 电机力矩。
// 使用连续多圈角度。速度环的 I 项提供平衡持续负载的力矩。
class CascadePID
{
public:
  explicit CascadePID(const Parameters & parameters, bool enable_feedforward = true)
  : parameters_(parameters), enable_feedforward_(enable_feedforward)
  {
  }

  PIDData data;

  // 清除过去累积的积分和速度前馈；下档、手动输入和切档时使用。
  void clear(float target)
  {
    last_target_ = std::isfinite(target) ? target : 0.0f;
    feedforward_speed_ = 0.0f;
    data = PIDData{};
  }

  float update(float target, float angle, float speed, float dt)
  {
    // 数据异常或控制间隔太长时，清积分并输出零力矩。
    if (
      !std::isfinite(target) || !std::isfinite(angle) || !std::isfinite(speed) ||
      !std::isfinite(dt) || dt <= 0.0f || dt > 0.05f) {
      clear(target);
      return 0.0f;
    }

    // 前馈：目标正在移动时，提前告诉电机目标移动得有多快。
    const float target_speed =
      enable_feedforward_ ? clamp((target - last_target_) / dt, parameters_.max_speed) : 0.0f;
    const float alpha = dt / (parameters_.velocity_filter_time + dt);
    feedforward_speed_ += alpha * (target_speed - feedforward_speed_);
    last_target_ = target;

    // 位置外环：角度还差多少，决定应该以多快的速度转动。
    data.angle_error = target - angle;
    data.speed_set =
      clamp(parameters_.position_kp * data.angle_error + feedforward_speed_, parameters_.max_speed);

    // 速度内环：速度还差多少，算 P 和 I 两项；暂时不使用 D。
    data.speed_error = data.speed_set - speed;
    data.pout = parameters_.speed_kp * data.speed_error;
    const float proposed_iout =
      clamp(data.iout + parameters_.speed_ki * data.speed_error * dt, parameters_.max_integral);
    const float proposed_torque = data.pout + proposed_iout;

    // 如果输出已经到上限，继续累积只会让积分越来越大，所以停止累积。
    // 误差反向时允许积分减小，帮助输出恢复。
    if (
      std::fabs(proposed_torque) <= parameters_.max_torque ||
      (proposed_torque > parameters_.max_torque && data.speed_error < 0.0f) ||
      (proposed_torque < -parameters_.max_torque && data.speed_error > 0.0f)) {
      data.iout = proposed_iout;
    }
    data.torque = clamp(data.pout + data.iout, parameters_.max_torque);
    return data.torque;
  }

private:
  const Parameters parameters_;
  const bool enable_feedforward_;
  float last_target_ = 0.0f;
  float feedforward_speed_ = 0.0f;

  static float clamp(float value, float limit)
  {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
  }
};

}  // namespace motor_control

// ---------- 三、联动逻辑：决定两台电机各自应该转到哪里 ----------
namespace attitude
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kEncoderStep = 2.0f * kPi / 8192.0f;

enum class Mode
{
  OFF,
  LINK,
  RESET
};  // 下档失能、中档联动、上档复位
enum class ManualMotor
{
  NONE,
  A,
  B
};  // 无手动输入、手动 A、手动 B

struct Feedback
{
  float yaw;        // C 板的实际 yaw，rad
  float yaw_speed;  // C 板的实际 yaw 角速度，rad/s
  float angle_a;    // A 电机编码器反馈的实际角度，rad
  float angle_b;    // B 电机编码器反馈的实际角度，rad
  float speed_a;    // A 电机反馈的实际速度，rad/s
  float speed_b;    // B 电机反馈的实际速度，rad/s
};

// 三个输入都用连续角度，避免跨过一整圈时突然跳变。
// 复位公式按两台电机底座固定、C 板独立转动计算。
class Linkage
{
public:
  static constexpr float kYawStillSpeed = 0.05f;  // rad/s
  static constexpr float kStillExcursion = 4.0f * kEncoderStep;
  static constexpr float kMotorStillExcursion = 4.0f * kEncoderStep;
  // 从停稳时的实际位置累计约 0.86 度新位移，再确认 20 ms。
  static constexpr float kManualEnterError = 0.004f;  // rad, about 0.23 degrees
  static constexpr uint32_t kCalibrationTimeMs = 500;
  static constexpr uint32_t kYawStillTimeMs = 150;
  static constexpr uint32_t kSettledTimeMs = 50;
  static constexpr uint32_t kManualConfirmTimeMs = 10;
  static constexpr uint32_t kManualHandoverStillTimeMs = 100;

  float target_a = 0.0f;       // 希望 A 电机到达的角度，交给 PID 使用
  float target_b = 0.0f;       // 希望 B 电机到达的角度，交给 PID 使用
  float yaw_reference = 0.0f;  // 联动参考 yaw：手动转动后更新它，实际 IMU yaw 不改
  bool calibrated = false;
  bool enabled = false;
  ManualMotor manual_motor = ManualMotor::NONE;

  bool manual_ready() const { return manual_ready_; }
  ManualMotor manual_candidate() const { return candidate_; }

  // 联动只计算 target_a/target_b；真正的电机力矩由后面的 PID 算。
  void update(Mode requested, float ratio, const Feedback & f, uint32_t now_ms, bool healthy)
  {
    if (!healthy || requested == Mode::OFF) {
      enabled = false;
      mode_ = Mode::OFF;
      manual_motor = ManualMotor::NONE;
      manual_ready_ = false;
      candidate_ = ManualMotor::NONE;
      if (healthy && !calibrated)
        calibrate(f, now_ms);
      else
        calibration_pending_ = false;
      return;
    }

    // 程序不知道 R 标在哪里，需要上电前先手动对齐三个 R 标。
    // 下档静止采集一次机械零点后，才能自动复位。
    if (requested == Mode::RESET && !calibrated) {
      enabled = false;
      mode_ = Mode::OFF;
      manual_motor = ManualMotor::NONE;
      manual_ready_ = false;
      candidate_ = ManualMotor::NONE;
      calibration_pending_ = false;
      return;
    }

    calibration_pending_ = false;
    enabled = true;
    if (requested != mode_) {
      manual_motor = ManualMotor::NONE;
      manual_ready_ = false;
      candidate_ = ManualMotor::NONE;
      yaw_still_anchor_ = f.yaw;
      yaw_still_since_ = now_ms;
      settled_since_ = now_ms;
      settled_angle_a_ = f.angle_a;
      settled_angle_b_ = f.angle_b;
      last_ = f;
      if (requested == Mode::RESET) {
        // 刚进入复位时，选择离当前位置最近的等价 R 方向。
        // 复位时两台都按 1:1 对齐 C 板方向，不看左拨杆比例。
        reset_a_ = nearest_angle(home_.angle_a + f.yaw - home_.yaw, f.angle_a);
        reset_b_ = nearest_angle(home_.angle_b + f.yaw - home_.yaw, f.angle_b);
        reset_yaw_ = f.yaw;
        target_a = reset_a_;
        target_b = reset_b_;
      }
      else {
        target_a = f.angle_a;
        target_b = f.angle_b;
      }
      mode_ = requested;
      update_reference(f.yaw);
      return;
    }

    if (requested == Mode::RESET) {
      target_a = reset_a_ + f.yaw - reset_yaw_;
      target_b = reset_b_ + f.yaw - reset_yaw_;
      last_ = f;
      return;
    }

    // 只加本次 yaw 变化量：手动调整过的位置会保留。
    // 切换比例只影响之后的运动，不会重新计算已经转过的角度。
    const float yaw_step = f.yaw - last_.yaw;
    target_a += yaw_step;
    target_b += ratio * yaw_step;

    const bool yaw_moving = std::fabs(f.yaw_speed) > kYawStillSpeed ||
                            std::fabs(f.yaw - yaw_still_anchor_) > kStillExcursion;
    if (yaw_moving) {
      yaw_still_anchor_ = f.yaw;
      yaw_still_since_ = now_ms;
      settled_since_ = now_ms;
      settled_angle_a_ = f.angle_a;
      settled_angle_b_ = f.angle_b;
      manual_motor = ManualMotor::NONE;
      manual_ready_ = false;
      candidate_ = ManualMotor::NONE;
    }
    const bool yaw_still = !yaw_moving && now_ms - yaw_still_since_ >= kYawStillTimeMs;

    if (manual_motor != ManualMotor::NONE) {
      follow_manual(f, ratio);
      const float owner_angle = manual_motor == ManualMotor::A ? f.angle_a : f.angle_b;
      if (std::fabs(owner_angle - manual_motion_anchor_) >= 0.5f * kEncoderStep) {
        manual_motion_anchor_ = owner_angle;
        last_manual_motion_ms_ = now_ms;
        manual_ready_ = false;
        candidate_ = ManualMotor::NONE;
        settled_since_ = now_ms;
      }
    }

    if (!manual_ready_) {
      // 看电机是否停稳，不要求角度误差接近零；静摩擦残差不会卡住入口。
      // 100 ms 内的累计编码器变化也要足够小，防止慢爬行被当成停稳。
      const bool settled = yaw_still &&
                           std::fabs(f.angle_a - settled_angle_a_) <= kMotorStillExcursion &&
                           std::fabs(f.angle_b - settled_angle_b_) <= kMotorStillExcursion &&
                           std::fabs(f.speed_a) < 0.12f && std::fabs(f.speed_b) < 0.12f &&
                           (manual_motor == ManualMotor::NONE ||
                            now_ms - last_manual_motion_ms_ >= kManualHandoverStillTimeMs);
      if (!settled) {
        settled_since_ = now_ms;
        settled_angle_a_ = f.angle_a;
        settled_angle_b_ = f.angle_b;
      }
      else if (now_ms - settled_since_ >= kSettledTimeMs) {
        manual_ready_ = true;
        // 只在准备好时采集一次，允许很慢的拖动继续累计位移。
        drag_anchor_a_ = f.angle_a;
        drag_anchor_b_ = f.angle_b;
      }
    }

    if (manual_ready_) {
      const auto previous_manual = manual_motor;
      detect_manual(f, ratio, now_ms);
      if (manual_motor != previous_manual) follow_manual(f, ratio, true);
    }

    // 松手后，手动输入端继续零力矩，位置不会被旧目标拉回。
    // 很慢的拖动也持续计入目标，不因读到零速度而重新上锁。
    // 从端继续跟完目标；停稳后可以改拖另一台电机。

    update_reference(f.yaw);
    last_ = f;
  }

private:
  Mode mode_ = Mode::OFF;
  Feedback last_{};
  Feedback home_{};
  Feedback calibration_anchor_{};
  bool calibration_pending_ = false;
  uint32_t calibration_since_ = 0;
  float reset_a_ = 0.0f, reset_b_ = 0.0f, reset_yaw_ = 0.0f;
  float yaw_still_anchor_ = 0.0f;
  uint32_t yaw_still_since_ = 0, settled_since_ = 0;
  float settled_angle_a_ = 0.0f, settled_angle_b_ = 0.0f;
  float drag_anchor_a_ = 0.0f, drag_anchor_b_ = 0.0f;
  bool manual_ready_ = false;
  ManualMotor candidate_ = ManualMotor::NONE;
  uint32_t candidate_since_ = 0, last_manual_motion_ms_ = 0;
  float candidate_peak_travel_ = 0.0f;
  float manual_motion_anchor_ = 0.0f;

  static float nearest_angle(float desired, float current)
  {
    return current + std::remainder(desired - current, 2.0f * kPi);
  }

  void update_reference(float yaw) { yaw_reference = yaw - (target_a - home_.angle_a); }

  void follow_manual(const Feedback & f, float ratio, bool capture_input = false)
  {
    // 刚识别手动输入时，补入检测前积累的全部偏转。
    // 之后只加编码器本次的位移，不覆盖已经累加的 yaw 变化量。
    // 这样再次很慢转 C 板时，起步阶段的 yaw 也会完整保留。
    if (manual_motor == ManualMotor::A) {
      const float correction = capture_input ? f.angle_a - target_a : f.angle_a - last_.angle_a;
      target_a += correction;
      target_b += ratio * correction;
    }
    else if (manual_motor == ManualMotor::B) {
      const float correction = capture_input ? f.angle_b - target_b : f.angle_b - last_.angle_b;
      target_b += correction;
      target_a += correction / ratio;
    }
  }

  void calibrate(const Feedback & f, uint32_t now_ms)
  {
    const bool stationary = std::fabs(f.yaw_speed) < kYawStillSpeed &&
                            std::fabs(f.speed_a) < 0.12f && std::fabs(f.speed_b) < 0.12f;
    if (!stationary) {
      calibration_pending_ = false;
      return;
    }
    if (
      !calibration_pending_ || std::fabs(f.yaw - calibration_anchor_.yaw) > kStillExcursion ||
      std::fabs(f.angle_a - calibration_anchor_.angle_a) > kStillExcursion ||
      std::fabs(f.angle_b - calibration_anchor_.angle_b) > kStillExcursion) {
      calibration_anchor_ = f;
      calibration_since_ = now_ms;
      calibration_pending_ = true;
    }
    if (now_ms - calibration_since_ >= kCalibrationTimeMs) {
      home_ = f;
      calibrated = true;
    }
  }

  void detect_manual(const Feedback & f, float ratio, uint32_t now_ms)
  {
    const float error_a = f.angle_a - target_a;
    const float error_b = f.angle_b - target_b;
    const float step_a = f.angle_a - last_.angle_a;
    const float step_b = f.angle_b - last_.angle_b;
    // 朝目标一侧正常追赶时，只刷新该台基准。
    // 已经拉离基准后的轻微回跳不抹掉累计拖动；候选确认另有两格回退容差。
    if (
      error_a * step_a < 0.0f && (f.angle_a - drag_anchor_a_) * error_a <= 0.0f &&
      candidate_ != ManualMotor::A) {
      drag_anchor_a_ = f.angle_a;
    }
    if (
      error_b * step_b < 0.0f && (f.angle_b - drag_anchor_b_) * error_b <= 0.0f &&
      candidate_ != ManualMotor::B) {
      drag_anchor_b_ = f.angle_b;
    }
    const float travel_a = f.angle_a - drag_anchor_a_;
    const float travel_b = f.angle_b - drag_anchor_b_;
    // 静止残差不算拖动：需要足够的新位移，而且是在朝目标外侧移动。
    const bool pulling_a = manual_motor != ManualMotor::A &&
                           std::fabs(travel_a) > kManualEnterError && std::fabs(step_a) > 0.0005f &&
                           travel_a * error_a > 0.0f;
    const bool pulling_b = manual_motor != ManualMotor::B &&
                           std::fabs(travel_b) > kManualEnterError && std::fabs(step_b) > 0.0005f &&
                           travel_b * error_b > 0.0f;
    if (candidate_ == ManualMotor::NONE) {
      if (pulling_a && (!pulling_b || std::fabs(travel_a) >= std::fabs(travel_b / ratio)))
        candidate_ = ManualMotor::A;
      else if (pulling_b)
        candidate_ = ManualMotor::B;
      if (candidate_ == ManualMotor::NONE) return;
      candidate_since_ = now_ms;
      candidate_peak_travel_ = candidate_ == ManualMotor::A ? travel_a : travel_b;
    }

    const float error = candidate_ == ManualMotor::A ? error_a : error_b;
    const float travel = candidate_ == ManualMotor::A ? travel_a : travel_b;
    if (std::fabs(travel) <= kManualEnterError || travel * error <= 0.0f || false) {
      candidate_ = ManualMotor::NONE;
      return;
    }
    if (std::fabs(travel) > std::fabs(candidate_peak_travel_)) candidate_peak_travel_ = travel;
    // 编码器短暂读到同一个数值时，仍保留拖动候选状态。
    if (now_ms - candidate_since_ >= kManualConfirmTimeMs) {
      manual_motor = candidate_;
      manual_motion_anchor_ = manual_motor == ManualMotor::A ? f.angle_a : f.angle_b;
      last_manual_motion_ms_ = now_ms;
      candidate_ = ManualMotor::NONE;
      manual_ready_ = false;
      settled_since_ = now_ms;
      settled_angle_a_ = f.angle_a;
      settled_angle_b_ = f.angle_b;
    }
  }
};

}  // namespace attitude

// ---------- 四、电机任务：把读取、联动、PID、发送串起来 ----------
#ifndef MOTOR_TASK_TEST
// 在调试器里展开 motor_debug，观察目标是否稳定，以及速度/力矩是否来回变号。
// 这些只是观测数据，不参与控制；电机与 C 板无需物理连接。
struct MotorDebug
{
  float yaw = 0.0f;
  float target_a = 0.0f, angle_a = 0.0f, speed_set_a = 0.0f, speed_a = 0.0f;
  float p_a = 0.0f, i_a = 0.0f, torque_a = 0.0f;
  float angle_error_a = 0.0f;
  float target_b = 0.0f, angle_b = 0.0f, speed_set_b = 0.0f, speed_b = 0.0f;
  float p_b = 0.0f, i_b = 0.0f, torque_b = 0.0f;
  float angle_error_b = 0.0f;
  float dt_ms = 0.0f;
  int mode = 0, manual_motor = 0;  // mode: 0下/1中/2上；manual: 0无/1A/2B
  int manual_candidate = 0;        // 正在确认：0无/1A/2B
  bool manual_ready = false;       // 两台停稳后允许识别，不要求角差接近零
  bool healthy = false, enabled = false;
};
volatile MotorDebug motor_debug;

namespace
{
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

attitude::Mode mode_from_right_switch(sp::DBusSwitchMode mode)
{
  switch (mode) {
    case sp::DBusSwitchMode::MID:
      return attitude::Mode::LINK;
    case sp::DBusSwitchMode::UP:
      return attitude::Mode::RESET;
    default:
      return attitude::Mode::OFF;
  }
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

  attitude::Linkage linkage;
  motor_control::CascadePID pid_a(kPIDParameters, kEnableVelocityFeedforward);
  motor_control::CascadePID pid_b(kPIDParameters, kEnableVelocityFeedforward);
  attitude::Mode previous_mode = attitude::Mode::OFF;
  attitude::ManualMotor previous_manual = attitude::ManualMotor::NONE;
  bool previous_enabled = false;
  uint32_t previous_ms = osKernelSysTick();

  for (;;) {
    // 第 1 步：读取右/左拨杆、C 板姿态和两台电机反馈。
    // 短暂关中断，避免读取到一半时反馈被改写；读完立即恢复。
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = osKernelSysTick();  // 本工程 1000 个 tick 为 1 秒，因此 now 的单位是 ms。
    const auto mode = mode_from_right_switch(remote.sw_r);
    const float ratio = ratio_from_left_switch(remote.sw_l);
    const attitude::Feedback feedback{
      kYawDirection * imu_yaw_unwrapped, kYawDirection * imu_yaw_speed,
      kMotorADirection * motor_a.angle,  kMotorBDirection * motor_b.angle,
      kMotorADirection * motor_a.speed,  kMotorBDirection * motor_b.speed};
    const bool online = remote.is_alive(now) && motor_a.is_alive(now) && motor_b.is_alive(now) &&
                        imu_ready && now - imu_last_update_ms < 100;
    __set_PRIMASK(irq_state);
    const bool healthy = online && std::isfinite(feedback.yaw) &&
                         std::isfinite(feedback.yaw_speed) && std::isfinite(feedback.angle_a) &&
                         std::isfinite(feedback.angle_b) && std::isfinite(feedback.speed_a) &&
                         std::isfinite(feedback.speed_b);

    const uint32_t elapsed_ms = now - previous_ms;
    const float dt = elapsed_ms > 0 ? elapsed_ms * 0.001f : 0.002f;
    // 第 2 步：下档/中档/上档 + 当前角度 -> 计算两台目标角度。
    // linkage.target_a 是 A 该到的位置，linkage.target_b 是 B 该到的位置。
    linkage.update(mode, ratio, feedback, now, healthy);
    float torque_a = 0.0f, torque_b = 0.0f;

    // 第 3 步：下档、失联或无法复位时，直接给两台零力矩。
    if (!linkage.enabled) {
      motor_a.cmd(0.0f);
      motor_b.cmd(0.0f);
      pid_a.clear(feedback.angle_a);
      pid_b.clear(feedback.angle_b);
    }
    else {
      // 第 4 步：切换状态时清掉旧积分，避免恢复控制时突然用力。
      if (!previous_enabled || mode != previous_mode) {
        pid_a.clear(linkage.target_a);
        pid_b.clear(linkage.target_b);
      }
      // 手动输入重新变成从端、或开始转 C 板时，也清除对应积分。
      if (
        previous_manual == attitude::ManualMotor::A &&
        linkage.manual_motor != attitude::ManualMotor::A)
        pid_a.clear(linkage.target_a);
      if (
        previous_manual == attitude::ManualMotor::B &&
        linkage.manual_motor != attitude::ManualMotor::B)
        pid_b.clear(linkage.target_b);

      // 第 5 步：目标角度 + 实际角度/速度 -> 双环 PID -> 力矩。
      // 被手动转动的那台保持 0 力矩；另一台继续跟随。
      if (linkage.manual_motor == attitude::ManualMotor::A)
        pid_a.clear(linkage.target_a);
      else
        torque_a = pid_a.update(linkage.target_a, feedback.angle_a, feedback.speed_a, dt);
      if (linkage.manual_motor == attitude::ManualMotor::B)
        pid_b.clear(linkage.target_b);
      else
        torque_b = pid_b.update(linkage.target_b, feedback.angle_b, feedback.speed_b, dt);

      motor_a.cmd(kMotorADirection * torque_a);
      motor_b.cmd(kMotorBDirection * torque_b);
    }
    // 第 6 步：通过 CAN 把上面算出的力矩发给两台电机。
    send_motor_commands();

    // 调试观察用：目标不动而角度/速度/力矩来回摆动，说明需继续整定闭环。
    motor_debug.yaw = feedback.yaw;
    motor_debug.target_a = linkage.target_a;
    motor_debug.angle_a = feedback.angle_a;
    motor_debug.speed_set_a = pid_a.data.speed_set;
    motor_debug.speed_a = feedback.speed_a;
    motor_debug.p_a = pid_a.data.pout;
    motor_debug.i_a = pid_a.data.iout;
    motor_debug.torque_a = torque_a;
    motor_debug.angle_error_a = linkage.target_a - feedback.angle_a;
    motor_debug.target_b = linkage.target_b;
    motor_debug.angle_b = feedback.angle_b;
    motor_debug.speed_set_b = pid_b.data.speed_set;
    motor_debug.speed_b = feedback.speed_b;
    motor_debug.p_b = pid_b.data.pout;
    motor_debug.i_b = pid_b.data.iout;
    motor_debug.torque_b = torque_b;
    motor_debug.angle_error_b = linkage.target_b - feedback.angle_b;
    motor_debug.dt_ms = elapsed_ms;
    motor_debug.mode = static_cast<int>(mode);
    motor_debug.manual_motor = static_cast<int>(linkage.manual_motor);
    motor_debug.manual_candidate = static_cast<int>(linkage.manual_candidate());
    motor_debug.manual_ready = linkage.manual_ready();
    motor_debug.healthy = healthy;
    motor_debug.enabled = linkage.enabled;
    previous_mode = mode;
    previous_manual = linkage.manual_motor;
    previous_enabled = linkage.enabled;
    previous_ms = now;
    osDelay(2);  // 等约 2 ms，再重新读取和控制一次。
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  // CAN 收到反馈时更新电机角度、速度；不是 CAN1 的消息不在这里处理。
  if (hcan != &hcan1) return;
  const uint32_t stamp_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    can1.recv();
    if (can1.frame_type) continue;
    if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
    if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
  }
}
#endif
