#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#define MOTOR_TASK_TEST
#include "../applications/motor_task.cpp"

namespace {
using motor_control::CascadePID;
using motor_control::Parameters;

const Parameters kMotorParameters{6.0f, 0.1f, 0.3f, 6.0f, 0.5f, 0.5f, 0.02f};

void require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

void near(float actual, float expected, const std::string &message, float tolerance = 0.0001f)
{
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
    std::ostringstream out;
    out << message << ": actual=" << actual << ", expected=" << expected;
    throw std::runtime_error(out.str());
  }
}

// This synthetic plant tests the effect of the control structure and constant
// load. Its inertia/damping are examples, not measurements or a hardware
// stability proof: J * dv/dt = torque - load - damping * v.
struct Plant {
  float angle = 0.0f;
  float speed = 0.0f;
  void step(float torque, float dt, float load = 0.12f)
  {
    const float inertia = 0.02f;
    const float damping = 0.03f;
    speed += (torque - load - damping * speed) * dt / inertia;
    angle += speed * dt;
  }
};

void constant_load_error_converges()
{
  CascadePID pid(kMotorParameters);
  Parameters without_i = kMotorParameters;
  without_i.speed_ki = 0.0f;
  CascadePID proportional(without_i);
  Plant pi_plant, p_plant;
  const float target = 0.8f;
  const float dt = 0.002f;
  pid.clear(target);
  proportional.clear(target);
  for (int frame = 0; frame < 15000; ++frame) {
    const float torque = pid.update(target, pi_plant.angle, pi_plant.speed, dt);
    require(std::isfinite(torque) && std::fabs(torque) <= kMotorParameters.max_torque,
            "simulated torque remains finite and bounded");
    pi_plant.step(torque, dt);
    p_plant.step(proportional.update(target, p_plant.angle, p_plant.speed, dt), dt);
  }
  near(pi_plant.angle, target, "PI eliminates synthetic constant-load position error", 0.0005f);
  near(pi_plant.speed, 0.0f, "synthetic plant settles", 0.0005f);
  near(pid.data.iout, 0.12f, "integral supplies holding torque", 0.0005f);
  near(target - p_plant.angle, 0.2f, "P-only baseline retains load-dependent position error", 0.0005f);
}

void saturation_freezes_integral()
{
  // Start with room for integral, then maintain a blocked actuator until the
  // output reaches its limit. This checks accumulated I, not only P saturation.
  Parameters parameters = kMotorParameters;
  parameters.speed_kp = 0.05f;
  parameters.speed_ki = 1.0f;
  CascadePID pid(parameters);
  pid.clear(2.0f);
  for (int frame = 0; frame < 1000; ++frame) pid.update(2.0f, 0.0f, 0.0f, 0.002f);
  const float integral_at_limit = pid.data.iout;
  require(integral_at_limit > 0.18f && integral_at_limit <= 0.2f,
          "I uses available torque margin before conditional freeze");
  for (int frame = 0; frame < 5000; ++frame) {
    const float torque = pid.update(2.0f, 0.0f, 0.0f, 0.002f);
    near(pid.data.iout, integral_at_limit, "blocked actuator cannot continue winding up");
    require(torque <= parameters.max_torque, "torque respects positive limit");
  }

  const float reverse_torque = pid.update(-2.0f, 0.0f, 0.0f, 0.002f);
  require(reverse_torque < 0.0f, "error reversal immediately restores reverse torque");
  require(pid.data.iout < integral_at_limit, "I unwinds when error reverses");
  for (int frame = 0; frame < 1000; ++frame) pid.update(-2.0f, 0.0f, 0.0f, 0.002f);
  require(pid.data.iout < -0.18f && pid.data.torque < -0.48f,
          "controller recovers from sustained positive saturation");
}

void proportional_saturation_cannot_accumulate_i()
{
  CascadePID pid(kMotorParameters);
  pid.clear(2.0f);
  for (int frame = 0; frame < 5000; ++frame) {
    near(pid.update(2.0f, 0.0f, 0.0f, 0.002f), 0.5f, "P saturation obeys torque limit");
    near(pid.data.iout, 0.0f, "I remains zero while P alone saturates");
  }
}

void integral_output_has_own_limit()
{
  Parameters parameters = kMotorParameters;
  parameters.speed_kp = 0.0f;
  parameters.speed_ki = 1.0f;
  parameters.max_torque = 5.0f;
  parameters.max_integral = 0.12f;
  CascadePID pid(parameters);
  pid.clear(2.0f);
  for (int frame = 0; frame < 1000; ++frame) pid.update(2.0f, 0.0f, 0.0f, 0.002f);
  near(pid.data.iout, 0.12f, "independent positive integral-output limit");
  for (int frame = 0; frame < 1000; ++frame) pid.update(-2.0f, 0.0f, 0.0f, 0.002f);
  near(pid.data.iout, -0.12f, "independent negative integral-output limit");
}

void clear_removes_mode_history()
{
  CascadePID pid(kMotorParameters);
  pid.clear(0.5f);
  for (int frame = 0; frame < 100; ++frame) pid.update(0.5f, 0.0f, 0.0f, 0.002f);
  require(pid.data.iout > 0.0f, "fixture accumulates holding integral");
  pid.update(0.501f, 0.0f, 0.0f, 0.002f); // Also leave feedforward history.
  pid.clear(10.0f); // OFF/manual ownership/target recapture use this operation.
  near(pid.data.angle_error, 0.0f, "clear resets diagnostics");
  near(pid.data.iout, 0.0f, "clear resets integral");
  near(pid.data.torque, 0.0f, "clear resets torque");
  near(pid.update(10.0f, 10.0f, 0.0f, 0.002f), 0.0f,
       "target recapture has no stale torque or derivative kick");
  near(pid.data.speed_set, 0.0f, "target recapture clears filtered feedforward");
}

void independent_motor_state()
{
  CascadePID a(kMotorParameters), b(kMotorParameters);
  a.clear(0.5f);
  b.clear(0.0f);
  for (int frame = 0; frame < 100; ++frame) {
    a.update(0.5f, 0.0f, 0.0f, 0.002f);
    near(b.update(0.0f, 0.0f, 0.0f, 0.002f), 0.0f, "A cannot change B integral or feedforward");
  }
  const float a_integral = a.data.iout;
  b.clear(20.0f);
  near(a.data.iout, a_integral, "clearing B cannot clear A");
}

void integral_uses_elapsed_time()
{
  Parameters parameters{1.0f, 0.0f, 0.3f, 6.0f, 2.0f, 2.0f, 0.02f};
  CascadePID fixed(parameters), irregular(parameters);
  fixed.clear(1.0f);
  irregular.clear(1.0f);
  for (int frame = 0; frame < 500; ++frame) fixed.update(1.0f, 0.0f, 0.0f, 0.002f);
  const float intervals[] = {0.001f, 0.003f, 0.004f, 0.002f};
  for (int cycle = 0; cycle < 100; ++cycle) {
    for (float dt : intervals) irregular.update(1.0f, 0.0f, 0.0f, dt);
  }
  near(fixed.data.iout, 0.3f, "fixed updates integrate one second of error");
  near(irregular.data.iout, 0.3f, "irregular updates integrate one second of error");
  near(irregular.data.iout, fixed.data.iout, "I depends on elapsed time, not callback count");
}

void speed_limit_and_multi_turn_error()
{
  CascadePID pid(kMotorParameters);
  const float two_turns = 4.0f * 3.14159265358979323846f;
  pid.clear(two_turns);
  pid.update(two_turns, 0.0f, 0.0f, 0.002f);
  near(pid.data.angle_error, two_turns, "multi-turn position does not wrap to shortest angle");
  near(pid.data.speed_set, 6.0f, "large positive position error respects 6 rad/s limit");
  pid.update(-two_turns, 0.0f, 0.0f, 0.002f);
  near(pid.data.angle_error, -two_turns, "negative multi-turn position remains continuous");
  near(pid.data.speed_set, -6.0f, "large negative position error respects speed limit");
  pid.clear(0.0f);
  for (int frame = 1; frame < 500; ++frame) {
    const float target = 100.0f * frame;
    pid.update(target, target, 0.0f, 0.002f);
    require(std::fabs(pid.data.speed_set) <= 6.0f, "feedforward alone respects speed-reference limit");
  }
}

void moving_target_feedforward()
{
  CascadePID pid(kMotorParameters);
  pid.clear(0.0f);
  for (int frame = 1; frame <= 1000; ++frame) {
    const float target = 0.002f * static_cast<float>(frame);
    pid.update(target, target, 1.0f, 0.002f);
  }
  near(pid.data.speed_set, 1.0f, "constant moving target contributes filtered target velocity", 0.0002f);
  near(pid.data.speed_error, 0.0f, "speed feedback matches target feedforward", 0.0002f);
}

void feedforward_can_be_disabled_for_tuning()
{
  CascadePID pid(kMotorParameters, false);
  pid.clear(0.0f);
  for (int frame = 1; frame <= 1000; ++frame) {
    const float target = 0.002f * static_cast<float>(frame);
    near(pid.update(target, target, 0.0f, 0.002f), 0.0f,
         "disabled feedforward cannot turn target increments into extra torque");
    near(pid.data.speed_set, 0.0f, "disabled feedforward keeps zero reference at zero angle error");
  }
  pid.update(2.0f, 1.9f, 0.0f, 0.002f);
  near(pid.data.speed_set, 0.6f, "position outer loop remains active with feedforward disabled");
}

void zero_ki_is_a_two_loop_tuning_stage()
{
  const Parameters tuning{2.0f, 0.03f, 0.0f, 1.5f, 0.15f, 0.05f, 0.02f};
  CascadePID pid(tuning, false);
  pid.clear(0.1f);
  for (int frame = 0; frame < 1000; ++frame) {
    near(pid.update(0.1f, 0.0f, 0.0f, 0.002f), 0.006f,
         "two-loop P tuning still commands torque from position error");
    near(pid.data.iout, 0.0f, "zero Ki never accumulates holding integral");
  }
  require(pid.update(0.1f, 0.0f, 0.3f, 0.002f) < 0.0f,
          "speed inner loop brakes when measured speed exceeds its position-derived reference");
}

void invalid_sample_clears_state()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const float samples[][4] = {
    {0.5f, 0.0f, 0.0f, 0.0f},
    {0.5f, 0.0f, 0.0f, -0.002f},
    {0.5f, 0.0f, 0.0f, 0.051f},
    {0.5f, 0.0f, 0.0f, nan},
    {nan, 0.0f, 0.0f, 0.002f},
    {0.5f, inf, 0.0f, 0.002f},
    {0.5f, 0.0f, nan, 0.002f}
  };
  for (const auto &sample : samples) {
    CascadePID pid(kMotorParameters);
    pid.clear(0.5f);
    for (int frame = 0; frame < 100; ++frame) pid.update(0.5f, 0.0f, 0.0f, 0.002f);
    require(pid.data.iout > 0.0f, "invalid-sample fixture has nonzero integral");
    near(pid.update(sample[0], sample[1], sample[2], sample[3]), 0.0f,
         "invalid sample emits zero torque");
    near(pid.data.iout, 0.0f, "invalid sample clears integral");
    near(pid.data.speed_set, 0.0f, "invalid sample clears speed state");
    const float recovered_target = std::isfinite(sample[0]) ? sample[0] : 0.0f;
    near(pid.update(recovered_target, recovered_target, 0.0f, 0.002f), 0.0f,
         "valid next sample recovers without stale state");
  }
}

int passed = 0;
int failed = 0;
void run(const std::string &name, const std::function<void()> &test)
{
  try {
    test();
    ++passed;
    std::cout << "PASS " << name << '\n';
  } catch (const std::exception &error) {
    ++failed;
    std::cout << "FAIL " << name << ": " << error.what() << '\n';
  }
}
} // namespace

int main()
{
  run("constant-load steady-state convergence in synthetic plant", constant_load_error_converges);
  run("saturation freeze and reversal recovery", saturation_freezes_integral);
  run("proportional saturation cannot accumulate I", proportional_saturation_cannot_accumulate_i);
  run("independent integral-output limit", integral_output_has_own_limit);
  run("clear removes OFF/manual mode history", clear_removes_mode_history);
  run("independent state for two motors", independent_motor_state);
  run("integral uses elapsed time", integral_uses_elapsed_time);
  run("speed-reference limits and multi-turn error", speed_limit_and_multi_turn_error);
  run("moving-target feedforward", moving_target_feedforward);
  run("feedforward can be disabled for hardware tuning", feedforward_can_be_disabled_for_tuning);
  run("zero Ki preserves two-loop P tuning and braking", zero_ki_is_a_two_loop_tuning_stage);
  run("invalid-sample clearing and recovery", invalid_sample_clears_state);
  std::cout << "RESULT " << passed << " passed, " << failed << " failed\n";
  return failed == 0 ? 0 : 1;
}
