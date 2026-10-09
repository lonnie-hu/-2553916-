#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#define MOTOR_TASK_TEST
#include "../applications/motor_task.cpp"

namespace {
using attitude::Feedback;
using attitude::Linkage;
using attitude::ManualMotor;
using attitude::Mode;

void require(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

void near(float actual, float expected, const std::string &message, float tolerance = 0.0001f)
{
  if (std::fabs(actual - expected) > tolerance || !std::isfinite(actual)) {
    std::ostringstream out;
    out << message << ": actual=" << std::setprecision(9) << actual
        << ", expected=" << expected << ", tolerance=" << tolerance;
    throw std::runtime_error(out.str());
  }
}

struct Fixture {
  Linkage link;
  Feedback f{0.7f, 0.0f, 0.4f, -1.2f, 0.0f, 0.0f};
  uint32_t now = 0;
  float ratio;

  explicit Fixture(float r, bool calibrate = true) : ratio(r)
  {
    if (calibrate) {
      link.update(Mode::OFF, ratio, f, now, true);
      now += Linkage::kCalibrationTimeMs;
      link.update(Mode::OFF, ratio, f, now, true);
      require(link.calibrated, "stationary OFF calibration completes");
    }
    step();
    idle(400);
  }

  void step(Mode mode = Mode::LINK, bool healthy = true)
  {
    now += 2;
    link.update(mode, ratio, f, now, healthy);
  }

  void follow()
  {
    f.angle_a = link.target_a;
    f.angle_b = link.target_b;
  }

  void idle(uint32_t duration, bool follower_tracks = true)
  {
    f.yaw_speed = f.speed_a = f.speed_b = 0.0f;
    for (uint32_t elapsed = 0; elapsed < duration; elapsed += 2) {
      if (follower_tracks) follow();
      step();
    }
  }

  void drag(ManualMotor owner, float increment, unsigned frames, bool follower_tracks = true)
  {
    for (unsigned i = 0; i < frames; ++i) {
      if (owner == ManualMotor::A) {
        f.angle_a += increment;
        f.speed_a = increment / 0.002f;
        f.speed_b = 0.0f;
        if (follower_tracks) f.angle_b = link.target_b;
      } else {
        f.angle_b += increment;
        f.speed_b = increment / 0.002f;
        f.speed_a = 0.0f;
        if (follower_tracks) f.angle_a = link.target_a;
      }
      step();
    }
  }
};

void initial_link_capture(float ratio)
{
  Fixture x(ratio, false);
  require(x.link.enabled && !x.link.calibrated, "LINK works without claiming mechanical calibration");
  near(x.link.target_a, x.f.angle_a, "LINK entry captures A");
  near(x.link.target_b, x.f.angle_b, "LINK entry captures B");
}

void yaw_ratio(float ratio)
{
  Fixture x(ratio);
  const float a = x.link.target_a, b = x.link.target_b;
  const float turn = attitude::kPi / 3.0f;
  x.f.yaw += turn;
  x.f.yaw_speed = 1.0f;
  x.step();
  near(x.link.target_a, a + turn, "yaw +60 degrees makes A +60 degrees");
  near(x.link.target_b, b + ratio * turn, "yaw +60 degrees applies B ratio");
  require(x.link.manual_motor == ManualMotor::NONE, "yaw motion is not manual motor input");
  x.follow();
  x.f.yaw -= 2.0f * turn;
  x.f.yaw_speed = -1.0f;
  x.step();
  near(x.link.target_a, a - turn, "reverse yaw follows continuously");
  near(x.link.target_b, b - ratio * turn, "reverse yaw applies B ratio");
}

void slow_drag_and_retained_zero(float ratio, ManualMotor owner)
{
  Fixture x(ratio);
  const float a = x.f.angle_a, b = x.f.angle_b, yaw = x.f.yaw;
  x.drag(owner, 0.0001f, 600);
  require(x.link.manual_motor == owner, "0.0001 rad per 2 ms drag is recognized after accumulated displacement threshold");
  const float manual_delta = owner == ManualMotor::A ? x.f.angle_a - a : x.f.angle_b - b;
  const float da = owner == ManualMotor::A ? manual_delta : manual_delta / ratio;
  const float db = owner == ManualMotor::A ? ratio * manual_delta : manual_delta;
  near(x.link.target_a, a + da, "manual drag synchronizes A target");
  near(x.link.target_b, b + db, "manual drag synchronizes B target");
  near(x.f.yaw, yaw, "manual input leaves board yaw unchanged");
  near(x.link.yaw_reference, yaw - da, "manual input updates board linkage reference");
  x.idle(1000);
  require(x.link.manual_motor == owner, "stopped manual owner stays backdrivable");
  near(x.link.target_a, a + da, "A does not return after stopping");
  near(x.link.target_b, b + db, "B does not return after stopping");
  x.f.yaw += 0.1f;
  x.f.yaw_speed = 0.5f;
  x.step();
  near(x.link.target_a, a + da + 0.1f, "later yaw starts from manually changed zero");
  near(x.link.target_b, b + db + ratio * 0.1f, "later yaw preserves B manual offset");
}

void delayed_follower(float ratio, ManualMotor owner)
{
  Fixture x(ratio);
  x.drag(owner, 0.0001f, 600, false);
  require(x.link.manual_motor == owner, "manual owner is detected with follower lag");
  const float a = x.link.target_a, b = x.link.target_b;
  x.idle(400, false);
  require(x.link.manual_motor == owner, "stationary owner stays backdrivable while follower lags");
  near(x.link.target_a, a, "release retains pending A target");
  near(x.link.target_b, b, "release retains pending B target");
  for (unsigned i = 0; i < 300; ++i) {
    float &angle = owner == ManualMotor::A ? x.f.angle_b : x.f.angle_a;
    const float target = owner == ManualMotor::A ? b : a;
    const float delta = std::fmax(-0.001f, std::fmin(0.001f, target - angle));
    angle += delta;
    x.f.speed_a = owner == ManualMotor::A ? 0.0f : delta / 0.002f;
    x.f.speed_b = owner == ManualMotor::A ? delta / 0.002f : 0.0f;
    x.step();
    require(x.link.manual_motor == owner, "commanded follower approaching target cannot become owner");
    near(x.link.target_a, a, "normal follower feedback cannot drift A target");
    near(x.link.target_b, b, "normal follower feedback cannot drift B target");
  }
}

void ultra_slow_drag(float ratio, ManualMotor owner)
{
  Fixture x(ratio);
  x.drag(owner, 0.0001f, 600);
  require(x.link.manual_motor == owner, "establish manual owner before ultra-slow motion");
  const float initial_owner = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;
  x.drag(owner, 0.00001f, 300);
  const float angle = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;
  require(angle > initial_owner + 0.002f, "ultra-slow owner keeps moving");
  near(owner == ManualMotor::A ? x.link.target_a : x.link.target_b, angle,
       "continuous ultra-slow drag retains every increment");
  require(x.link.manual_motor == owner, "continuous ultra-slow drag must not time out as released");
}

void quantized_drag(float ratio, ManualMotor owner)
{
  Fixture x(ratio);
  for (unsigned i = 0; i < 1000; ++i) {
    const float increment = i % 8 == 0 ? attitude::kEncoderStep : 0.0f;
    x.drag(owner, increment, 1);
  }
  require(x.link.manual_motor == owner, "quantized plateaus do not prevent manual confirmation");
  const float actual = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;
  near(owner == ManualMotor::A ? x.link.target_a : x.link.target_b, actual,
       "quantized increments are retained");
}

void very_slow_quantized_drag(float ratio, ManualMotor owner)
{
  Fixture x(ratio);
  x.drag(owner, 0.0001f, 600);
  for (unsigned i = 0; i < 2000; ++i) {
    // One encoder count every 400 ms must not relock a known manual input.
    const float increment = i % 200 == 0 ? attitude::kEncoderStep : 0.0f;
    x.drag(owner, increment, 1);
    require(x.link.manual_motor == owner, "long encoder plateaus preserve manual owner");
    const float actual = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;
    near(owner == ManualMotor::A ? x.link.target_a : x.link.target_b, actual,
         "every very-slow quantized increment is retained");
  }
}

void handover_and_follower_overshoot(float ratio)
{
  Fixture x(ratio);
  x.drag(ManualMotor::A, 0.0001f, 600);
  x.idle(800);
  require(x.link.manual_motor == ManualMotor::A, "settled A remains owner until other motor is dragged");
  const float a = x.link.target_a, b = x.link.target_b;
  // A brief follower overshoot returns toward the target before confirmation.
  x.f.angle_b = b + 0.04f;
  x.f.speed_b = 1.0f;
  x.step();
  for (unsigned i = 0; i < 50; ++i) {
    x.f.angle_b = b + std::fmax(0.0f, 0.04f - 0.001f * static_cast<float>(i + 1));
    x.f.speed_b = x.f.angle_b == b ? 0.0f : -0.5f;
    x.step();
    require(x.link.manual_motor == ManualMotor::A, "receding follower overshoot cannot take ownership");
    near(x.link.target_a, a, "follower overshoot cannot change A target");
    near(x.link.target_b, b, "follower overshoot cannot change B target");
  }
  x.idle(800);
  x.drag(ManualMotor::B, 0.0001f, 600);
  require(x.link.manual_motor == ManualMotor::B, "settled A can hand over to manually dragged B");
  near(x.link.target_b, x.f.angle_b, "new B owner is tracked");
  x.idle(800);
  x.drag(ManualMotor::A, -0.0001f, 600);
  require(x.link.manual_motor == ManualMotor::A, "settled B can hand back to A in reverse direction");
  near(x.link.target_a, x.f.angle_a, "returning A owner is tracked");
}

void manual_waits_for_settle_and_exits_on_yaw(float ratio)
{
  Fixture x(ratio);
  x.f.yaw += 0.2f;
  x.f.yaw_speed = 1.0f;
  x.step();
  x.f.yaw_speed = 0.0f;
  x.drag(ManualMotor::A, -0.0001f, 400, false);
  require(x.link.manual_motor == ManualMotor::NONE, "manual detection waits while an encoder continues moving before readiness");
  x.idle(800);
  x.drag(ManualMotor::A, -0.0001f, 600);
  require(x.link.manual_motor == ManualMotor::A, "reverse manual input is recognized after settling");
  const float a = x.link.target_a, b = x.link.target_b;
  x.f.yaw += 0.1f;
  x.f.yaw_speed = 0.5f;
  x.step();
  require(x.link.manual_motor == ManualMotor::NONE, "explicit board yaw motion ends manual ownership");
  near(x.link.target_a, a + 0.1f, "yaw resumes from manual A target");
  near(x.link.target_b, b + ratio * 0.1f, "yaw resumes from manual B target");
}

void stopped_with_residual_can_be_dragged(float ratio, ManualMotor owner, float direction)
{
  Fixture x(ratio);
  x.f.yaw += 0.2f;
  x.f.yaw_speed = 1.0f;
  x.step();
  const float commanded_a = x.link.target_a, commanded_b = x.link.target_b;
  const float old_reference = x.link.yaw_reference, stationary_yaw = x.f.yaw;

  // Preserve both residuals while waiting. Fixture::follow() would hide this bug.
  x.f.angle_a = commanded_a + direction * 0.02f;
  x.f.angle_b = commanded_b + direction * 0.02f;
  x.idle(450, false);
  require(x.link.manual_motor == ManualMotor::NONE,
          "a stationary position residual alone cannot identify a manual input");
  near(x.link.target_a, commanded_a, "stationary A residual does not rewrite its commanded target");
  near(x.link.target_b, commanded_b, "stationary B residual does not rewrite its commanded target");

  x.drag(owner, direction * 0.0001f, 600);
  require(x.link.manual_motor == owner,
          "a stopped motor can enter manual mode despite a residual above the old settle limit");
  const float owner_angle = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;
  const float owner_error = owner_angle - (owner == ManualMotor::A ? commanded_a : commanded_b);
  const float da = owner == ManualMotor::A ? owner_error : owner_error / ratio;
  const float db = owner == ManualMotor::A ? ratio * owner_error : owner_error;
  near(x.link.target_a, commanded_a + da, "capture includes the complete A displacement and residual");
  near(x.link.target_b, commanded_b + db, "capture includes the complete B displacement and residual");
  near(owner == ManualMotor::A ? x.link.target_a : x.link.target_b, owner_angle,
       "manual owner target reaches its actual new angle");
  near(x.f.yaw, stationary_yaw, "manual residual capture keeps measured board yaw unchanged");
  near(x.link.yaw_reference, old_reference - da, "A displacement updates the board linkage reference");

  x.idle(1000);
  require(x.link.manual_motor == owner, "release retains the manual owner after residual capture");
  near(x.link.target_a, commanded_a + da, "release retains the adjusted A zero without recentering");
  near(x.link.target_b, commanded_b + db, "release retains the adjusted B zero without recentering");
  x.f.yaw += 0.08f;
  x.f.yaw_speed = 0.5f;
  x.step();
  require(x.link.manual_motor == ManualMotor::NONE, "new board motion exits residual-based manual input");
  near(x.link.target_a, commanded_a + da + 0.08f, "later board motion adds to the new A zero");
  near(x.link.target_b, commanded_b + db + ratio * 0.08f, "later board motion adds to the new B zero");
}

void residual_slow_tracking_is_not_manual(float ratio)
{
  Fixture x(ratio);
  x.f.yaw += 0.2f;
  x.f.yaw_speed = 1.0f;
  x.step();
  const float a = x.link.target_a, b = x.link.target_b;
  x.f.angle_a = a - 0.025f;
  x.f.angle_b = b + 0.03f;
  x.idle(450, false);

  for (unsigned frame = 0; frame < 700; ++frame) {
    const float da = std::fmin(0.00005f, a - x.f.angle_a);
    const float db = -std::fmin(0.00005f, x.f.angle_b - b);
    x.f.angle_a += da;
    x.f.angle_b += db;
    x.f.speed_a = da / 0.002f;
    x.f.speed_b = db / 0.002f;
    x.step();
    require(x.link.manual_motor == ManualMotor::NONE,
            "normal slow convergence after a stationary residual cannot become manual input");
    near(x.link.target_a, a, "normal A convergence keeps the original target");
    near(x.link.target_b, b, "normal B convergence keeps the original target");
  }
}

void residual_encoder_noise_is_not_manual(float ratio)
{
  Fixture x(ratio);
  x.f.yaw += 0.2f;
  x.f.yaw_speed = 1.0f;
  x.step();
  const float a = x.link.target_a, b = x.link.target_b;
  const float stopped_a = a + 0.03f, stopped_b = b - 0.03f;
  x.f.angle_a = stopped_a;
  x.f.angle_b = stopped_b;
  x.idle(450, false);

  const int noise_counts[] = {0, 1, 2, 1, 0, -1, -2, -1};
  for (unsigned frame = 0; frame < 1200; ++frame) {
    // Hold each noise sample longer than manual confirmation to exercise plateaus.
    const float noise = static_cast<float>(noise_counts[(frame / 20) % 8]) * attitude::kEncoderStep;
    x.f.angle_a = stopped_a + noise;
    x.f.angle_b = stopped_b - noise;
    x.step();
    require(x.link.manual_motor == ManualMotor::NONE,
            "one or two encoder counts of noise cannot turn a stationary residual into manual input");
    near(x.link.target_a, a, "residual A encoder noise does not drift the reference target");
    near(x.link.target_b, b, "residual B encoder noise does not drift the reference target");
  }
}

void cumulative_motion_blocks_initial_readiness(float ratio)
{
  Fixture x(ratio);
  x.f.yaw += 0.2f;
  x.f.yaw_speed = 1.0f;
  x.step();
  const float a = x.link.target_a, b = x.link.target_b;
  x.f.angle_a = a + 0.02f;
  x.f.angle_b = b - 0.02f;
  x.f.yaw_speed = x.f.speed_a = x.f.speed_b = 0.0f;

  for (unsigned frame = 0; frame < 700; ++frame) {
    // Sub-RPM feedback can read zero, but one count every 16 ms is still motion.
    if (frame % 8 == 0) x.f.angle_a += attitude::kEncoderStep;
    x.step();
    require(x.link.manual_motor == ManualMotor::NONE,
            "accumulated encoder motion must prevent arming even with zero reported RPM");
    near(x.link.target_a, a, "unsettled input cannot overwrite A target");
    near(x.link.target_b, b, "unsettled input cannot overwrite B target");
  }

  x.idle(450, false);
  x.drag(ManualMotor::A, 0.0001f, 600);
  require(x.link.manual_motor == ManualMotor::A,
          "the same motor becomes eligible after it actually stops despite its remaining residual");
  near(x.link.target_a, x.f.angle_a, "post-settle manual capture retains all actual A movement");
  near(x.link.target_b, b + ratio * (x.f.angle_a - a), "post-settle capture applies the correct B ratio");
}

void encoder_bounce_preserves_manual_confirmation(ManualMotor owner, bool before_candidate)
{
  Fixture x(3.0f);
  const float a = x.link.target_a, b = x.link.target_b;
  const auto move_counts = [&](int counts) {
    const float delta = static_cast<float>(counts) * attitude::kEncoderStep;
    if (owner == ManualMotor::A) x.f.angle_a += delta;
    else x.f.angle_b += delta;
    x.step();
  };

  if (before_candidate) {
    move_counts(12);
    require(x.link.manual_candidate() == ManualMotor::NONE,
            "twelve encoder counts have not crossed the candidate threshold");
    move_counts(-1);
    move_counts(10);
    require(x.link.manual_candidate() == owner,
            "one-count bounce before confirmation retains net twenty-one-count displacement");
  } else {
    move_counts(21);
    require(x.link.manual_candidate() == owner, "twenty-one counts starts manual confirmation");
    move_counts(-1);
    require(x.link.manual_candidate() == owner,
            "one-count bounce during confirmation preserves the manual candidate");
  }

  x.idle(Linkage::kManualConfirmTimeMs + 2, false);
  require(x.link.manual_motor == owner, "encoder plateau after a one-count bounce confirms manual input");
  const float owner_angle = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;
  const float correction = owner_angle - (owner == ManualMotor::A ? a : b);
  near(x.link.target_a, a + (owner == ManualMotor::A ? correction : correction / x.ratio),
       "confirmed bounce retains the complete A target correction");
  near(x.link.target_b, b + (owner == ManualMotor::A ? x.ratio * correction : correction),
       "confirmed bounce retains the complete B target correction");
  near(owner == ManualMotor::A ? x.link.target_a : x.link.target_b, owner_angle,
       "confirmed manual owner follows its measured angle despite encoder bounce");
  x.idle(500, false);
  near(owner == ManualMotor::A ? x.link.target_a : x.link.target_b, owner_angle,
       "release after bounce does not restore the old owner target");
}

void large_encoder_reversal_cancels_manual_candidate(ManualMotor owner)
{
  Fixture x(3.0f);
  const float a = x.link.target_a, b = x.link.target_b;
  float &angle = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;
  angle += 21.0f * attitude::kEncoderStep;
  x.step();
  require(x.link.manual_candidate() == owner, "start a candidate before reversing three encoder counts");
  angle -= 3.0f * attitude::kEncoderStep;
  x.step();
  require(x.link.manual_candidate() == ManualMotor::NONE,
          "three-count reversal below the displacement threshold cancels the candidate");
  x.idle(100, false);
  require(x.link.manual_motor == ManualMotor::NONE, "a canceled encoder candidate cannot confirm on a plateau");
  near(x.link.target_a, a, "canceled candidate leaves A target unchanged");
  near(x.link.target_b, b, "canceled candidate leaves B target unchanged");
}

void slow_yaw_after_manual(float ratio, ManualMotor owner)
{
  Fixture x(ratio);
  x.drag(owner, 0.0001f, 600);
  x.idle(800);
  require(x.link.manual_motor == owner, "establish stationary manual owner before slow yaw");
  const float a = x.link.target_a, b = x.link.target_b, yaw = x.f.yaw;

  x.f.yaw_speed = 0.02f;
  for (unsigned i = 0; i < 100; ++i) {
    x.f.yaw += 0.00004f;  // 0.02 rad/s at the 2 ms control period.
    if (owner == ManualMotor::A) x.f.angle_b = x.link.target_b;
    else x.f.angle_a = x.link.target_a;
    x.step();
    const float delta_yaw = x.f.yaw - yaw;
    near(x.link.target_a, a + delta_yaw, "slow yaw retains every A increment after manual input");
    near(x.link.target_b, b + ratio * delta_yaw,
         "slow yaw retains every B increment after manual input");
  }
  require(x.link.manual_motor == ManualMotor::NONE,
          "accumulated slow yaw eventually restores both motors to closed-loop control");
}

void small_yaw_after_manual(float ratio, ManualMotor owner)
{
  Fixture x(ratio);
  x.drag(owner, 0.0001f, 600);
  x.idle(800);
  require(x.link.manual_motor == owner, "establish manual owner before sub-threshold board motion");
  const float a = x.link.target_a, b = x.link.target_b, yaw = x.f.yaw;

  // The full excursion stays below kStillExcursion; it must still be counted.
  x.f.yaw_speed = 0.02f;
  for (unsigned i = 0; i < 50; ++i) {
    x.f.yaw += 0.00004f;
    if (owner == ManualMotor::A) x.f.angle_b = x.link.target_b;
    else x.f.angle_a = x.link.target_a;
    x.step();
  }
  const float delta_yaw = x.f.yaw - yaw;
  require(delta_yaw < Linkage::kStillExcursion,
          "small yaw regression does not depend on the board-motion detector firing");
  x.f.yaw_speed = 0.0f;
  for (unsigned i = 0; i < 200; ++i) {
    if (owner == ManualMotor::A) x.f.angle_b = x.link.target_b;
    else x.f.angle_a = x.link.target_a;
    x.step();
    near(x.link.target_a, a + delta_yaw, "stopped small yaw retains the A target");
    near(x.link.target_b, b + ratio * delta_yaw, "stopped small yaw retains the B target");
  }

  // Reverse through the start, then return: signed increments must cancel.
  for (float increment : {-0.00004f, 0.00004f}) {
    x.f.yaw_speed = increment / 0.002f;
    const unsigned frames = increment < 0.0f ? 100u : 50u;
    for (unsigned i = 0; i < frames; ++i) {
      x.f.yaw += increment;
      if (owner == ManualMotor::A) x.f.angle_b = x.link.target_b;
      else x.f.angle_a = x.link.target_a;
      x.step();
      const float net_yaw = x.f.yaw - yaw;
      near(x.link.target_a, a + net_yaw, "small reverse yaw preserves signed A increments");
      near(x.link.target_b, b + ratio * net_yaw, "small reverse yaw preserves signed B increments");
    }
  }
  near(x.link.target_a, a, "small board swing with zero net yaw returns only to its post-manual A target");
  near(x.link.target_b, b, "small board swing with zero net yaw returns only to its post-manual B target");
}

void small_yaw_and_manual_increments(float ratio, ManualMotor owner)
{
  Fixture x(ratio);
  x.drag(owner, 0.0001f, 600);
  x.idle(800);
  const float a = x.link.target_a, b = x.link.target_b, yaw = x.f.yaw;
  const float owner_start = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;

  x.f.yaw_speed = 0.02f;
  for (unsigned i = 0; i < 40; ++i) {
    x.f.yaw += 0.00004f;
    if (owner == ManualMotor::A) {
      x.f.angle_a += 0.0001f;
      x.f.angle_b = x.link.target_b;
    } else {
      x.f.angle_b -= 0.0001f;
      x.f.angle_a = x.link.target_a;
    }
    x.step();
    const float delta_yaw = x.f.yaw - yaw;
    const float owner_delta =
      (owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b) - owner_start;
    const float manual_a = owner == ManualMotor::A ? owner_delta : owner_delta / ratio;
    const float manual_b = owner == ManualMotor::A ? ratio * owner_delta : owner_delta;
    near(x.link.target_a, a + delta_yaw + manual_a,
         "sub-threshold yaw and manual encoder increments add in A target");
    near(x.link.target_b, b + ratio * delta_yaw + manual_b,
         "sub-threshold yaw and manual encoder increments add in B target");
    require(x.link.manual_motor == owner,
            "small simultaneous board motion has not crossed the owner exit threshold");
  }
}

void ratio_switch(float old_ratio, float new_ratio, ManualMotor owner)
{
  Fixture x(old_ratio);
  if (owner != ManualMotor::NONE) {
    x.drag(owner, 0.0001f, 600);
    require(x.link.manual_motor == owner, "establish owner for switch test");
  } else {
    x.f.yaw += 0.8f;
    x.f.yaw_speed = 1.0f;
    x.step();
    x.follow();
    x.idle(400);
  }
  const float a = x.link.target_a, b = x.link.target_b;
  x.ratio = new_ratio;
  x.f.speed_a = x.f.speed_b = 0.0f;
  x.step();
  near(x.link.target_a, a, "switch does not jump A target");
  near(x.link.target_b, b, "switch does not rescale previous B motion");
  if (owner != ManualMotor::NONE) {
    const float owner_start = owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b;
    x.drag(owner, 0.0001f, 40);
    const float increment = (owner == ManualMotor::A ? x.f.angle_a : x.f.angle_b) - owner_start;
    near(x.link.target_a, a + (owner == ManualMotor::A ? increment : increment / new_ratio),
         "manual continuation uses new A ratio increment");
    near(x.link.target_b, b + (owner == ManualMotor::A ? new_ratio * increment : increment),
         "manual continuation uses new B ratio increment");
  } else {
    x.f.yaw += 0.1f;
    x.f.yaw_speed = 0.5f;
    x.step();
    near(x.link.target_a, a + 0.1f, "after switch A remains 1:1");
    near(x.link.target_b, b + new_ratio * 0.1f, "after switch B uses new ratio");
  }
}

void off_and_health()
{
  Fixture x(3.0f);
  x.drag(ManualMotor::A, 0.0001f, 600);
  x.step(Mode::OFF);
  require(!x.link.enabled && x.link.manual_motor == ManualMotor::NONE, "OFF disables control and clears owner");
  x.f.angle_a += 1.0f;
  x.f.angle_b -= 0.4f;
  x.f.yaw += 0.2f;
  x.step();
  near(x.link.target_a, x.f.angle_a, "re-enable captures current A position");
  near(x.link.target_b, x.f.angle_b, "re-enable captures current B position");
  x.step(Mode::LINK, false);
  require(!x.link.enabled, "unhealthy feedback disables LINK");
  x.f.angle_a += 0.2f;
  x.step();
  near(x.link.target_a, x.f.angle_a, "health recovery captures current position");
  x.step(Mode::RESET, false);
  require(!x.link.enabled, "unhealthy feedback also disables RESET");
}

void calibration_and_reset()
{
  Fixture uncalibrated(3.0f, false);
  uncalibrated.drag(ManualMotor::A, 0.0001f, 600);
  require(uncalibrated.link.manual_motor == ManualMotor::A, "uncalibrated LINK can accept manual input");
  uncalibrated.step(Mode::RESET);
  require(!uncalibrated.link.enabled && uncalibrated.link.manual_motor == ManualMotor::NONE,
          "rejected uncalibrated RESET disables output and clears manual state");

  Linkage x;
  Feedback home{0.7f, 0.0f, 0.4f, -1.2f, 0.0f, 0.0f};
  x.update(Mode::RESET, 3.0f, home, 0, true);
  require(!x.enabled && !x.calibrated, "uncalibrated RESET cannot invent R zeros");
  x.update(Mode::OFF, 3.0f, home, 2, true);
  home.speed_a = 0.2f;
  x.update(Mode::OFF, 3.0f, home, 600, true);
  require(!x.calibrated, "moving motor blocks initial calibration");
  home.speed_a = 0.0f;
  x.update(Mode::OFF, 3.0f, home, 602, true);
  x.update(Mode::OFF, 3.0f, home, 1100, true);
  require(!x.calibrated, "calibration requires full stationary interval");
  x.update(Mode::OFF, 3.0f, home, 1102, true);
  require(x.calibrated, "initial mechanical calibration is stored");
  Feedback moved = home;
  moved.yaw += 0.6f;
  moved.angle_a = home.angle_a + 4.0f * attitude::kPi + 0.8f;
  moved.angle_b = home.angle_b - 6.0f * attitude::kPi + 0.3f;
  x.update(Mode::OFF, 3.0f, moved, 1200, true);
  x.update(Mode::OFF, 3.0f, moved, 1800, true);
  x.update(Mode::RESET, 3.0f, moved, 1802, true);
  near(x.target_a, home.angle_a + 4.0f * attitude::kPi + 0.6f,
       "RESET uses original A calibration and nearest equivalent direction");
  near(x.target_b, home.angle_b - 6.0f * attitude::kPi + 0.6f,
       "RESET uses original B calibration and nearest equivalent direction");
  require(std::fabs(x.target_a - moved.angle_a) <= attitude::kPi &&
          std::fabs(x.target_b - moved.angle_b) <= attitude::kPi,
          "RESET seeks at most a half turn on entry");
  const float a = x.target_a, b = x.target_b;
  moved.angle_a += 2.0f * attitude::kPi;
  moved.angle_b -= 2.0f * attitude::kPi;
  x.update(Mode::RESET, -1.0f, moved, 2000, true);
  near(x.target_a, a, "held RESET does not resample A nearest turn");
  near(x.target_b, b, "held RESET does not resample B nearest turn");
  moved.yaw += 0.1f;
  x.update(Mode::RESET, 0.5f, moved, 2002, true);
  near(x.target_a, a + 0.1f, "RESET follows yaw at A 1:1");
  near(x.target_b, b + 0.1f, "RESET follows yaw at B 1:1 regardless of left switch");
  x.update(Mode::LINK, 0.5f, moved, 2004, true);
  near(x.target_a, moved.angle_a, "RESET to LINK starts from actual A position");
  near(x.target_b, moved.angle_b, "RESET to LINK starts from actual B position");
}

void angle_and_tick_boundaries()
{
  Fixture x(-1.0f);
  x.f.yaw = attitude::kPi - 0.001f;
  x.f.yaw_speed = 1.0f;
  x.step();
  x.follow();
  const float a = x.link.target_a, b = x.link.target_b;
  x.f.yaw += 0.002f;  // The IMU task supplies unwrapped yaw across +pi.
  x.step();
  near(x.link.target_a, a + 0.002f, "unwrapped yaw crossing pi stays continuous");
  near(x.link.target_b, b - 0.002f, "B remains continuous across yaw boundary");

  Linkage wrap;
  Feedback f{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const uint32_t start = UINT32_MAX - 250u;
  wrap.update(Mode::OFF, 0.5f, f, start, true);
  wrap.update(Mode::OFF, 0.5f, f, start + 500u, true);
  require(wrap.calibrated, "calibration duration works across uint32 tick rollover");
  wrap.update(Mode::LINK, 0.5f, f, start + 502u, true);
  for (uint32_t dt = 504u; dt <= 1900u; dt += 2u) {
    if (dt > 904u) {
      f.angle_a += 0.0001f;
      f.speed_a = 0.05f;
      f.angle_b = wrap.target_b;
    }
    wrap.update(Mode::LINK, 0.5f, f, start + dt, true);
  }
  require(wrap.manual_motor == ManualMotor::A, "manual timing works after tick rollover");
}

int passed = 0, failed = 0;
void run(const std::string &name, const std::function<void()> &test)
{
  try {
    test();
    ++passed;
    std::cout << "PASS " << name << '\n';
  } catch (const std::exception &e) {
    ++failed;
    std::cout << "FAIL " << name << ": " << e.what() << '\n';
  }
}
}  // namespace

int main()
{
  const float ratios[] = {0.5f, -1.0f, 3.0f};
  for (float ratio : ratios) {
    const std::string label = std::to_string(ratio);
    run("initial capture ratio=" + label, [=] { initial_link_capture(ratio); });
    run("yaw ratio=" + label, [=] { yaw_ratio(ratio); });
    for (ManualMotor owner : {ManualMotor::A, ManualMotor::B}) {
      const std::string who = owner == ManualMotor::A ? "A" : "B";
      run("slow drag " + who + " ratio=" + label, [=] { slow_drag_and_retained_zero(ratio, owner); });
      run("delayed follower of " + who + " ratio=" + label, [=] { delayed_follower(ratio, owner); });
      run("ultra-slow drag " + who + " ratio=" + label, [=] { ultra_slow_drag(ratio, owner); });
      run("quantized drag " + who + " ratio=" + label, [=] { quantized_drag(ratio, owner); });
      run("very-slow quantized drag " + who + " ratio=" + label,
          [=] { very_slow_quantized_drag(ratio, owner); });
      run("slow yaw after manual " + who + " ratio=" + label,
          [=] { slow_yaw_after_manual(ratio, owner); });
      run("small yaw after manual " + who + " ratio=" + label,
          [=] { small_yaw_after_manual(ratio, owner); });
      run("small yaw and manual increments " + who + " ratio=" + label,
          [=] { small_yaw_and_manual_increments(ratio, owner); });
      for (float direction : {-1.0f, 1.0f}) {
        run("stopped residual then drag " + who + " direction=" + std::to_string(direction) + " ratio=" + label,
            [=] { stopped_with_residual_can_be_dragged(ratio, owner, direction); });
      }
    }
    run("residual slow convergence ratio=" + label, [=] { residual_slow_tracking_is_not_manual(ratio); });
    run("residual encoder noise ratio=" + label, [=] { residual_encoder_noise_is_not_manual(ratio); });
    run("cumulative motion blocks initial readiness ratio=" + label,
        [=] { cumulative_motion_blocks_initial_readiness(ratio); });
    run("handover and follower overshoot ratio=" + label, [=] { handover_and_follower_overshoot(ratio); });
    run("settling and yaw owner exit ratio=" + label, [=] { manual_waits_for_settle_and_exits_on_yaw(ratio); });
    for (float next : ratios) {
      if (ratio == next) continue;
      for (ManualMotor owner : {ManualMotor::NONE, ManualMotor::A, ManualMotor::B}) {
        const std::string who = owner == ManualMotor::NONE ? "none" : owner == ManualMotor::A ? "A" : "B";
        run("switch " + label + " to " + std::to_string(next) + " owner=" + who,
            [=] { ratio_switch(ratio, next, owner); });
      }
    }
  }
  run("OFF and health recovery", off_and_health);
  for (ManualMotor owner : {ManualMotor::A, ManualMotor::B}) {
    const std::string who = owner == ManualMotor::A ? "A" : "B";
    run("one-count bounce during manual confirmation " + who,
        [=] { encoder_bounce_preserves_manual_confirmation(owner, false); });
    run("one-count bounce before manual confirmation " + who,
        [=] { encoder_bounce_preserves_manual_confirmation(owner, true); });
    run("three-count reversal cancels manual confirmation " + who,
        [=] { large_encoder_reversal_cancels_manual_candidate(owner); });
  }
  run("calibration and nearest-direction RESET", calibration_and_reset);
  run("angle and tick boundaries", angle_and_tick_boundaries);
  std::cout << "RESULT " << passed << " passed, " << failed << " failed\n";
  return failed == 0 ? 0 : 1;
}
