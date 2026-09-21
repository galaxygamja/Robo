#include <assert.h>
#include <limits.h>
#include <string.h>
#include "drive_output.h"
#include "robot_config.h"

using robot_profiles::DriveType;
using drive_output::calculate;
constexpr drive_output::Calibration left{96, 1, 1380, 1500, 1620};
constexpr drive_output::Calibration right{96, -1, 1370, 1490, 1630};
constexpr drive_output::Calibration invalidDirection{96, 0, 1380, 1500, 1620};
constexpr drive_output::Calibration invalidLimit{0, 1, 1380, 1500, 1620};
constexpr drive_output::Calibration invalidPulse{96, 1, 1500, 1500, 1620};

// These are also compile-time regressions when a cross compiler is available
// but no native executable can be run. Runtime sweeps below remain required CI.
static_assert(calculate(DriveType::ContinuousServo, true, 0, left).pulseUs == 1500, "Armed zero must be neutral");
static_assert(calculate(DriveType::ContinuousServo, false, 0, left).pulseUs == 0, "Disarmed must have no pulse");
static_assert(calculate(DriveType::ContinuousServo, false, 96, left).logicalPwm == 0, "Disabled ignores commands");
static_assert(calculate(DriveType::ContinuousServo, true, 96, left).pulseUs == 1620, "Forward endpoint");
static_assert(calculate(DriveType::ContinuousServo, true, -96, left).pulseUs == 1380, "Reverse endpoint");
static_assert(calculate(DriveType::ContinuousServo, true, 48, left).pulseUs == 1560, "Half positive power");
static_assert(calculate(DriveType::ContinuousServo, true, -48, left).pulseUs == 1440, "Half negative power");
static_assert(calculate(DriveType::ContinuousServo, true, 40, left).pulseUs == 1550, "Protocol fixture mapping");
static_assert(calculate(DriveType::ContinuousServo, true, 96, right).pulseUs == 1370, "Independent neutral and inversion");
static_assert(calculate(DriveType::ContinuousServo, true, -96, right).pulseUs == 1630, "Asymmetric span");
static_assert(calculate(DriveType::ContinuousServo, true, INT_MAX, left).pulseUs == 1620, "Clamp before arithmetic");
static_assert(calculate(DriveType::ContinuousServo, true, INT_MIN, right).pulseUs == 1630, "Clamp before sign inversion");
static_assert(calculate(DriveType::DcHbridge, true, 96, left).forwardDuty == 96, "DC forward");
static_assert(calculate(DriveType::DcHbridge, true, -96, left).reverseDuty == 96, "DC reverse");
static_assert(calculate(DriveType::DcHbridge, true, 40, right).reverseDuty == 40, "DC inversion");
static_assert(calculate(DriveType::DcHbridge, true, 0, left).forwardDuty == 0 && calculate(DriveType::DcHbridge, true, 0, left).reverseDuty == 0, "DC zero coasts");
static_assert(calculate(DriveType::DcHbridge, false, 96, left).forwardDuty == 0, "DC disabled output");
static_assert(calculate(DriveType::ContinuousServo, true, 96, invalidDirection).pulseUs == 0, "Bad calibration fail closed");
static_assert(calculate(DriveType::ContinuousServo, true, 96, invalidLimit).pulseUs == 0, "Zero limit fail closed");
static_assert(calculate(DriveType::ContinuousServo, true, 96, invalidPulse).pulseUs == 0, "Invalid pulse fail closed");
static_assert(drive_output::servoDuty(0) == 0, "No signal for zero");
static_assert(drive_output::servoDuty(-1) == 0 && drive_output::servoDuty(INT_MAX) == 0, "Duty invalid input guard");
static_assert(drive_output::servoDuty(1500) == 1229, "50 Hz 14-bit pulse conversion");
static_assert(config::maxPwm == 96 && config::pinsValid(), "Shared wire limit and pin safety");
static_assert(config::hardwareEnabled == (HARDWARE_OUTPUT_ENABLED == 1 && ACTUATOR_CALIBRATION_CONFIRMED == 1), "Both safety acknowledgements required");
static_assert(config::continuousDrive == (ROBOT_PROFILE == ROBOT_PROFILE_H1), "Build profile determines drive");
static_assert(config::hasDiscSensor == (ROBOT_PROFILE == ROBOT_PROFILE_H1), "Only H1 has disc sensor");
static_assert(config::toolServoCount == (ROBOT_PROFILE == ROBOT_PROFILE_H1 ? 1 : 2), "Functional servo count");

// Exercise the complete bounded power sweep at compile time as well: board
// cross-compilers can verify this even when no host executable is available.
constexpr bool mappingSweepValid() {
  int previousLeftPulse = 0;
  int previousRightPulse = 2001;
  for (int power = -300; power <= 300; ++power) {
    const int expected = power < -96 ? -96 : power > 96 ? 96 : power;
    const auto l = calculate(DriveType::ContinuousServo, true, power, left);
    const auto r = calculate(DriveType::ContinuousServo, true, power, right);
    if (l.logicalPwm != expected || r.logicalPwm != expected ||
        l.pulseUs < 1380 || l.pulseUs > 1620 ||
        r.pulseUs < 1370 || r.pulseUs > 1630 ||
        l.pulseUs < previousLeftPulse || r.pulseUs > previousRightPulse ||
        l.forwardDuty != 0 || l.reverseDuty != 0 ||
        r.forwardDuty != 0 || r.reverseDuty != 0) return false;
    previousLeftPulse = l.pulseUs;
    previousRightPulse = r.pulseUs;
    for (int inverted = 0; inverted <= 1; ++inverted) {
      const auto c = inverted ? right : left;
      const auto dc = calculate(DriveType::DcHbridge, true, power, c);
      const int signedPower = expected * c.direction;
      if (dc.logicalPwm != expected || dc.pulseUs != 0 ||
          dc.forwardDuty != unsigned(signedPower > 0 ? signedPower : 0) ||
          dc.reverseDuty != unsigned(signedPower < 0 ? -signedPower : 0) ||
          (dc.forwardDuty != 0 && dc.reverseDuty != 0)) return false;
      for (int continuous = 0; continuous <= 1; ++continuous) {
        const auto type = continuous ? DriveType::ContinuousServo : DriveType::DcHbridge;
        const auto stopped = calculate(type, false, power, c);
        if (stopped.logicalPwm != 0 || stopped.pulseUs != 0 ||
            stopped.forwardDuty != 0 || stopped.reverseDuty != 0) return false;
      }
    }
  }
  return true;
}
static_assert(mappingSweepValid(), "Full -300..300 mapping, clamp, inversion, monotonicity and disabled-output sweep");

int main() {
  const char* wireIds[] = {"H1", "B1", "B2", "H2"};
  const char* displayIds[] = {"H1", "B1", "B2", "B3"};
  for (int i = 1; i <= 4; ++i) {
    const auto profile = robot_profiles::get(i);
    assert(robot_profiles::valid(i));
    assert(strcmp(profile.wireId, wireIds[i - 1]) == 0);
    assert(strcmp(profile.displayId, displayIds[i - 1]) == 0);
    assert(profile.toolServoCount == (i == 1 ? 1 : 2));
    assert(profile.discSensor == (i == 1));
    assert(strcmp(profile.role, i == 1 ? "hamster" : "beaver") == 0);
    assert(strcmp(robot_profiles::driveName(profile.drive), i == 1 ? "continuous_servo" : "dc_hbridge") == 0);
  }
  assert(!robot_profiles::valid(0) && !robot_profiles::valid(5));
  assert(strcmp(config::robotId, wireIds[ROBOT_PROFILE - 1]) == 0);
  assert(strcmp(config::displayId, displayIds[ROBOT_PROFILE - 1]) == 0);
  int lastPulse = 0;
  for (int power = -300; power <= 300; ++power) {
    const auto continuous = calculate(DriveType::ContinuousServo, true, power, left);
    assert(continuous.logicalPwm >= -96 && continuous.logicalPwm <= 96);
    assert(continuous.pulseUs >= 1380 && continuous.pulseUs <= 1620);
    assert(continuous.pulseUs >= lastPulse);
    assert(continuous.forwardDuty == 0 && continuous.reverseDuty == 0);
    lastPulse = continuous.pulseUs;
    const auto dc = calculate(DriveType::DcHbridge, true, power, right);
    assert(dc.forwardDuty <= 96 && dc.reverseDuty <= 96 && dc.pulseUs == 0);
    assert(dc.forwardDuty == 0 || dc.reverseDuty == 0);
    const auto stopped = calculate(DriveType::ContinuousServo, false, power, right);
    assert(stopped.logicalPwm == 0 && stopped.pulseUs == 0 && stopped.forwardDuty == 0 && stopped.reverseDuty == 0);
  }
  // A non-neutral stop pulse is never synthesized by the disabled path.
  for (int pulse = 900; pulse <= 2100; ++pulse) {
    assert(drive_output::servoDuty(pulse) > 0 && drive_output::servoDuty(pulse) < (1U << 14));
  }
  return 0;
}
