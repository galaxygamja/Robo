#pragma once
#include <stdint.h>
#include "robot_profiles.h"

// Pure calculations shared by the real output path and host unit tests.
// A zero command while enabled is a calibrated neutral pulse for H1. A
// disarmed/expired/disabled output is *no pulse*; it is not a holding brake.
namespace drive_output {
struct Calibration {
  int limit;
  int direction;
  int minimumUs;
  int neutralUs;
  int maximumUs;
};
struct Output {
  int logicalPwm;
  unsigned forwardDuty;
  unsigned reverseDuty;
  int pulseUs;
};
constexpr int clamp(int value, int limit) {
  return value < -limit ? -limit : value > limit ? limit : value;
}
constexpr Output calculate(robot_profiles::DriveType type, bool enabled,
                           int wanted, const Calibration& c) {
  if (!enabled || c.limit <= 0 || c.limit > 255 ||
      (c.direction != -1 && c.direction != 1)) return {0, 0, 0, 0};
  const int logical = clamp(wanted, c.limit);
  const int signedPower = logical * c.direction;
  if (type == robot_profiles::DriveType::DcHbridge) {
    return {logical, unsigned(signedPower > 0 ? signedPower : 0),
            unsigned(signedPower < 0 ? -signedPower : 0), 0};
  }
  if (c.minimumUs < 1000 || c.maximumUs > 2000 ||
      c.minimumUs >= c.neutralUs || c.neutralUs >= c.maximumUs) return {0, 0, 0, 0};
  const int span = signedPower >= 0 ? c.maximumUs - c.neutralUs : c.neutralUs - c.minimumUs;
  return {logical, 0, 0, c.neutralUs + signedPower * span / c.limit};
}
constexpr uint32_t servoDuty(int pulseUs) {
  // 50 Hz, 14 bits; zero explicitly means no signal. Guard the range so a
  // future caller cannot overflow or accidentally emit an invalid pulse.
  return pulseUs >= 900 && pulseUs <= 2100
      ? (uint32_t(pulseUs) * ((1UL << 14) - 1) + 10000) / 20000
      : 0;
}
} // namespace drive_output
