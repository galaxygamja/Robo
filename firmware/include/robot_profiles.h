#pragma once
#include <stdint.h>

// Stable build choices. B3 keeps the legacy wire ID H2 so existing camera maps
// and protocol-v1 clients do not silently address a different robot.
#define ROBOT_PROFILE_H1 1
#define ROBOT_PROFILE_B1 2
#define ROBOT_PROFILE_B2 3
#define ROBOT_PROFILE_B3 4

namespace robot_profiles {
enum class DriveType : uint8_t { ContinuousServo, DcHbridge };
struct Profile {
  const char* wireId;
  const char* displayId;
  const char* role;
  DriveType drive;
  uint8_t toolServoCount;
  bool discSensor;
};
constexpr Profile get(int selection) {
  return selection == ROBOT_PROFILE_H1
      ? Profile{"H1", "H1", "hamster", DriveType::ContinuousServo, 1, true}
      : selection == ROBOT_PROFILE_B1
      ? Profile{"B1", "B1", "beaver", DriveType::DcHbridge, 2, false}
      : selection == ROBOT_PROFILE_B2
      ? Profile{"B2", "B2", "beaver", DriveType::DcHbridge, 2, false}
      : Profile{"H2", "B3", "beaver", DriveType::DcHbridge, 2, false};
}
constexpr bool valid(int selection) { return selection >= 1 && selection <= 4; }
constexpr const char* driveName(DriveType type) {
  return type == DriveType::ContinuousServo ? "continuous_servo" : "dc_hbridge";
}
} // namespace robot_profiles
