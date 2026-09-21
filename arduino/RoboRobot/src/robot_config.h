#pragma once
#include <stddef.h>
#include <stdint.h>
#include "robot_profiles.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif
#if defined(ROBOT_ID) || defined(ROBOT_SERVO_COUNT) || defined(HAS_DISC_SENSOR)
#error "Legacy independent robot macros are unsafe here. Select one ROBOT_PROFILE instead."
#endif
#ifndef WIFI_SSID
#define WIFI_SSID "CHANGE_ME"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "CHANGE_ME"
#endif
#ifndef ROBOT_SHARED_TOKEN
#define ROBOT_SHARED_TOKEN "CHANGE_ME"
#endif
#ifndef HARDWARE_OUTPUT_ENABLED
#define HARDWARE_OUTPUT_ENABLED 0
#endif
#ifndef ACTUATOR_CALIBRATION_CONFIRMED
#define ACTUATOR_CALIBRATION_CONFIRMED 0
#endif
#ifndef ROBOT_PROFILE
#define ROBOT_PROFILE ROBOT_PROFILE_H1
#endif
#ifndef DRIVE_LEFT_DIRECTION
#define DRIVE_LEFT_DIRECTION 1
#endif
#ifndef DRIVE_RIGHT_DIRECTION
#if ROBOT_PROFILE == ROBOT_PROFILE_H1
#define DRIVE_RIGHT_DIRECTION -1
#else
#define DRIVE_RIGHT_DIRECTION 1
#endif
#endif
// SM-S4303R starting values ONLY: narrow, NOT measured. Calibrate each motor
// independently with the wheels clear of the table/floor.
#ifndef DRIVE_LEFT_MIN_US
#define DRIVE_LEFT_MIN_US 1380
#endif
#ifndef DRIVE_LEFT_NEUTRAL_US
#define DRIVE_LEFT_NEUTRAL_US 1500
#endif
#ifndef DRIVE_LEFT_MAX_US
#define DRIVE_LEFT_MAX_US 1620
#endif
#ifndef DRIVE_RIGHT_MIN_US
#define DRIVE_RIGHT_MIN_US 1380
#endif
#ifndef DRIVE_RIGHT_NEUTRAL_US
#define DRIVE_RIGHT_NEUTRAL_US 1500
#endif
#ifndef DRIVE_RIGHT_MAX_US
#define DRIVE_RIGHT_MAX_US 1620
#endif
#ifndef TOOL0_MIN_US
#define TOOL0_MIN_US 900
#endif
#ifndef TOOL0_MAX_US
#define TOOL0_MAX_US 2100
#endif
#ifndef TOOL1_MIN_US
#define TOOL1_MIN_US 900
#endif
#ifndef TOOL1_MAX_US
#define TOOL1_MAX_US 2100
#endif

namespace config {
constexpr auto profile = robot_profiles::get(ROBOT_PROFILE);
constexpr const char* robotId = profile.wireId;
constexpr const char* displayId = profile.displayId;
constexpr const char* firmwareVersion = "2.0.0";
constexpr bool continuousDrive = profile.drive == robot_profiles::DriveType::ContinuousServo;
constexpr uint8_t toolServoCount = profile.toolServoCount;
constexpr bool hasDiscSensor = profile.discSensor;
constexpr uint16_t udpPort = 4210;
constexpr size_t packetMax = 1024;
constexpr size_t replyMax = 1536;
constexpr uint8_t motorPins[4] = {3, 4, 5, 6}; // Beaver: L IN1/IN2, R IN1/IN2.
constexpr uint8_t driveServoPins[2] = {3, 5}; // Hamster: left/right SM-S4303R signal.
constexpr uint8_t servoPins[2] = {7, 10}; // MG90S tools; second absent on H1.
constexpr uint8_t sensorPin = 0;
constexpr int maxPwm = 96; // Protocol-v1 signed logical power, NOT measured RPM.
constexpr int motorDirection[2] = {DRIVE_LEFT_DIRECTION, DRIVE_RIGHT_DIRECTION};
constexpr int driveServoMinUs[2] = {DRIVE_LEFT_MIN_US, DRIVE_RIGHT_MIN_US};
constexpr int driveServoNeutralUs[2] = {DRIVE_LEFT_NEUTRAL_US, DRIVE_RIGHT_NEUTRAL_US};
constexpr int driveServoMaxUs[2] = {DRIVE_LEFT_MAX_US, DRIVE_RIGHT_MAX_US};
constexpr int servoMinUs[2] = {TOOL0_MIN_US, TOOL1_MIN_US};
constexpr int servoMaxUs[2] = {TOOL0_MAX_US, TOOL1_MAX_US};
constexpr bool discActiveLow = true; // Check TCRT5000 D0 polarity; ESP32 inputs are 3.3 V only.
constexpr uint32_t leaseMaxMs = 250;
constexpr uint32_t motorFrequency = 20000;
constexpr uint8_t motorBits = 8;
constexpr uint32_t servoFrequency = 50;
constexpr uint8_t servoBits = 14;
constexpr bool calibrationConfirmed = ACTUATOR_CALIBRATION_CONFIRMED == 1;
constexpr bool hardwareEnabled = HARDWARE_OUTPUT_ENABLED == 1 && calibrationConfirmed;
static_assert(robot_profiles::valid(ROBOT_PROFILE), "ROBOT_PROFILE must be 1=H1, 2=B1, 3=B2, or 4=B3 (wire H2)");
static_assert(HARDWARE_OUTPUT_ENABLED == 0 || HARDWARE_OUTPUT_ENABLED == 1, "Use 0 or 1 for output flag");
static_assert(ACTUATOR_CALIBRATION_CONFIRMED == 0 || ACTUATOR_CALIBRATION_CONFIRMED == 1, "Use 0 or 1 for calibration flag");
static_assert(maxPwm > 0 && maxPwm <= 255, "Invalid PWM limit");
static_assert(motorDirection[0] * motorDirection[0] == 1 && motorDirection[1] * motorDirection[1] == 1, "Motor polarity must be +/-1");
constexpr bool reserved(uint8_t pin) { return pin == 2 || pin == 8 || pin == 9 || pin == 18 || pin == 19; }
constexpr bool pinsValid() {
  uint8_t all[7] = {};
  unsigned count = 0;
  if (continuousDrive) {
    for (uint8_t pin : driveServoPins) all[count++] = pin;
  } else {
    for (uint8_t pin : motorPins) all[count++] = pin;
  }
  for (unsigned i = 0; i < toolServoCount; ++i) all[count++] = servoPins[i];
  if (hasDiscSensor) all[count++] = sensorPin;
  for (unsigned i = 0; i < count; ++i) {
    if (reserved(all[i])) return false;
    for (unsigned j = i + 1; j < count; ++j) if (all[i] == all[j]) return false;
  }
  return true;
}
constexpr bool driveCalibrationValid() {
  for (unsigned i = 0; i < 2; ++i) {
    if (driveServoMinUs[i] < 1000 || driveServoMaxUs[i] > 2000 ||
        driveServoMinUs[i] >= driveServoNeutralUs[i] ||
        driveServoNeutralUs[i] >= driveServoMaxUs[i]) return false;
  }
  return true;
}
static_assert(pinsValid(), "Pin collision or USB/boot pin in use");
static_assert(driveCalibrationValid(), "Drive servo pulses need 1000 <= min < neutral < max <= 2000");
static_assert(servoMinUs[0] >= 900 && servoMaxUs[0] <= 2100 && servoMinUs[0] <= servoMaxUs[0], "Invalid servo 0 endpoints");
static_assert(servoMinUs[1] >= 900 && servoMaxUs[1] <= 2100 && servoMinUs[1] <= servoMaxUs[1], "Invalid servo 1 endpoints");
} // namespace config
