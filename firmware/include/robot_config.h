#pragma once
#include <stdint.h>

#if __has_include("secrets.h")
#include "secrets.h"
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
#ifndef ROBOT_ID
#define ROBOT_ID "H1"
#endif
#ifndef ROBOT_SERVO_COUNT
#define ROBOT_SERVO_COUNT 1
#endif
#ifndef HAS_DISC_SENSOR
#define HAS_DISC_SENSOR 1
#endif

namespace config {
constexpr uint16_t udpPort = 4210;
constexpr size_t packetMax = 1024;
constexpr uint8_t motorPins[4] = {3, 4, 5, 6};
constexpr uint8_t servoPins[2] = {7, 10};
constexpr uint8_t sensorPin = 0;
constexpr int maxPwm = 96; // 8-bit PWM; calibrate with wheels raised first.
constexpr int motorDirection[2] = {1, 1}; // Change an entry to -1 when required.
constexpr int servoMinUs[2] = {900, 900};
constexpr int servoMaxUs[2] = {2100, 2100};
constexpr bool discActiveLow = true; // Check the actual TCRT5000 D0 polarity.
constexpr uint32_t leaseMaxMs = 250;
constexpr uint32_t motorFrequency = 20000;
constexpr uint8_t motorBits = 8;
constexpr uint32_t servoFrequency = 50;
constexpr uint8_t servoBits = 14;
constexpr bool hardwareEnabled = HARDWARE_OUTPUT_ENABLED == 1;
static_assert(HARDWARE_OUTPUT_ENABLED == 0 || HARDWARE_OUTPUT_ENABLED == 1, "Use 0 or 1");
static_assert(ROBOT_SERVO_COUNT == 1 || ROBOT_SERVO_COUNT == 2, "Invalid servo count");
static_assert(maxPwm > 0 && maxPwm <= 255, "Invalid PWM limit");
static_assert(motorDirection[0] * motorDirection[0] == 1 && motorDirection[1] * motorDirection[1] == 1, "Motor polarity must be +/-1");
constexpr bool reserved(uint8_t pin) { return pin == 2 || pin == 8 || pin == 9 || pin == 18 || pin == 19; }
constexpr bool pinsValid() {
  const uint8_t all[] = {motorPins[0], motorPins[1], motorPins[2], motorPins[3], servoPins[0], servoPins[1], sensorPin};
  for (unsigned i = 0; i < 7; ++i) {
    if (reserved(all[i])) return false;
    for (unsigned j = i + 1; j < 7; ++j) if (all[i] == all[j]) return false;
  }
  return true;
}
static_assert(pinsValid(), "Pin collision or USB/boot pin in use");
static_assert(servoMinUs[0] >= 900 && servoMaxUs[0] <= 2100 && servoMinUs[0] <= servoMaxUs[0], "Invalid servo 0 endpoints");
static_assert(servoMinUs[1] >= 900 && servoMaxUs[1] <= 2100 && servoMinUs[1] <= servoMaxUs[1], "Invalid servo 1 endpoints");
} // namespace config

