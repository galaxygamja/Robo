#pragma once

// Copy to secrets.h. Never commit the real Wi-Fi password or token.
#define WIFI_SSID "CHANGE_ME"
#define WIFI_PASSWORD "CHANGE_ME"
// Generate a random token, >=24 ASCII letters/digits/underscore/hyphen.
#define ROBOT_SHARED_TOKEN "CHANGE_ME"
// Select robot in PlatformIO environment or Arduino RobotSettings.h.
// Do not override ROBOT_PROFILE here when building several PlatformIO robots.
// Keep both 0 for USB/network checks. Both must be 1 for physical output.
// Verify wiring/power and calibrate drive neutral/direction and MG90S travel
// with wheels raised and mechanisms unloaded before normal operation.
#define HARDWARE_OUTPUT_ENABLED 0
#define ACTUATOR_CALIBRATION_CONFIRMED 0

// Optional per-robot overrides; defaults are NOT measurements:
// #define DRIVE_LEFT_DIRECTION 1
// #define DRIVE_RIGHT_DIRECTION -1  // H1 default; Beavers default +1.
// #define DRIVE_LEFT_MIN_US 1380
// #define DRIVE_LEFT_NEUTRAL_US 1500
// #define DRIVE_LEFT_MAX_US 1620
// #define DRIVE_RIGHT_MIN_US 1380
// #define DRIVE_RIGHT_NEUTRAL_US 1500
// #define DRIVE_RIGHT_MAX_US 1620
// #define TOOL0_MIN_US 900
// #define TOOL0_MAX_US 2100
// #define TOOL1_MIN_US 900
// #define TOOL1_MAX_US 2100

