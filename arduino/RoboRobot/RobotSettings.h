#pragma once

// Select ONE physical robot before uploading to ONE connected board.
// 1 = Hamster H1 (SM-S4303R drive servos + MG90S gate)
// 2 = Beaver B1 (N20 DC motors + DRV8833)
// 3 = Beaver B2 (N20 DC motors + DRV8833)
// 4 = Beaver B3 (N20 DC motors + DRV8833; legacy network ID H2)
#ifndef ROBOT_PROFILE
#define ROBOT_PROFILE 1
#endif

// Optional local credentials/settings; excluded from Git and release archives.
// Copy Secrets.example.h to Secrets.h, then edit your own Wi-Fi and token.
#if __has_include("Secrets.h")
#include "Secrets.h"
#endif

// Keep BOTH at 0 during initial USB upload / network checks.
// Enable only after the wiring, supply voltage, safe pulse limits, neutral,
// motor direction and wheels-raised test conditions have been checked.
#ifndef HARDWARE_OUTPUT_ENABLED
#define HARDWARE_OUTPUT_ENABLED 0
#endif
#ifndef ACTUATOR_CALIBRATION_CONFIRMED
#define ACTUATOR_CALIBRATION_CONFIRMED 0
#endif

// Calibration overrides can be added here or in the local Secrets.h.
// Defaults are EXAMPLES, not measured values for your servos.
// DRIVE_LEFT_NEUTRAL_US / DRIVE_RIGHT_NEUTRAL_US
// DRIVE_LEFT_MIN_US / DRIVE_LEFT_MAX_US
// DRIVE_RIGHT_MIN_US / DRIVE_RIGHT_MAX_US
// DRIVE_LEFT_DIRECTION / DRIVE_RIGHT_DIRECTION
// For all supported settings, see docs/ARDUINO_START_HERE_KO.md.
