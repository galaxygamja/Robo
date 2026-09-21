// Arduino IDE entry point. Open this file, then edit the RobotSettings.h tab.
// Board: ESP32C3 Dev Module; esp32 core 2.0.17; ArduinoJson 6.21.5.
#include "RobotSettings.h"
#include "src/firmware_entry.h"

void setup() {
  roboSetup();
}

void loop() {
  roboLoop();
}
