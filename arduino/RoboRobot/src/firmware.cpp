// Generated from firmware/src/main.cpp; edit the canonical source.
#include "../RobotSettings.h"
#include <Arduino.h>
#include <esp_arduino_version.h>
#if !defined(ARDUINO_ARCH_ESP32) || !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "This firmware requires an ESP32-C3 board; do not upload it to Uno/Nano/classic ESP32."
#endif
#if ESP_ARDUINO_VERSION != ESP_ARDUINO_VERSION_VAL(2, 0, 17)
#error "Install esp32 by Espressif Systems version 2.0.17; this LEDC driver is not compatible with core 3.x."
#endif
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "robot_config.h"
#include "lease_state.h"
#include "json_envelope.h"
#include "drive_output.h"

namespace {
WiFiUDP udp;
SemaphoreHandle_t lockHandle;
LeaseState lease;
char bootId[33] = {};
char permit[33] = {};
uint32_t permitIssued = 0;
IPAddress peerIp;
uint16_t peerPort = 0;
bool peerKnown = false;
bool networkStarted = false;
int appliedPwm[2] = {0, 0};
int appliedServo[2] = {0, 0};
int appliedDriveServo[2] = {0, 0};

struct Guard {
  Guard() { xSemaphoreTake(lockHandle, portMAX_DELAY); }
  ~Guard() { xSemaphoreGive(lockHandle); }
};

void randomHex(char (&out)[33]) {
  static constexpr char hex[] = "0123456789abcdef";
  uint8_t bytes[16];
  esp_fill_random(bytes, sizeof(bytes));
  for (size_t i = 0; i < sizeof(bytes); ++i) {
    out[i * 2] = hex[bytes[i] >> 4];
    out[i * 2 + 1] = hex[bytes[i] & 15];
  }
  out[32] = 0;
}

bool safeToken(const char* value, size_t minimum, size_t maximum) {
  if (!value) return false;
  const size_t length = strlen(value);
  if (length < minimum || length > maximum) return false;
  for (size_t i = 0; i < length; ++i) {
    const char c = value[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
  }
  return true;
}

bool equalSecret(const char* a, const char* b) {
  if (!a || !b) return false;
  const size_t len = strlen(a);
  if (len != strlen(b)) return false;
  uint8_t difference = 0;
  for (size_t i = 0; i < len; ++i) difference |= uint8_t(a[i]) ^ uint8_t(b[i]);
  return difference == 0;
}

void zeroOutputs() {
  const unsigned driveChannels = config::continuousDrive ? 2 : 4;
  for (unsigned channel = 0; channel < driveChannels; ++channel) ledcWrite(channel, 0);
  for (unsigned i = 0; i < config::toolServoCount; ++i) ledcWrite(4 + i, 0);
  appliedPwm[0] = appliedPwm[1] = 0;
  appliedServo[0] = appliedServo[1] = 0;
  appliedDriveServo[0] = appliedDriveServo[1] = 0;
}

void disarm() {
  zeroOutputs();
  lease.disarm();
  permit[0] = 0;
}

void checkExpiry(uint32_t now) {
  if (lease.tick(now)) { zeroOutputs(); permit[0] = 0; }
}

void safetyTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    {
      Guard guard;
      checkExpiry(millis());
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(2) > 0 ? pdMS_TO_TICKS(2) : 1);
  }
}

void issuePermit(uint32_t now) { randomHex(permit); permitIssued = now; }

void writeOutputs(int left, int right, int servo0, int servo1) {
  if (!config::hardwareEnabled) { zeroOutputs(); return; }
  const int wanted[2] = {left, right};
  for (unsigned motor = 0; motor < 2; ++motor) {
    const drive_output::Calibration calibration = {
        config::maxPwm, config::motorDirection[motor], config::driveServoMinUs[motor],
        config::driveServoNeutralUs[motor], config::driveServoMaxUs[motor]};
    const auto output = drive_output::calculate(config::profile.drive, true, wanted[motor], calibration);
    if (config::continuousDrive) {
      // H1 SM-S4303R: signal straight to the servo, NOT via an H-bridge.
      // Zero logical speed is neutral only during a valid leased command.
      ledcWrite(motor, drive_output::servoDuty(output.pulseUs));
    } else {
      // Beaver DRV8833: clear both inputs before changing direction. Low/low
      // coasts; neither this stop nor loss of servo pulses is a holding brake.
      ledcWrite(motor * 2, 0);
      ledcWrite(motor * 2 + 1, 0);
      ledcWrite(motor * 2, output.forwardDuty);
      ledcWrite(motor * 2 + 1, output.reverseDuty);
    }
    appliedPwm[motor] = output.logicalPwm;
    appliedDriveServo[motor] = output.pulseUs;
  }
  const int servos[2] = {servo0, servo1};
  for (unsigned i = 0; i < 2; ++i) {
    const int us = i < config::toolServoCount ? servos[i] : 0;
    if (i < config::toolServoCount) ledcWrite(4 + i, drive_output::servoDuty(us));
    appliedServo[i] = us;
  }
}

bool currentPeer(const IPAddress& ip, uint16_t port) {
  return peerKnown && ip == peerIp && port == peerPort;
}

bool integer(JsonVariantConst value, int32_t minimum, int32_t maximum) {
  return !value.is<bool>() && value.is<int32_t>() && value.as<int32_t>() >= minimum && value.as<int32_t>() <= maximum;
}

bool keysAllowed(JsonObjectConst object, const char* type) {
  const char* base[] = {"protocol", "version", "robot_id", "token", "request_id", "type"};
  const char* arm[] = {"boot_id", "permit", "seq"};
  const char* command[] = {"ttl_ms", "left_pwm", "right_pwm", "servo_us"};
  const bool isArm = strcmp(type, "arm") == 0;
  const bool isCommand = strcmp(type, "command") == 0;
  for (JsonPairConst pair : object) {
    bool allowed = false;
    for (const char* key : base) if (strcmp(pair.key().c_str(), key) == 0) allowed = true;
    if (isArm || isCommand) for (const char* key : arm) if (strcmp(pair.key().c_str(), key) == 0) allowed = true;
    if (isCommand) for (const char* key : command) if (strcmp(pair.key().c_str(), key) == 0) allowed = true;
    // Emergency stop remains valid even if the sender includes a stale sequence.
    if ((strcmp(type, "stop") == 0 || strcmp(type, "estop") == 0) && strcmp(pair.key().c_str(), "seq") == 0) allowed = true;
    if (!allowed) return false;
  }
  return true;
}

void makeReply(JsonDocument& reply, const char* requestId, bool accepted, const char* reason, uint32_t now) {
  reply["protocol"] = "robo-hw";
  reply["version"] = 1;
  reply["type"] = "response";
  reply["robot_id"] = config::robotId;
  reply["firmware_version"] = config::firmwareVersion;
  reply["display_id"] = config::displayId;
  reply["robot_role"] = config::profile.role;
  reply["drive_type"] = robot_profiles::driveName(config::profile.drive);
  reply["max_pwm"] = config::maxPwm;
  reply["request_id"] = requestId;
  reply["boot_id"] = bootId;
  reply["permit"] = permit;
  reply["state"] = lease.name();
  reply["accepted"] = accepted;
  reply["reason"] = reason;
  reply["seq"] = lease.seq;
  reply["lease_remaining_ms"] = lease.replyRemaining(now, permitIssued, permit[0] != 0);
  reply["left_pwm"] = appliedPwm[0];
  reply["right_pwm"] = appliedPwm[1];
  JsonArray servo = reply.createNestedArray("servo_us");
  servo.add(appliedServo[0]); servo.add(appliedServo[1]);
  JsonArray driveServo = reply.createNestedArray("drive_servo_us");
  driveServo.add(appliedDriveServo[0]); driveServo.add(appliedDriveServo[1]);
  if (config::hasDiscSensor) reply["disc_present"] = bool(digitalRead(config::sensorPin) == (config::discActiveLow ? LOW : HIGH));
  else reply["disc_present"] = nullptr;
  reply["hardware_enabled"] = config::hardwareEnabled;
  reply["calibration_confirmed"] = config::calibrationConfirmed;
  JsonObject capabilities = reply.createNestedObject("capabilities");
  capabilities["tool_servos"] = config::toolServoCount;
  capabilities["disc_sensor"] = config::hasDiscSensor;
  capabilities["strafe"] = false;
  reply["uptime_ms"] = now;
}

const char* handleRequest(JsonObjectConst request, const IPAddress& ip, uint16_t port, uint32_t now) {
  const char* type = request["type"].as<const char*>();
  // Authenticated emergency commands win even if other fields are malformed.
  if (type && (strcmp(type, "stop") == 0 || strcmp(type, "estop") == 0)) {
    disarm();
    if (strcmp(type, "estop") == 0) lease.estop();
    return nullptr;
  }
  if (!type || !integer(request["version"], 1, 1) || strcmp(request["protocol"] | "", "robo-hw") != 0 || !keysAllowed(request, type)) return "invalid_envelope";
  if (strcmp(type, "hello") == 0) {
    disarm();
    peerIp = ip; peerPort = port; peerKnown = true;
    if (lease.mode == LeaseState::Mode::Estop) return "estop_latched";
    issuePermit(now);
    return nullptr;
  }
  if (!currentPeer(ip, port)) return "peer_mismatch";
  if (strcmp(type, "status") == 0) return nullptr;
  if (lease.mode == LeaseState::Mode::Estop) return "estop_latched";
  const bool isArm = strcmp(type, "arm") == 0;
  const bool isCommand = strcmp(type, "command") == 0;
  if (!isArm && !isCommand) return "unknown_type";
  if (!equalSecret(request["boot_id"].as<const char*>(), bootId)) return "boot_mismatch";
  if (!permit[0] || !equalSecret(request["permit"].as<const char*>(), permit)) return "permit_mismatch";
  if (uint32_t(now - permitIssued) >= config::leaseMaxMs) return "permit_expired";
  if (!integer(request["seq"], 1, INT32_MAX) || request["seq"].as<int32_t>() <= lease.seq) return "sequence_rejected";
  const int32_t seq = request["seq"].as<int32_t>();
  if (isArm) {
    if (!lease.arm(now, seq)) return "arm_rejected";
    zeroOutputs(); issuePermit(now); return nullptr;
  }
  if (!integer(request["ttl_ms"], 1, config::leaseMaxMs) || !integer(request["left_pwm"], -config::maxPwm, config::maxPwm) || !integer(request["right_pwm"], -config::maxPwm, config::maxPwm)) return "command_out_of_range";
  JsonArrayConst servos = request["servo_us"].as<JsonArrayConst>();
  if (servos.isNull() || servos.size() != 2) return "invalid_servo_array";
  for (unsigned i = 0; i < 2; ++i) {
    if (!integer(servos[i], 0, 2100)) return "invalid_servo";
    const int value = servos[i].as<int>();
    if (i >= config::toolServoCount && value != 0) return "unused_servo";
    if (value && (value < config::servoMinUs[i] || value > config::servoMaxUs[i])) return "servo_out_of_range";
  }
  if (!lease.command(now, permitIssued, request["ttl_ms"].as<uint32_t>(), seq)) return "lease_rejected";
  writeOutputs(request["left_pwm"].as<int>(), request["right_pwm"].as<int>(), servos[0].as<int>(), servos[1].as<int>());
  issuePermit(now);
  return nullptr;
}

void processPacket() {
  const int packetSize = udp.parsePacket();
  if (!packetSize) return;
  const IPAddress ip = udp.remoteIP();
  const uint16_t port = udp.remotePort();
  // Only loop() calls this function. Fixed static storage keeps the JSON and
  // UDP buffers off the small Arduino loop-task stack; the safety task never
  // reads them. Clear reply storage explicitly for each authenticated reply.
  static char bytes[config::packetMax + 1];
  const int read = udp.read(bytes, config::packetMax);
  udp.flush();
  static StaticJsonDocument<2048> request;
  const bool sized = packetSize <= int(config::packetMax) && read == packetSize && read > 0;
  const auto error = sized ? deserializeJson(request, static_cast<const char*>(bytes), size_t(read), DeserializationOption::NestingLimit(3)) : DeserializationError(DeserializationError::InvalidInput);
  const bool validJson = sized && !error && request.is<JsonObject>() && envelopeShape(bytes, read, request.size());
  static StaticJsonDocument<2048> reply;
  reply.clear();
  bool shouldReply = false;
  {
    Guard guard;
    const uint32_t now = millis();
    checkExpiry(now); // Never let packet traffic postpone a stop.
    if (!validJson) { if (currentPeer(ip, port)) disarm(); return; }
    JsonObjectConst object = request.as<JsonObjectConst>();
    for (JsonPairConst pair : object) {
      if (pair.value().is<const char*>() && pair.value().as<JsonString>().size() != strlen(pair.value().as<const char*>())) {
        if (currentPeer(ip, port)) disarm();
        return;
      }
    }
    if (!equalSecret(object["token"].as<const char*>(), ROBOT_SHARED_TOKEN) || strcmp(object["robot_id"] | "", config::robotId) != 0) return;
    const char* requestId = object["request_id"].as<const char*>();
    const char* type = object["type"].as<const char*>();
    const bool emergency = type && (strcmp(type, "stop") == 0 || strcmp(type, "estop") == 0);
    const char* reason = nullptr;
    if (!safeToken(requestId, 1, 64) && !emergency) reason = "invalid_request_id";
    else reason = handleRequest(object, ip, port, now);
    if (reason && currentPeer(ip, port)) disarm();
    makeReply(reply, safeToken(requestId, 1, 64) ? requestId : "", reason == nullptr, reason ? reason : "ok", now);
    shouldReply = true;
  }
  if (shouldReply) {
    static char output[config::replyMax];
    if (reply.overflowed() || measureJson(reply) >= sizeof(output)) return;
    const size_t written = serializeJson(reply, output, sizeof(output));
    udp.beginPacket(ip, port); udp.write(reinterpret_cast<uint8_t*>(output), written); udp.endPacket();
  }
}
} // namespace

void roboSetup() {
  // Avoid boot-time motion or a servo centering pulse.
  // Only configure pins present on this profile; H1 GPIO4/6/10 stay unused.
  if (config::continuousDrive) {
    for (uint8_t pin : config::driveServoPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  } else {
    for (uint8_t pin : config::motorPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  }
  for (unsigned i = 0; i < config::toolServoCount; ++i) { pinMode(config::servoPins[i], OUTPUT); digitalWrite(config::servoPins[i], LOW); }
  if (config::hasDiscSensor) pinMode(config::sensorPin, INPUT_PULLUP);
  if (config::continuousDrive) {
    for (unsigned i = 0; i < 2; ++i) { ledcSetup(i, config::servoFrequency, config::servoBits); ledcAttachPin(config::driveServoPins[i], i); ledcWrite(i, 0); }
  } else {
    for (unsigned i = 0; i < 4; ++i) { ledcSetup(i, config::motorFrequency, config::motorBits); ledcAttachPin(config::motorPins[i], i); ledcWrite(i, 0); }
  }
  for (unsigned i = 0; i < config::toolServoCount; ++i) { ledcSetup(i + 4, config::servoFrequency, config::servoBits); ledcAttachPin(config::servoPins[i], i + 4); ledcWrite(i + 4, 0); }
  zeroOutputs();
  Serial.begin(115200);
  lockHandle = xSemaphoreCreateMutex();
  if (!lockHandle) { for (;;) delay(1000); }
  randomHex(bootId);
  if (xTaskCreate(safetyTask, "motor-safety", 3072, nullptr, configMAX_PRIORITIES - 2, nullptr) != pdPASS) { for (;;) delay(1000); }
  if (!safeToken(ROBOT_SHARED_TOKEN, 24, 64) || strcmp(ROBOT_SHARED_TOKEN, "CHANGE_ME") == 0 || strcmp(WIFI_SSID, "CHANGE_ME") == 0 || strlen(WIFI_PASSWORD) < 8 || strcmp(WIFI_PASSWORD, "CHANGE_ME") == 0) {
    Serial.println("SAFE IDLE: configure include/secrets.h with Wi-Fi and a random token.");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("%s (wire %s), firmware %s, %s; hardware outputs %s; calibration %s.\n", config::displayId, config::robotId, config::firmwareVersion, robot_profiles::driveName(config::profile.drive), config::hardwareEnabled ? "ENABLED" : "DISABLED", config::calibrationConfirmed ? "ACKNOWLEDGED" : "NOT CONFIRMED");
}

void roboLoop() {
  {
    Guard guard;
    checkExpiry(millis());
    if (WiFi.status() != WL_CONNECTED) disarm();
  }
  if (WiFi.status() == WL_CONNECTED) {
    if (!networkStarted) {
      networkStarted = udp.begin(config::udpPort) == 1;
      if (networkStarted) Serial.printf("%s (wire %s) UDP %s:%u\n", config::displayId, config::robotId, WiFi.localIP().toString().c_str(), config::udpPort);
    }
    if (networkStarted) processPacket(); // One datagram per loop, bounded to 1024 B.
  } else if (networkStarted) { udp.stop(); networkStarted = false; }
  delay(1); // The independent watchdog gets CPU even under network flood.
}
