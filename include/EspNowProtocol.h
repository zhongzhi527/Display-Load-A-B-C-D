#pragma once

#include <stdint.h>

// Shared ESP-NOW wire protocol between the handheld display and the load-cell
// nodes (A/B). Every device must agree on these exact layouts, so this header
// is kept identical to the copy in the ESP32-Load-A-B-C-D project.

enum CommandType : uint8_t {
  COMMAND_NONE = 0,
  COMMAND_TARE = 1,
  COMMAND_CALIBRATE = 2,
  COMMAND_CLEAR_LOCK = 3,
  COMMAND_SET_LOCK_THRESHOLD = 4,
  COMMAND_SET_AUTO_LOCK = 5,
  COMMAND_SET_STABILITY_SECONDS = 6
};

enum CommandResult : uint8_t {
  COMMAND_RESULT_NONE = 0,
  COMMAND_RESULT_OK = 1,
  COMMAND_RESULT_INVALID = 2,
  COMMAND_RESULT_FAILED = 3
};

enum ScaleStatus : uint8_t {
  STATUS_MEASURING = 0,
  STATUS_STABLE = 1,
  STATUS_TARING = 2,
  STATUS_CALIBRATING = 3,
  STATUS_CALIBRATION_COMPLETE = 4,
  STATUS_ERROR = 5
};

// Telemetry: load-cell node -> display.
struct __attribute__((packed)) WeightPacket {
  uint8_t nodeId;
  float weightKg;
  uint8_t locked;
  uint8_t status;
  uint8_t lastCommand;
  uint8_t commandResult;
  float minLockWeightKg;
  uint8_t autoLockEnabled;
  uint16_t stabilityTimeMs;
};

// Command: display -> load-cell node.
struct __attribute__((packed)) CommandPacket {
  float value;
  uint8_t command;
};
