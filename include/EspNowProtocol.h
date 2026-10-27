#pragma once

#include <stdint.h>

// Shared wire protocol between the OLED controller display and the remote
// load-cell node. Both devices communicate over ESP-NOW and must agree on
// these exact packet layouts, so this header is the single source of truth.
namespace LoadCellEspNow {

// Commands the display sends to the load-cell node.
enum class CommandType : uint8_t {
  None = 0,
  Tare = 1,
  Calibrate = 2,
  ClearLock = 3,
};

// Operating state the load-cell node reports back to the display.
enum class ScaleStatus : uint8_t {
  Measuring = 0,
  Stable = 1,
  Taring = 2,
  Calibrating = 3,
  CalibrationComplete = 4,
  Error = 5,
};

// Telemetry: load-cell node -> display.
struct __attribute__((packed)) WeightPacket {
  float weightKg;
  uint8_t locked;
  uint8_t status;
};

// Command: display -> load-cell node.
struct __attribute__((packed)) CommandPacket {
  float knownWeightKg;
  uint8_t command;
};

} // namespace LoadCellEspNow
