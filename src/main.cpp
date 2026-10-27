// Handheld touchscreen display for the wireless dual load-cell scale.
//
// A Guition JC2432W328C (240x320 ST7789 TFT + CST820 touch) shows the live
// weight of two load-cell nodes (A and B) plus their total, and sends
// tare / calibrate / settings commands to them over ESP-NOW.
//
// Pairs with the load-cell node project ESP32-Load-A-B-C-D.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <Wire.h>
#include <Preferences.h>
#include <LovyanGFX.hpp>

#include <EspNowProtocol.h>

constexpr int TFT_BL_PIN = 27;
constexpr int TOP_BUTTON_PIN = 0;  // BOOT / top physical button
constexpr int TOUCH_SDA_PIN = 33;
constexpr int TOUCH_SCL_PIN = 32;
constexpr int TOUCH_RST_PIN = 25;
constexpr int TOUCH_INT_PIN = 21;
constexpr uint8_t CST820_ADDRESS = 0x15;

constexpr unsigned long CONNECTION_TIMEOUT_MS = 2500;
constexpr unsigned long TOUCH_DEBOUNCE_MS = 220;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 40;
constexpr unsigned long BUTTON_LONG_PRESS_MS = 2000;

// MAC addresses of the two load-cell nodes (A and B).
uint8_t nodeMac[2][6] = {
  {0xE0, 0xE2, 0xE6, 0xD7, 0x3E, 0xD4}, // A
  {0xE0, 0xE2, 0xE6, 0xCC, 0xFE, 0xE0}  // B
};

enum Screen : uint8_t {
  SCREEN_MAIN,
  SCREEN_CALIBRATE,
  SCREEN_CAL_WEIGHT,
  SCREEN_CAL_REMOVE,
  SCREEN_CAL_PLACE,
  SCREEN_CAL_DONE,
  SCREEN_EXIT_CONFIRM,
  SCREEN_SETTINGS
};

struct NodeData {
  float weightKg = 0.0f;
  uint8_t status = STATUS_MEASURING;
  bool locked = false;
  unsigned long lastPacketMs = 0;
  uint8_t lastCommand = COMMAND_NONE;
  uint8_t commandResult = COMMAND_RESULT_NONE;
};

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 panel;
  lgfx::Bus_SPI bus;

public:
  LGFX() {
    auto busCfg = bus.config();
    busCfg.spi_host = VSPI_HOST;
    busCfg.spi_mode = 0;
    busCfg.freq_write = 27000000;
    busCfg.freq_read = 16000000;
    busCfg.spi_3wire = false;
    busCfg.use_lock = true;
    busCfg.dma_channel = 1;
    busCfg.pin_sclk = 14;
    busCfg.pin_mosi = 13;
    busCfg.pin_miso = -1;
    busCfg.pin_dc = 2;
    bus.config(busCfg);
    panel.setBus(&bus);

    auto panelCfg = panel.config();
    panelCfg.pin_cs = 15;
    panelCfg.pin_rst = -1;
    panelCfg.pin_busy = -1;
    panelCfg.panel_width = 240;
    panelCfg.panel_height = 320;
    panelCfg.offset_x = 0;
    panelCfg.offset_y = 0;
    panelCfg.offset_rotation = 0;
    panelCfg.dummy_read_pixel = 8;
    panelCfg.dummy_read_bits = 1;
    panelCfg.readable = false;
    panelCfg.invert = false;
    panelCfg.rgb_order = false;
    panelCfg.dlen_16bit = false;
    panelCfg.bus_shared = true;
    panel.config(panelCfg);

    setPanel(&panel);
  }
};

LGFX lcd;
Preferences preferences;
NodeData nodes[2];

Screen screen = SCREEN_MAIN;
Screen screenBeforeExitConfirm = SCREEN_MAIN;
uint8_t selectedNode = 0;
float calibrationWeightKg = 5.0f;
bool waitingForTare = false;
bool waitingForCalibration = false;
bool staticScreenRequired = true;
bool espNowReady = false;

float settingLockThresholdKg = 5.0f;
bool settingAutoLock = true;
uint8_t settingBrightnessPercent = 100;
uint8_t settingDecimals = 3;

unsigned long lastTouchMs = 0;
unsigned long doneUntilMs = 0;

bool buttonLastReading = HIGH;
bool buttonStableState = HIGH;
unsigned long buttonChangedMs = 0;
unsigned long buttonPressedMs = 0;
bool buttonLongHandled = false;

float paintedWeight[2] = {NAN, NAN};
float paintedTotal = NAN;
uint8_t paintedStatus[2] = {255, 255};
bool paintedConnected[2] = {false, false};
bool paintedLocked[2] = {false, false};
bool paintedHeaderConnected[2] = {false, false};
bool paintedHeaderValid = false;

void setBacklightPercent(uint8_t percent) {
  percent = constrain(percent, 0, 100);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  static bool attached = false;
  if (!attached) {
    ledcAttach(TFT_BL_PIN, 5000, 8);
    attached = true;
  }
  ledcWrite(TFT_BL_PIN, map(percent, 0, 100, 0, 255));
#else
  static bool attached = false;
  if (!attached) {
    ledcSetup(0, 5000, 8);
    ledcAttachPin(TFT_BL_PIN, 0);
    attached = true;
  }
  ledcWrite(0, map(percent, 0, 100, 0, 255));
#endif
}

bool nodeConnected(uint8_t i) {
  return nodes[i].lastPacketMs != 0 &&
         millis() - nodes[i].lastPacketMs <= CONNECTION_TIMEOUT_MS;
}

float totalWeightKg() {
  float total = 0.0f;
  if (nodeConnected(0)) total += nodes[0].weightKg;
  if (nodeConnected(1)) total += nodes[1].weightKg;
  return total;
}

const char* statusText(uint8_t i) {
  if (!nodeConnected(i)) return "OFFLINE";
  if (nodes[i].locked) return "LOCKED";
  switch (nodes[i].status) {
    case STATUS_STABLE: return "STABLE";
    case STATUS_TARING: return "TARING";
    case STATUS_CALIBRATING: return "CALIBRATING";
    case STATUS_CALIBRATION_COMPLETE: return "CAL COMPLETE";
    case STATUS_ERROR: return "ERROR";
    default: return "STABILIZING";
  }
}

uint32_t statusColor(uint8_t i) {
  if (!nodeConnected(i)) return TFT_RED;
  if (nodes[i].locked || nodes[i].status == STATUS_STABLE) return TFT_GREEN;
  switch (nodes[i].status) {
    case STATUS_TARING: return TFT_BLUE;
    case STATUS_CALIBRATING: return TFT_ORANGE;
    case STATUS_CALIBRATION_COMPLETE: return TFT_GREEN;
    case STATUS_ERROR: return TFT_RED;
    default: return TFT_YELLOW;
  }
}

void registerActivity() {
  setBacklightPercent(settingBrightnessPercent);
}

void resetPaintCache() {
  paintedWeight[0] = paintedWeight[1] = NAN;
  paintedTotal = NAN;
  paintedStatus[0] = paintedStatus[1] = 255;
  paintedConnected[0] = paintedConnected[1] = false;
  paintedLocked[0] = paintedLocked[1] = false;
  paintedHeaderValid = false;
}

void changeScreen(Screen next) {
  screen = next;
  staticScreenRequired = true;
  resetPaintCache();
}

void drawButton(int x, int y, int w, int h, const char* label,
                uint32_t color, bool enabled = true) {
  uint32_t c = enabled ? color : TFT_DARKGREY;
  lcd.fillRoundRect(x, y, w, h, 7, c);
  lcd.drawRoundRect(x, y, w, h, 7, enabled ? TFT_WHITE : TFT_LIGHTGREY);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(enabled ? TFT_WHITE : TFT_LIGHTGREY, c);
  lcd.setTextSize(1);
  lcd.drawString(label, x + w / 2, y + h / 2);
}

void updateHeaderIndicators(bool force = false) {
  for (uint8_t i = 0; i < 2; i++) {
    bool connected = nodeConnected(i);
    if (!force && paintedHeaderValid &&
        paintedHeaderConnected[i] == connected) {
      continue;
    }

    int x = i == 0 ? 186 : 212;
    lcd.fillRect(x, 3, 24, 22, TFT_DARKGREY);
    lcd.setTextDatum(middle_center);
    lcd.setTextSize(1);
    lcd.setTextColor(connected ? TFT_GREEN : TFT_RED, TFT_DARKGREY);
    lcd.drawString(i == 0 ? "A" : "B", x + 12, 14);
    paintedHeaderConnected[i] = connected;
  }
  paintedHeaderValid = true;
}

void drawHeader(const char* title) {
  lcd.fillRect(0, 0, 240, 28, TFT_DARKGREY);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_WHITE, TFT_DARKGREY);
  lcd.setTextSize(2);
  lcd.drawString(title, 120, 14);
  updateHeaderIndicators(true);
}

void formatWeight(char* buffer, size_t len, float value, bool connected) {
  if (!connected) {
    snprintf(buffer, len, "--");
  } else if (settingDecimals == 2) {
    snprintf(buffer, len, "%.2f", value);
  } else {
    snprintf(buffer, len, "%.3f", value);
  }
}

// Y position of the top of each compact dashboard cell row. Rows A and B are
// real nodes (nodes[0]/nodes[1]); rows C and D are decorative placeholders
// that advertise the 4-cell capability but have no node behind them.
constexpr int CELL_ROW_Y[4] = {34, 80, 126, 172};
constexpr int CELL_ROW_H = 44;
constexpr int TOTAL_ROW_Y = 218;
constexpr int TOTAL_ROW_H = 44;

// Draw a decorative (C/D) cell row once: greyed out, always OFFLINE / "--".
// These rows are not tappable and never carry a node's data.
void drawDecorativeCell(const char* label, int y) {
  lcd.drawRoundRect(6, y, 228, CELL_ROW_H, 6, TFT_DARKGREY);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
  lcd.drawString(label, 14, y + 6);
  lcd.drawString("OFFLINE", 14, y + 24);

  lcd.setTextDatum(middle_right);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
  lcd.drawString("--", 206, y + CELL_ROW_H / 2);
  lcd.setTextSize(1);
  lcd.drawString("kg", 228, y + CELL_ROW_H / 2 + 1);
}

void drawMainStatic() {
  lcd.fillScreen(TFT_BLACK);
  drawHeader("WIRELESS SCALE");

  // Real cells A and B (updated live by updateMainNode).
  lcd.drawRoundRect(6, CELL_ROW_Y[0], 228, CELL_ROW_H, 6, TFT_DARKGREY);
  lcd.drawRoundRect(6, CELL_ROW_Y[1], 228, CELL_ROW_H, 6, TFT_DARKGREY);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("LOAD CELL A", 14, CELL_ROW_Y[0] + 6);
  lcd.drawString("LOAD CELL B", 14, CELL_ROW_Y[1] + 6);

  // Decorative cells C and D (drawn once, never updated).
  drawDecorativeCell("LOAD CELL C", CELL_ROW_Y[2]);
  drawDecorativeCell("LOAD CELL D", CELL_ROW_Y[3]);

  // Total row (A + B only).
  lcd.drawRoundRect(6, TOTAL_ROW_Y, 228, TOTAL_ROW_H, 6, TFT_CYAN);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString("TOTAL", 14, TOTAL_ROW_Y + 6);

  // Global action buttons.
  drawButton(6, 270, 111, 40, "TARE ALL", TFT_BLUE);
  drawButton(123, 270, 111, 40, "CALIBRATE", TFT_ORANGE);
  resetPaintCache();
}

void updateMainNode(uint8_t i) {
  int y = CELL_ROW_Y[i];
  bool connected = nodeConnected(i);
  bool changed =
    isnan(paintedWeight[i]) ||
    fabsf(paintedWeight[i] - nodes[i].weightKg) >= 0.001f ||
    paintedStatus[i] != nodes[i].status ||
    paintedConnected[i] != connected ||
    paintedLocked[i] != nodes[i].locked;

  if (!changed) return;

  // Clear the status line (under the label) and the weight area.
  lcd.fillRect(14, y + 23, 110, 14, TFT_BLACK);
  lcd.fillRect(110, y + 2, 122, CELL_ROW_H - 4, TFT_BLACK);

  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(statusColor(i), TFT_BLACK);
  lcd.drawString(statusText(i), 14, y + 24);

  char text[24];
  formatWeight(text, sizeof(text), nodes[i].weightKg, connected);
  lcd.setTextDatum(middle_right);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.drawString(text, 206, y + CELL_ROW_H / 2);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  lcd.drawString("kg", 228, y + CELL_ROW_H / 2 + 1);

  paintedWeight[i] = nodes[i].weightKg;
  paintedStatus[i] = nodes[i].status;
  paintedConnected[i] = connected;
  paintedLocked[i] = nodes[i].locked;
}

void updateMainTotal() {
  float total = totalWeightKg();
  if (!isnan(paintedTotal) && fabsf(total - paintedTotal) < 0.001f) return;

  lcd.fillRect(70, TOTAL_ROW_Y + 2, 162, TOTAL_ROW_H - 4, TFT_BLACK);
  char text[24];
  formatWeight(text, sizeof(text), total, true);

  lcd.setTextDatum(middle_right);
  lcd.setTextSize(3);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(text, 206, TOTAL_ROW_Y + TOTAL_ROW_H / 2);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString("kg", 228, TOTAL_ROW_Y + TOTAL_ROW_H / 2 + 1);
  paintedTotal = total;
}

// Calibration cell-picker page (Display-Load-A-B style). Pick A or B, adjust
// the known weight, tare the selected cell, or start the guided calibration.
// Only the two real cells are selectable.
void drawCalibrateStatic() {
  lcd.fillScreen(TFT_BLACK);
  drawHeader("CALIBRATE");

  // Cell selector (A / B). The selected cell is highlighted.
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("SELECT CELL", 14, 36);
  drawButton(6, 50, 111, 42, "CELL A",
             selectedNode == 0 ? TFT_GREEN : TFT_DARKGREY);
  drawButton(123, 50, 111, 42, "CELL B",
             selectedNode == 1 ? TFT_GREEN : TFT_DARKGREY);

  // Known weight with +/- adjust.
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("KNOWN WEIGHT", 14, 104);

  char text[20];
  snprintf(text, sizeof(text), "%.1f kg", calibrationWeightKg);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.setTextSize(3);
  lcd.drawString(text, 120, 153);

  drawButton(6, 132, 60, 42, "-", TFT_BLUE);
  drawButton(174, 132, 60, 42, "+", TFT_BLUE);

  // Per-cell actions.
  bool connected = nodeConnected(selectedNode);
  drawButton(6, 196, 111, 42, "TARE", TFT_BLUE, connected);
  drawButton(123, 196, 111, 42, "CALIBRATE", TFT_ORANGE, connected);

  // Navigation.
  drawButton(65, 254, 110, 40, "HOME", TFT_DARKGREY);
  resetPaintCache();
}

void drawSimpleMessage(const char* title, const char* line1,
                       const char* line2 = nullptr) {
  lcd.fillScreen(TFT_BLACK);
  drawHeader(title);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.drawString(line1, 120, 70);
  if (line2) {
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString(line2, 120, 120);
  }
}

void drawCalWeight() {
  drawSimpleMessage(selectedNode == 0 ? "CALIBRATE A" : "CALIBRATE B",
                    "Choose known weight");

  char text[20];
  snprintf(text, sizeof(text), "%.1f kg", calibrationWeightKg);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.setTextSize(4);
  lcd.drawString(text, 120, 165);

  drawButton(6, 230, 68, 54, "-", TFT_BLUE);
  drawButton(86, 230, 68, 54, "+", TFT_BLUE);
  drawButton(166, 230, 68, 54, "NEXT", TFT_GREEN);
}

void drawCalRemove() {
  drawSimpleMessage("STEP 1 OF 2",
                    waitingForTare ? "Taring..." : "Remove all weight",
                    waitingForTare ? "Please wait" : "Then press TARE");
  if (!waitingForTare) drawButton(30, 210, 180, 54, "TARE", TFT_BLUE);
}

void drawCalPlace() {
  char line[30];
  snprintf(line, sizeof(line), "Place %.1f kg", calibrationWeightKg);
  drawSimpleMessage("STEP 2 OF 2",
                    waitingForCalibration ? "Calibrating..." : line,
                    waitingForCalibration ? "Keep weight still" : "Then press START");
  if (!waitingForCalibration)
    drawButton(30, 210, 180, 54, "START", TFT_GREEN);
}

void drawCalDone() {
  drawSimpleMessage("CALIBRATION", "COMPLETE", "Calibration saved");
  drawButton(50, 210, 140, 50, "DONE", TFT_BLUE);
}

void drawExitConfirm() {
  drawSimpleMessage("EXIT CALIBRATION", "Return to main menu?",
                    "The current load-cell operation may finish.");
  drawButton(18, 210, 96, 54, "NO", TFT_DARKGREY);
  drawButton(126, 210, 96, 54, "YES", TFT_RED);
}

void drawSettings() {
  lcd.fillScreen(TFT_BLACK);
  drawHeader("SETTINGS");

  lcd.setTextDatum(middle_left);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(1);

  char line[48];
  snprintf(line, sizeof(line), "Lock above: %.1f kg", settingLockThresholdKg);
  lcd.drawString(line, 14, 58);
  drawButton(136, 42, 46, 32, "-", TFT_BLUE);
  drawButton(188, 42, 46, 32, "+", TFT_BLUE);

  snprintf(line, sizeof(line), "Auto-lock: %s", settingAutoLock ? "ON" : "OFF");
  lcd.drawString(line, 14, 110);
  drawButton(150, 94, 84, 32, "TOGGLE", TFT_DARKGREEN);

  snprintf(line, sizeof(line), "Brightness: %u%%", settingBrightnessPercent);
  lcd.drawString(line, 14, 162);
  drawButton(136, 146, 46, 32, "-", TFT_BLUE);
  drawButton(188, 146, 46, 32, "+", TFT_BLUE);

  snprintf(line, sizeof(line), "Decimals: %u", settingDecimals);
  lcd.drawString(line, 14, 214);
  drawButton(150, 198, 84, 32, "TOGGLE", TFT_BLUE);

  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString("Lock window: 1.0 kg for 4 seconds (fixed)", 14, 246);

  drawButton(6, 270, 111, 40, "HOME", TFT_DARKGREY);
  drawButton(123, 270, 111, 40, "SAVE", TFT_GREEN);
}

void renderScreen() {
  if (staticScreenRequired) {
    switch (screen) {
      case SCREEN_MAIN: drawMainStatic(); break;
      case SCREEN_CALIBRATE: drawCalibrateStatic(); break;
      case SCREEN_CAL_WEIGHT: drawCalWeight(); break;
      case SCREEN_CAL_REMOVE: drawCalRemove(); break;
      case SCREEN_CAL_PLACE: drawCalPlace(); break;
      case SCREEN_CAL_DONE: drawCalDone(); break;
      case SCREEN_EXIT_CONFIRM: drawExitConfirm(); break;
      case SCREEN_SETTINGS: drawSettings(); break;
    }
    staticScreenRequired = false;
  }

  if (screen == SCREEN_MAIN) {
    updateHeaderIndicators();
    updateMainNode(0);
    updateMainNode(1);
    updateMainTotal();
  } else if (screen == SCREEN_CALIBRATE) {
    updateHeaderIndicators();
  }
}

bool readTouch(uint16_t& x, uint16_t& y) {
  Wire.beginTransmission(CST820_ADDRESS);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(CST820_ADDRESS, (uint8_t)7) != (uint8_t)7) return false;

  uint8_t data[7];
  for (uint8_t i = 0; i < 7; i++) data[i] = Wire.read();
  if ((data[2] & 0x0F) == 0) return false;

  uint16_t rawX = ((data[3] & 0x0F) << 8) | data[4];
  uint16_t rawY = ((data[5] & 0x0F) << 8) | data[6];
  // Portrait (rotation 0): the CST820 axes map directly to the panel, so no
  // landscape swap/invert. x spans 0..239, y spans 0..319.
  x = rawX;
  y = rawY;
  return x <= 239 && y <= 319;
}

bool inside(uint16_t x, uint16_t y, int bx, int by, int bw, int bh) {
  return x >= bx && x < bx + bw && y >= by && y < by + bh;
}

bool addPeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void sendCommand(uint8_t node, uint8_t command, float value = 0.0f) {
  if (!espNowReady) return;
  CommandPacket packet{value, command};
  esp_now_send(nodeMac[node],
               reinterpret_cast<const uint8_t*>(&packet),
               sizeof(packet));
  registerActivity();
}

void sendSettingToBoth(uint8_t command, float value) {
  sendCommand(0, command, value);
  delay(30);
  sendCommand(1, command, value);
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataReceived(const esp_now_recv_info_t* info,
                    const uint8_t* data, int len) {
  const uint8_t* senderMac = info->src_addr;
#else
void onDataReceived(const uint8_t* senderMac,
                    const uint8_t* data, int len) {
#endif
  if (len != sizeof(WeightPacket)) return;

  WeightPacket packet{};
  memcpy(&packet, data, sizeof(packet));
  if (packet.nodeId < 1 || packet.nodeId > 2) return;

  uint8_t i = packet.nodeId - 1;
  if (memcmp(senderMac, nodeMac[i], 6) != 0) return;

  nodes[i].weightKg = packet.weightKg;
  nodes[i].locked = packet.locked != 0;
  nodes[i].status = packet.status;
  nodes[i].lastPacketMs = millis();
  nodes[i].lastCommand = packet.lastCommand;
  nodes[i].commandResult = packet.commandResult;

  if (i == selectedNode) {
    if (waitingForTare &&
        packet.lastCommand == COMMAND_TARE &&
        packet.commandResult == COMMAND_RESULT_OK) {
      waitingForTare = false;
      changeScreen(SCREEN_CAL_PLACE);
    }

    if (waitingForCalibration &&
        packet.lastCommand == COMMAND_CALIBRATE) {
      waitingForCalibration = false;
      if (packet.commandResult == COMMAND_RESULT_OK) {
        doneUntilMs = millis() + 2500;
        changeScreen(SCREEN_CAL_DONE);
      } else {
        changeScreen(SCREEN_CALIBRATE);
      }
    }
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.print("Display MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(onDataReceived);

  if (!addPeer(nodeMac[0]) || !addPeer(nodeMac[1])) {
    esp_now_deinit();
    return;
  }

  espNowReady = true;
}

bool calibrationScreenActive() {
  return screen == SCREEN_CAL_WEIGHT ||
         screen == SCREEN_CAL_REMOVE ||
         screen == SCREEN_CAL_PLACE;
}

void requestHome() {
  registerActivity();

  if (screen == SCREEN_MAIN) return;

  if (calibrationScreenActive()) {
    screenBeforeExitConfirm = screen;
    changeScreen(SCREEN_EXIT_CONFIRM);
  } else {
    waitingForTare = false;
    waitingForCalibration = false;
    changeScreen(SCREEN_MAIN);
  }
}

void handleTopButton() {
  bool reading = digitalRead(TOP_BUTTON_PIN);
  unsigned long now = millis();

  if (reading != buttonLastReading) {
    buttonLastReading = reading;
    buttonChangedMs = now;
  }

  if (now - buttonChangedMs < BUTTON_DEBOUNCE_MS) return;

  if (reading != buttonStableState) {
    buttonStableState = reading;

    if (buttonStableState == LOW) {
      buttonPressedMs = now;
      buttonLongHandled = false;
      registerActivity();
    } else {
      unsigned long held = now - buttonPressedMs;
      if (!buttonLongHandled && held < BUTTON_LONG_PRESS_MS) {
        requestHome();
      }
    }
  }

  if (buttonStableState == LOW &&
      !buttonLongHandled &&
      now - buttonPressedMs >= BUTTON_LONG_PRESS_MS) {
    buttonLongHandled = true;
    changeScreen(SCREEN_SETTINGS);
  }
}

void saveDisplaySettings() {
  preferences.putUChar("brightness", settingBrightnessPercent);
  preferences.putUChar("decimals", settingDecimals);
  preferences.putFloat("lockKg", settingLockThresholdKg);
  preferences.putBool("autoLock", settingAutoLock);

  sendSettingToBoth(COMMAND_SET_LOCK_THRESHOLD, settingLockThresholdKg);
  delay(40);
  sendSettingToBoth(COMMAND_SET_AUTO_LOCK, settingAutoLock ? 1.0f : 0.0f);

  setBacklightPercent(settingBrightnessPercent);
}

void handleTouch() {
  uint16_t x, y;
  if (!readTouch(x, y)) return;

  unsigned long now = millis();
  if (now - lastTouchMs < TOUCH_DEBOUNCE_MS) return;
  lastTouchMs = now;
  registerActivity();

  if (screen == SCREEN_MAIN) {
    // TARE ALL: tare both real nodes (A and B). C/D are decorative.
    if (inside(x, y, 6, 270, 111, 40)) {
      sendCommand(0, COMMAND_TARE);
      delay(30);
      sendCommand(1, COMMAND_TARE);
    } else if (inside(x, y, 123, 270, 111, 40)) {
      changeScreen(SCREEN_CALIBRATE);
    }
    return;
  }

  if (screen == SCREEN_CALIBRATE) {
    if (inside(x, y, 6, 50, 111, 42)) {
      selectedNode = 0;
      staticScreenRequired = true;
    } else if (inside(x, y, 123, 50, 111, 42)) {
      selectedNode = 1;
      staticScreenRequired = true;
    } else if (inside(x, y, 6, 132, 60, 42)) {
      calibrationWeightKg = max(0.1f, calibrationWeightKg - 0.1f);
      staticScreenRequired = true;
    } else if (inside(x, y, 174, 132, 60, 42)) {
      calibrationWeightKg = min(100.0f, calibrationWeightKg + 0.1f);
      staticScreenRequired = true;
    } else if (inside(x, y, 6, 196, 111, 42) &&
               nodeConnected(selectedNode)) {
      // Individual TARE: tare only the selected cell.
      sendCommand(selectedNode, COMMAND_TARE);
    } else if (inside(x, y, 123, 196, 111, 42) &&
               nodeConnected(selectedNode)) {
      // Enter the guided calibration wizard for the selected cell.
      changeScreen(SCREEN_CAL_WEIGHT);
    } else if (inside(x, y, 65, 254, 110, 40)) {
      changeScreen(SCREEN_MAIN);
    }
    return;
  }

  if (screen == SCREEN_CAL_WEIGHT) {
    if (inside(x, y, 6, 230, 68, 54)) {
      calibrationWeightKg = max(0.1f, calibrationWeightKg - 0.1f);
      staticScreenRequired = true;
    } else if (inside(x, y, 86, 230, 68, 54)) {
      calibrationWeightKg = min(100.0f, calibrationWeightKg + 0.1f);
      staticScreenRequired = true;
    } else if (inside(x, y, 166, 230, 68, 54)) {
      changeScreen(SCREEN_CAL_REMOVE);
    }
    return;
  }

  if (screen == SCREEN_CAL_REMOVE && !waitingForTare &&
      inside(x, y, 30, 210, 180, 54)) {
    waitingForTare = true;
    sendCommand(selectedNode, COMMAND_TARE);
    staticScreenRequired = true;
    return;
  }

  if (screen == SCREEN_CAL_PLACE && !waitingForCalibration &&
      inside(x, y, 30, 210, 180, 54)) {
    waitingForCalibration = true;
    sendCommand(selectedNode, COMMAND_CALIBRATE, calibrationWeightKg);
    staticScreenRequired = true;
    return;
  }

  if (screen == SCREEN_CAL_DONE &&
      inside(x, y, 50, 210, 140, 50)) {
    changeScreen(SCREEN_MAIN);
    return;
  }

  if (screen == SCREEN_EXIT_CONFIRM) {
    if (inside(x, y, 18, 210, 96, 54)) {
      changeScreen(screenBeforeExitConfirm);
    } else if (inside(x, y, 126, 210, 96, 54)) {
      waitingForTare = false;
      waitingForCalibration = false;
      changeScreen(SCREEN_MAIN);
    }
    return;
  }

  if (screen == SCREEN_SETTINGS) {
    if (inside(x, y, 136, 42, 46, 32)) {
      settingLockThresholdKg = max(0.0f, settingLockThresholdKg - 0.5f);
    } else if (inside(x, y, 188, 42, 46, 32)) {
      settingLockThresholdKg = min(100.0f, settingLockThresholdKg + 0.5f);
    } else if (inside(x, y, 150, 94, 84, 32)) {
      settingAutoLock = !settingAutoLock;
    } else if (inside(x, y, 136, 146, 46, 32)) {
      settingBrightnessPercent =
        max(25, (int)settingBrightnessPercent - 25);
      setBacklightPercent(settingBrightnessPercent);
    } else if (inside(x, y, 188, 146, 46, 32)) {
      settingBrightnessPercent =
        min(100, (int)settingBrightnessPercent + 25);
      setBacklightPercent(settingBrightnessPercent);
    } else if (inside(x, y, 150, 198, 84, 32)) {
      settingDecimals = settingDecimals == 2 ? 3 : 2;
    } else if (inside(x, y, 6, 270, 111, 40)) {
      changeScreen(SCREEN_MAIN);
    } else if (inside(x, y, 123, 270, 111, 40)) {
      saveDisplaySettings();
      changeScreen(SCREEN_MAIN);
    }
    if (screen == SCREEN_SETTINGS) staticScreenRequired = true;
  }
}

void setupTouch() {
  pinMode(TOUCH_RST_PIN, OUTPUT);
  pinMode(TOUCH_INT_PIN, INPUT);
  digitalWrite(TOUCH_RST_PIN, LOW);
  delay(10);
  digitalWrite(TOUCH_RST_PIN, HIGH);
  delay(100);
  Wire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN);
  Wire.setClock(400000);
}

void setup() {
  Serial.begin(115200);
  pinMode(TOP_BUTTON_PIN, INPUT_PULLUP);

  preferences.begin("display", false);
  settingBrightnessPercent = preferences.getUChar("brightness", 100);
  settingDecimals = preferences.getUChar("decimals", 3);
  settingLockThresholdKg = preferences.getFloat("lockKg", 5.0f);
  settingAutoLock = preferences.getBool("autoLock", true);

  lcd.init();
  lcd.setRotation(0);
  setBacklightPercent(settingBrightnessPercent);
  lcd.fillScreen(TFT_BLACK);

  setupTouch();
  setupEspNow();
  registerActivity();
  changeScreen(SCREEN_MAIN);
}

void loop() {
  handleTopButton();
  handleTouch();

  if (screen == SCREEN_CAL_DONE &&
      doneUntilMs != 0 &&
      millis() >= doneUntilMs) {
    doneUntilMs = 0;
    changeScreen(SCREEN_MAIN);
  }

  renderScreen();
  delay(10);
}
