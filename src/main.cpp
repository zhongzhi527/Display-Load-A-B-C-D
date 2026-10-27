// Single load-cell weighing display.
//
// A small SSD1306 OLED plus four push-buttons act as the controller for one
// remote load-cell node. The display receives live weight telemetry over
// ESP-NOW and sends tare / calibrate commands back to the node.
//
// Hardware:
//   - OLED SSD1306 128x64 over I2C (SDA=21, SCL=22)
//   - Four momentary buttons to GND: Up, Down, OK, Back
//   - ESP32 (esp32dev), ESP-NOW over Wi-Fi STA

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <Wire.h>
#include <U8g2lib.h>

#include <EspNowProtocol.h>

using LoadCellEspNow::CommandPacket;
using LoadCellEspNow::CommandType;
using LoadCellEspNow::ScaleStatus;
using LoadCellEspNow::WeightPacket;

namespace {

// --- Pin assignments -------------------------------------------------------
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr int BUTTON_UP_PIN = 25;
constexpr int BUTTON_DOWN_PIN = 26;
constexpr int BUTTON_OK_PIN = 27;
constexpr int BUTTON_BACK_PIN = 32;

// --- Timing ----------------------------------------------------------------
constexpr unsigned long CONNECTION_TIMEOUT_MS = 2000;
constexpr unsigned long DEBOUNCE_MS = 180;
constexpr unsigned long DONE_SCREEN_MS = 1800;

// --- Calibration limits ----------------------------------------------------
constexpr float CALIBRATION_MIN_KG = 0.1f;
constexpr float CALIBRATION_MAX_KG = 20.0f;
constexpr float CALIBRATION_STEP_KG = 0.1f;

// MAC address of the remote load-cell node this display pairs with.
uint8_t loadCellMacAddress[] = {0xE0, 0xE2, 0xE6, 0xD7, 0x3E, 0xD4};

// UI screens, in navigation order.
enum class Screen : uint8_t {
  Main,
  Menu,
  CalWeight,
  CalRemove,
  CalPlace,
  Done,
};

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Telemetry written by the ESP-NOW receive callback (ISR context), read by the
// main loop. Marked volatile because they cross that boundary.
volatile float receivedWeightKg = 0.0f;
volatile uint8_t receivedStatus = static_cast<uint8_t>(ScaleStatus::Measuring);
volatile bool newPacket = false;

unsigned long lastPacketMs = 0;
unsigned long lastButtonMs = 0;
unsigned long doneUntil = 0;
float calibrationWeightKg = 1.0f;
Screen screen = Screen::Main;
int menuIndex = 0;
bool waitingForTare = false;
bool waitingForCalibration = false;

const char *menuItems[] = {"Tare", "Calibrate", "Back"};

// Debounced active-low button read. Returns true once per distinct press.
bool pressed(int pin) {
  if (digitalRead(pin) != LOW) {
    return false;
  }
  const unsigned long now = millis();
  if (now - lastButtonMs < DEBOUNCE_MS) {
    return false;
  }
  lastButtonMs = now;
  return true;
}

void drawCentered(const char *text, int y) {
  u8g2.drawStr((128 - u8g2.getStrWidth(text)) / 2, y, text);
}

bool connected() {
  return millis() - lastPacketMs <= CONNECTION_TIMEOUT_MS;
}

const char *statusText() {
  switch (static_cast<ScaleStatus>(receivedStatus)) {
  case ScaleStatus::Stable:
    return "Stable";
  case ScaleStatus::Taring:
    return "Taring...";
  case ScaleStatus::Calibrating:
    return "Calibrating...";
  case ScaleStatus::CalibrationComplete:
    return "Calibration OK";
  case ScaleStatus::Error:
    return "Error";
  case ScaleStatus::Measuring:
  default:
    return "Measuring...";
  }
}

void sendCommand(CommandType command, float knownWeightKg = 0.0f) {
  CommandPacket packet{knownWeightKg, static_cast<uint8_t>(command)};
  esp_now_send(loadCellMacAddress,
               reinterpret_cast<uint8_t *>(&packet),
               sizeof(packet));
}

// ESP-NOW receive callback. The signature changed in ESP-IDF 5, so select the
// right one at compile time.
#if ESP_IDF_VERSION_MAJOR >= 5
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *data, int len) {
#else
void onDataReceived(const uint8_t *, const uint8_t *data, int len) {
#endif
  if (len != sizeof(WeightPacket)) {
    return;
  }
  WeightPacket packet;
  memcpy(&packet, data, sizeof(packet));
  receivedWeightKg = packet.weightKg;
  receivedStatus = packet.status;
  newPacket = true;
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    // Fatal: without ESP-NOW there is nothing to display. Halt visibly.
    while (true) {
      delay(1000);
    }
  }
  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, loadCellMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  if (!esp_now_is_peer_exist(loadCellMacAddress)) {
    esp_now_add_peer(&peerInfo);
  }
}

// Apply any freshly received telemetry and advance the calibration wizard when
// the node confirms a step completed.
void handlePacket() {
  if (!newPacket) {
    return;
  }
  newPacket = false;
  lastPacketMs = millis();

  if (waitingForTare &&
      static_cast<ScaleStatus>(receivedStatus) == ScaleStatus::Measuring) {
    waitingForTare = false;
    screen = Screen::CalPlace;
  }
  if (waitingForCalibration &&
      static_cast<ScaleStatus>(receivedStatus) ==
          ScaleStatus::CalibrationComplete) {
    waitingForCalibration = false;
    screen = Screen::Done;
    doneUntil = millis() + DONE_SCREEN_MS;
  }
}

void handleButtons() {
  const bool up = pressed(BUTTON_UP_PIN);
  const bool down = pressed(BUTTON_DOWN_PIN);
  const bool ok = pressed(BUTTON_OK_PIN);
  const bool back = pressed(BUTTON_BACK_PIN);

  switch (screen) {
  case Screen::Main:
    if (ok) {
      screen = Screen::Menu;
    }
    if (back) {
      sendCommand(CommandType::Tare);
    }
    break;

  case Screen::Menu:
    if (up) {
      menuIndex = (menuIndex + 2) % 3;
    }
    if (down) {
      menuIndex = (menuIndex + 1) % 3;
    }
    if (back) {
      screen = Screen::Main;
    }
    if (ok) {
      if (menuIndex == 0) {
        sendCommand(CommandType::Tare);
        screen = Screen::Main;
      } else if (menuIndex == 1) {
        screen = Screen::CalWeight;
      } else {
        screen = Screen::Main;
      }
    }
    break;

  case Screen::CalWeight:
    if (up) {
      calibrationWeightKg =
          min(CALIBRATION_MAX_KG, calibrationWeightKg + CALIBRATION_STEP_KG);
    }
    if (down) {
      calibrationWeightKg =
          max(CALIBRATION_MIN_KG, calibrationWeightKg - CALIBRATION_STEP_KG);
    }
    if (ok) {
      screen = Screen::CalRemove;
    }
    if (back) {
      screen = Screen::Menu;
    }
    break;

  case Screen::CalRemove:
    if (ok && !waitingForTare) {
      waitingForTare = true;
      sendCommand(CommandType::Tare);
    }
    if (back && !waitingForTare) {
      screen = Screen::CalWeight;
    }
    break;

  case Screen::CalPlace:
    if (ok && !waitingForCalibration) {
      waitingForCalibration = true;
      sendCommand(CommandType::Calibrate, calibrationWeightKg);
    }
    if (back && !waitingForCalibration) {
      screen = Screen::CalWeight;
    }
    break;

  case Screen::Done:
    if (ok || back) {
      screen = Screen::Main;
    }
    break;
  }
}

void drawMain() {
  char weight[20];
  snprintf(weight, sizeof(weight), "%.3f kg", receivedWeightKg);

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, connected() ? "Connected" : "Connection lost");

  u8g2.setFont(u8g2_font_ncenB20_tr);
  drawCentered(weight, 39);

  u8g2.setFont(u8g2_font_6x12_tr);
  drawCentered(connected() ? statusText() : "Searching...", 55);
  u8g2.drawStr(0, 64, "OK:Menu Back:Tare");
}

void drawMenu() {
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(0, 13, "Menu");
  u8g2.setFont(u8g2_font_6x12_tr);
  for (int i = 0; i < 3; i++) {
    char line[20];
    snprintf(line, sizeof(line), "%c %s", i == menuIndex ? '>' : ' ',
             menuItems[i]);
    u8g2.drawStr(8, 29 + i * 13, line);
  }
}

void drawCalWeight() {
  char weight[20];
  snprintf(weight, sizeof(weight), "%.1f kg", calibrationWeightKg);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  drawCentered("Known Weight", 16);
  u8g2.setFont(u8g2_font_ncenB20_tr);
  drawCentered(weight, 43);
  u8g2.setFont(u8g2_font_6x12_tr);
  drawCentered("Up/Down, OK next", 61);
}

void drawCalRemove() {
  u8g2.setFont(u8g2_font_ncenB10_tr);
  drawCentered(waitingForTare ? "Taring..." : "Remove Weight", 18);
  u8g2.setFont(u8g2_font_6x12_tr);
  drawCentered(waitingForTare ? "Please wait" : "Press OK to tare", 44);
}

void drawCalPlace() {
  char line[22];
  snprintf(line, sizeof(line), "Place %.1f kg", calibrationWeightKg);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  drawCentered(waitingForCalibration ? "Calibrating..." : "Calibration", 18);
  u8g2.setFont(u8g2_font_6x12_tr);
  drawCentered(waitingForCalibration ? "Keep weight still" : line, 40);
  drawCentered(waitingForCalibration ? "Please wait" : "Press OK", 57);
}

void drawDone() {
  u8g2.setFont(u8g2_font_ncenB10_tr);
  drawCentered("Calibration", 20);
  drawCentered("Complete", 43);
}

void drawScreen() {
  u8g2.clearBuffer();
  switch (screen) {
  case Screen::Main:
    drawMain();
    break;
  case Screen::Menu:
    drawMenu();
    break;
  case Screen::CalWeight:
    drawCalWeight();
    break;
  case Screen::CalRemove:
    drawCalRemove();
    break;
  case Screen::CalPlace:
    drawCalPlace();
    break;
  case Screen::Done:
    drawDone();
    break;
  }
  u8g2.sendBuffer();
}

} // namespace

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_OK_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BACK_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  u8g2.begin();
  setupEspNow();
}

void loop() {
  handlePacket();
  handleButtons();
  if (screen == Screen::Done && millis() >= doneUntil) {
    screen = Screen::Main;
  }
  drawScreen();
  delay(20);
}
