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

// Shared colour palette (RGB565), matching the Display-Load-A-B (DUAL SCALE)
// firmware: a dark navy screen with slate panels and palette-tinted text.
constexpr uint16_t COLOR_BACKGROUND  = 0x0083;  // dark navy screen bg
constexpr uint16_t COLOR_PANEL       = 0x08E5;  // slate panel fill
constexpr uint16_t COLOR_PANEL_LIGHT = 0x1188;  // lighter slate (button fill)
constexpr uint16_t COLOR_PRIMARY     = 0x047E;  // blue
constexpr uint16_t COLOR_SUCCESS     = 0x3EEE;  // green
constexpr uint16_t COLOR_WARNING     = 0xF5A6;  // amber
constexpr uint16_t COLOR_DANGER      = 0xF228;  // red
constexpr uint16_t COLOR_MUTED       = 0x8D16;  // grey

// Pulsing-border animation timing (half periods, in ms) and service cadence.
constexpr uint32_t BLINK_SEARCH_MS = 700;
constexpr uint32_t BLINK_ATTENTION_MS = 900;
constexpr uint32_t BLINK_ERROR_MS = 800;
constexpr unsigned long BORDER_SERVICE_MS = 50;

// Battery sensing, ported from the Display-Load-A-B (DUAL SCALE) firmware. The
// single-cell LiPo is fed through an on-board divider to GPIO39 (input-only
// ADC); BATTERY_DIVIDER_RATIO converts the pin voltage back to the pack
// voltage. ~4.2 V full, ~3.3 V flat; ~3.45 V (~15%) raises the low warning.
constexpr int BATTERY_SENSE_PIN = 39;
constexpr float BATTERY_DIVIDER_RATIO = 1.5f;
constexpr float BATTERY_FULL_VOLTS = 4.20f;
constexpr float BATTERY_EMPTY_VOLTS = 3.30f;
constexpr float BATTERY_LOW_VOLTS = 3.45f;
constexpr uint32_t BATTERY_SAMPLE_INTERVAL_MS = 2000;
constexpr uint8_t BATTERY_SAMPLE_AVERAGE_COUNT = 16;
// Time smoothing so the gauge does not jitter: each refresh eases this
// fraction of the way toward the newest raw reading (smaller = steadier).
constexpr float BATTERY_SMOOTHING_FACTOR = 0.15f;

// MAC addresses of the two load-cell nodes (A and B).
uint8_t nodeMac[2][6] = {
  {0x38, 0x18, 0x2B, 0xA7, 0x8A, 0x10}, // A
  {0x80, 0x7D, 0x3A, 0xB9, 0x27, 0x2C}  // B
};

enum Screen : uint8_t {
  SCREEN_MAIN,
  SCREEN_LINK,
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

// Battery state, refreshed roughly every BATTERY_SAMPLE_INTERVAL_MS. Mirrors
// the DUAL display: a smoothed pack voltage drives a 0-100% estimate.
float batteryVolts = 0.0f;            // smoothed pack voltage (displayed basis)
float batterySmoothedPercent = -1.0f; // smoothed %, -1 until first sample
uint8_t batteryPercent = 0;           // integer % actually shown
bool batteryReadingValid = false;
bool batteryLow = false;
uint32_t lastBatterySampleMs = 0;
int16_t batteryDrawnPercent = -1;     // -1 forces a first draw
bool batteryDrawnLow = false;

// Last border colour drawn for each real cell panel (A=0, B=1). Sentinel
// 0xFFFF forces a redraw on the next service pass. Used by
// serviceDashboardBorders() to avoid redrawing (and flickering) the border
// when the animated colour has not changed.
uint16_t cellBorderDrawn[2] = {0xFFFF, 0xFFFF};

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
  if (!nodeConnected(i)) return COLOR_DANGER;             // OFFLINE
  if (nodes[i].locked || nodes[i].status == STATUS_STABLE)
    return COLOR_SUCCESS;                                 // STABLE / LOCKED
  switch (nodes[i].status) {
    case STATUS_CALIBRATION_COMPLETE: return COLOR_SUCCESS;  // CAL COMPLETE
    case STATUS_ERROR: return COLOR_DANGER;                  // ERROR
    // In-progress states: SEARCHING / STABILIZING / TARING / CALIBRATING.
    case STATUS_TARING:
    case STATUS_CALIBRATING:
    default: return COLOR_WARNING;
  }
}

// Weighted blend of two RGB565 colours: returns fg mixed over bg with
// fgWeight/255 of fg (0 = all bg, 255 = all fg). Channels are unpacked to
// 5/6/5, averaged with +127 rounding, and repacked. Copied from the DUAL
// display so the pulsing "dim" phase glows instead of turning off.
uint16_t blendRgb565(uint16_t fg, uint16_t bg, uint8_t fgWeight) {
  uint16_t bgWeight = 255 - fgWeight;
  uint16_t fr = (fg >> 11) & 0x1F, fgn = (fg >> 5) & 0x3F, fb = fg & 0x1F;
  uint16_t br = (bg >> 11) & 0x1F, bgn = (bg >> 5) & 0x3F, bb = bg & 0x1F;
  uint16_t r = (fr * fgWeight + br * bgWeight + 127) / 255;
  uint16_t g = (fgn * fgWeight + bgn * bgWeight + 127) / 255;
  uint16_t b = (fb * fgWeight + bb * bgWeight + 127) / 255;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

// Animated border colour: alternates between the full "bright" colour and a
// dim glow (72/255 of bright blended over the panel fill) every halfPeriodMs.
// The dim phase never fully turns off, so the border pulses/glows.
uint16_t animatedBorderColor(uint16_t bright, uint32_t halfPeriodMs) {
  bool brightPhase = ((millis() / halfPeriodMs) & 1U) == 0;
  return brightPhase ? bright : blendRgb565(bright, COLOR_PANEL, 72);
}

// Border colour for a real cell panel (A=0, B=1), driven by its live state:
//   connected + READY/STABLE/LOCKED -> solid green (no blink)
//   connected but working           -> pulsing amber (attention)
//   error                           -> pulsing red
//   offline / searching             -> pulsing amber (trying to connect)
uint16_t cellBorderColor(uint8_t i) {
  if (nodeConnected(i)) {
    if (nodes[i].status == STATUS_ERROR)
      return animatedBorderColor(COLOR_DANGER, BLINK_ERROR_MS);
    if (nodes[i].locked ||
        nodes[i].status == STATUS_STABLE ||
        nodes[i].status == STATUS_CALIBRATION_COMPLETE)
      return COLOR_SUCCESS;  // connected + ready -> solid green
    // In-progress (TARING / CALIBRATING / STABILIZING / MEASURING).
    return animatedBorderColor(COLOR_WARNING, BLINK_ATTENTION_MS);
  }
  // Not connected: pulse amber while trying to connect / searching.
  return animatedBorderColor(COLOR_WARNING, BLINK_SEARCH_MS);
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
  cellBorderDrawn[0] = cellBorderDrawn[1] = 0xFFFF;
}

void changeScreen(Screen next) {
  screen = next;
  staticScreenRequired = true;
  resetPaintCache();
}

// Draw a palette button. `color` is the fill colour (one of the COLOR_*
// palette entries); `borderColor` tints the outline (0 = auto: a lighter tint
// of the fill). The label is white. Disabled buttons fall back to a muted
// slate fill/outline. Positions, sizes and labels are unchanged from before.
void drawButton(int x, int y, int w, int h, const char* label,
                uint32_t color, bool enabled = true,
                uint16_t borderColor = 0) {
  uint32_t c = enabled ? color : COLOR_PANEL;
  uint16_t border;
  if (!enabled) {
    border = COLOR_MUTED;
  } else if (borderColor != 0) {
    border = borderColor;
  } else {
    border = blendRgb565((uint16_t)color, 0xFFFF, 150);  // lighter tint of fill
  }
  lcd.fillRoundRect(x, y, w, h, 7, c);
  lcd.drawRoundRect(x, y, w, h, 7, border);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(enabled ? TFT_WHITE : COLOR_MUTED, c);
  lcd.setTextSize(1);
  lcd.drawString(label, x + w / 2, y + h / 2);
}

void drawHeader(const char* title) {
  lcd.fillRect(0, 0, 240, 28, COLOR_PANEL);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_WHITE, COLOR_PANEL);
  lcd.setTextSize(2);
  lcd.drawString(title, 120, 14);
  // No A/B connection letters on sub-page headers (CALIBRATE/SETTINGS/LINK/
  // wizard) - they belong only on the dashboard, shown via the LINK button.
}

// Sample the battery pin and update the cached percentage. Ported from the
// DUAL display: the raw reading is averaged across ADC samples and smoothed
// across time (EMA) so the gauge holds steady and climbs gently when charging.
void updateBatteryReading(bool force = false) {
  const uint32_t now = millis();
  if (!force && lastBatterySampleMs != 0 &&
      now - lastBatterySampleMs < BATTERY_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastBatterySampleMs = now;

  uint32_t rawSum = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLE_AVERAGE_COUNT; ++i) {
    rawSum += analogRead(BATTERY_SENSE_PIN);
  }
  const float rawAverage =
      static_cast<float>(rawSum) / BATTERY_SAMPLE_AVERAGE_COUNT;

  // ESP32 ADC: ~0..4095 over ~0..3.3 V at 11 dB attenuation.
  const float pinVolts = (rawAverage / 4095.0f) * 3.3f;
  const float packVolts = pinVolts * BATTERY_DIVIDER_RATIO;

  // A plausible pack sits above ~2.5 V at the pin; far below means the sense
  // line is not reading a battery (e.g. bench power), so flag invalid.
  const bool valid = packVolts > 2.5f;
  batteryReadingValid = valid;
  if (!valid) {
    return;
  }

  // Smooth the pack voltage over time so the reading does not jitter.
  if (batteryVolts <= 0.0f) {
    batteryVolts = packVolts;  // first valid sample seeds the filter
  } else {
    batteryVolts += (packVolts - batteryVolts) * BATTERY_SMOOTHING_FACTOR;
  }

  // Map the smoothed voltage to a percentage from the LiPo curve.
  float targetPercent =
      (batteryVolts - BATTERY_EMPTY_VOLTS) /
      (BATTERY_FULL_VOLTS - BATTERY_EMPTY_VOLTS) * 100.0f;
  targetPercent = constrain(targetPercent, 0.0f, 100.0f);
  batterySmoothedPercent = targetPercent;

  batteryPercent = static_cast<uint8_t>(batterySmoothedPercent + 0.5f);
  batteryLow = batteryVolts <= BATTERY_LOW_VOLTS;
}

// Compact battery glyph + "NN%" at the far right of the dashboard header
// (0..28 tall). The body sits at x=176 so it clears the SET button, with the
// percentage text to its right. Green >=50%, amber below, red when low; an
// outlined "--" is shown while the reading is not yet valid. `background` is
// the header fill so the glyph clears cleanly on an incremental redraw.
void drawBatteryIndicator(uint16_t background) {
  // Keep the change-tracker in sync so the loop's incremental redraw does not
  // immediately repaint after a full-page draw.
  batteryDrawnPercent =
      batteryReadingValid ? static_cast<int16_t>(batteryPercent) : -1;
  batteryDrawnLow = batteryLow;

  constexpr int ICON_X = 176;  // battery body left edge
  constexpr int ICON_Y = 7;
  constexpr int ICON_W = 22;
  constexpr int ICON_H = 15;
  constexpr int TEXT_X = ICON_X + ICON_W + 3;  // 201 -> "%" text anchor

  // Clear the indicator region (icon + nub + percentage text).
  lcd.fillRect(ICON_X - 2, 2, 240 - (ICON_X - 2), 24, background);

  lcd.setTextDatum(middle_left);
  lcd.setTextSize(1);

  if (!batteryReadingValid) {
    // Unknown: draw an empty outline with a dash.
    lcd.drawRect(ICON_X, ICON_Y, ICON_W, ICON_H, COLOR_MUTED);
    lcd.fillRect(ICON_X + ICON_W, ICON_Y + 4, 3, 7, COLOR_MUTED);
    lcd.setTextColor(COLOR_MUTED, background);
    lcd.drawString("--", TEXT_X, 14);
    return;
  }

  const uint16_t color =
      batteryLow ? COLOR_DANGER
                 : (batteryPercent >= 50 ? COLOR_SUCCESS : COLOR_WARNING);

  // Battery outline + positive terminal nub.
  lcd.drawRect(ICON_X, ICON_Y, ICON_W, ICON_H, color);
  lcd.fillRect(ICON_X + ICON_W, ICON_Y + 4, 3, 7, color);

  // Fill proportional to charge.
  const int innerW = ICON_W - 4;
  const int fillW = (innerW * batteryPercent) / 100;
  if (fillW > 0) {
    lcd.fillRect(ICON_X + 2, ICON_Y + 2, fillW, ICON_H - 4, color);
  }

  // Percentage text to the right of the icon.
  char label[8];
  snprintf(label, sizeof(label), "%u%%", batteryPercent);
  lcd.setTextColor(color, background);
  lcd.drawString(label, TEXT_X, 14);
}

// Dashboard header in the Display-Load-A-B style: the "QUAD SCALE" title on the
// left, then LINK and SET buttons, then a battery indicator at the far right.
// LINK opens the link-status page; SET opens settings. LINK and SET are shifted
// left (from the old x=96/x=166) to free the top-right corner for the battery.
void drawDashboardHeader() {
  lcd.fillRect(0, 0, 240, 28, COLOR_PANEL);
  lcd.setTextDatum(middle_left);
  lcd.setTextColor(TFT_WHITE, COLOR_PANEL);
  lcd.setTextSize(1);
  lcd.drawString("QUAD SCALE", 8, 14);

  // LINK button (border coloured by overall connection) and SET button, both
  // shifted left to make room for the battery indicator at the far right.
  const uint32_t linkColor =
      (nodeConnected(0) && nodeConnected(1)) ? COLOR_SUCCESS
      : (nodeConnected(0) || nodeConnected(1)) ? COLOR_WARNING
                                               : COLOR_DANGER;
  drawButton(70, 2, 52, 24, "LINK", COLOR_PRIMARY);
  lcd.drawRoundRect(70, 2, 52, 24, 7, linkColor);
  drawButton(126, 2, 44, 24, "SET", COLOR_PANEL_LIGHT, true, COLOR_MUTED);

  // Battery indicator at the top-right of the header.
  drawBatteryIndicator(COLOR_PANEL);

  paintedHeaderConnected[0] = nodeConnected(0);
  paintedHeaderConnected[1] = nodeConnected(1);
  paintedHeaderValid = true;
}

// Recolour the LINK button border when the overall connection state changes,
// so the dashboard header shows link status live without a full redraw.
void updateDashboardLink() {
  bool a = nodeConnected(0);
  bool b = nodeConnected(1);
  if (paintedHeaderValid &&
      paintedHeaderConnected[0] == a &&
      paintedHeaderConnected[1] == b) {
    return;
  }
  const uint32_t linkColor =
      (a && b) ? COLOR_SUCCESS : (a || b) ? COLOR_WARNING : COLOR_DANGER;
  lcd.drawRoundRect(70, 2, 52, 24, 7, linkColor);
  paintedHeaderConnected[0] = a;
  paintedHeaderConnected[1] = b;
  paintedHeaderValid = true;
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
  // Filled slate panel with a static muted border (decorative: always OFFLINE).
  lcd.fillRoundRect(6, y, 228, CELL_ROW_H, 6, COLOR_PANEL);
  lcd.drawRoundRect(6, y, 228, CELL_ROW_H, 6, COLOR_MUTED);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_MUTED, COLOR_PANEL);
  lcd.drawString(label, 14, y + 6);
  lcd.drawString("OFFLINE", 14, y + 24);

  lcd.setTextDatum(middle_right);
  lcd.setTextSize(2);
  lcd.setTextColor(COLOR_MUTED, COLOR_PANEL);
  lcd.drawString("--", 206, y + CELL_ROW_H / 2);
  lcd.setTextSize(1);
  lcd.drawString("kg", 228, y + CELL_ROW_H / 2 + 1);
}

void drawMainStatic() {
  lcd.fillScreen(COLOR_BACKGROUND);
  drawDashboardHeader();

  // Real cells A and B (updated live by updateMainNode). Each panel is filled
  // slate, with an animated status-coloured border drawn on top. Seed the
  // border cache so serviceDashboardBorders() stays in sync with this draw.
  for (uint8_t i = 0; i < 2; i++) {
    uint16_t border = cellBorderColor(i);
    lcd.fillRoundRect(6, CELL_ROW_Y[i], 228, CELL_ROW_H, 6, COLOR_PANEL);
    lcd.drawRoundRect(6, CELL_ROW_Y[i], 228, CELL_ROW_H, 6, border);
    cellBorderDrawn[i] = border;
  }
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, COLOR_PANEL);
  lcd.drawString("LOAD CELL A", 14, CELL_ROW_Y[0] + 6);
  lcd.drawString("LOAD CELL B", 14, CELL_ROW_Y[1] + 6);

  // Decorative cells C and D (drawn once, never updated).
  drawDecorativeCell("LOAD CELL C", CELL_ROW_Y[2]);
  drawDecorativeCell("LOAD CELL D", CELL_ROW_Y[3]);

  // Total row (A + B only): filled slate panel with a success/muted border.
  bool totalValid = (nodeConnected(0) || nodeConnected(1)) &&
                    fabsf(totalWeightKg()) >= 0.001f;
  lcd.fillRoundRect(6, TOTAL_ROW_Y, 228, TOTAL_ROW_H, 6, COLOR_PANEL);
  lcd.drawRoundRect(6, TOTAL_ROW_Y, 228, TOTAL_ROW_H, 6,
                    totalValid ? COLOR_SUCCESS : COLOR_MUTED);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_PRIMARY, COLOR_PANEL);
  lcd.drawString("TOTAL", 14, TOTAL_ROW_Y + 6);

  // Global action buttons.
  drawButton(6, 270, 111, 40, "TARE ALL", COLOR_PRIMARY);
  drawButton(123, 270, 111, 40, "CALIBRATE", COLOR_PANEL_LIGHT, true,
             COLOR_WARNING);
  resetPaintCache();
  // resetPaintCache() clears the border cache; re-seed it to match what we
  // just drew so the animated border does not immediately repaint.
  cellBorderDrawn[0] = cellBorderColor(0);
  cellBorderDrawn[1] = cellBorderColor(1);
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

  // Clear the status line (under the label) and the weight area to the panel
  // fill so no black box appears around the text.
  lcd.fillRect(14, y + 23, 110, 14, COLOR_PANEL);
  lcd.fillRect(110, y + 2, 122, CELL_ROW_H - 4, COLOR_PANEL);

  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(statusColor(i), COLOR_PANEL);
  lcd.drawString(statusText(i), 14, y + 24);

  char text[24];
  formatWeight(text, sizeof(text), nodes[i].weightKg, connected);
  lcd.setTextDatum(middle_right);
  lcd.setTextColor(connected ? TFT_WHITE : COLOR_MUTED, COLOR_PANEL);
  lcd.setTextSize(2);
  lcd.drawString(text, 206, y + CELL_ROW_H / 2);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_MUTED, COLOR_PANEL);
  lcd.drawString("kg", 228, y + CELL_ROW_H / 2 + 1);

  paintedWeight[i] = nodes[i].weightKg;
  paintedStatus[i] = nodes[i].status;
  paintedConnected[i] = connected;
  paintedLocked[i] = nodes[i].locked;

  // The weight-area clear painted over the right edge of the panel border;
  // redraw the border and re-sync the cache so the animated border and this
  // static repaint stay consistent.
  uint16_t border = cellBorderColor(i);
  lcd.drawRoundRect(6, y, 228, CELL_ROW_H, 6, border);
  cellBorderDrawn[i] = border;
}

void updateMainTotal() {
  float total = totalWeightKg();
  if (!isnan(paintedTotal) && fabsf(total - paintedTotal) < 0.001f) return;

  lcd.fillRect(70, TOTAL_ROW_Y + 2, 162, TOTAL_ROW_H - 4, COLOR_PANEL);
  char text[24];
  formatWeight(text, sizeof(text), total, true);

  lcd.setTextDatum(middle_right);
  lcd.setTextSize(3);
  lcd.setTextColor(TFT_WHITE, COLOR_PANEL);
  lcd.drawString(text, 206, TOTAL_ROW_Y + TOTAL_ROW_H / 2);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_SUCCESS, COLOR_PANEL);
  lcd.drawString("kg", 228, TOTAL_ROW_Y + TOTAL_ROW_H / 2 + 1);
  paintedTotal = total;

  // Redraw the total panel border (the clear reached its right edge) with the
  // success/muted tint based on whether the total is valid/non-zero.
  bool totalValid = (nodeConnected(0) || nodeConnected(1)) &&
                    fabsf(total) >= 0.001f;
  lcd.drawRoundRect(6, TOTAL_ROW_Y, 228, TOTAL_ROW_H, 6,
                    totalValid ? COLOR_SUCCESS : COLOR_MUTED);
}

// Drive the pulsing/blinking borders of the A/B cell panels on the dashboard.
// Throttled to BORDER_SERVICE_MS; only redraws a panel's border when its
// animated colour actually changes (tracked in cellBorderDrawn[]), so the
// panels glow smoothly without flicker. No-op off the main dashboard.
void serviceDashboardBorders() {
  if (screen != SCREEN_MAIN) return;
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs < BORDER_SERVICE_MS) return;
  lastMs = now;

  for (uint8_t i = 0; i < 2; i++) {
    uint16_t color = cellBorderColor(i);
    if (color != cellBorderDrawn[i]) {
      lcd.drawRoundRect(6, CELL_ROW_Y[i], 228, CELL_ROW_H, 6, color);
      cellBorderDrawn[i] = color;
    }
  }
}

// Redraw the dashboard battery indicator only when the shown percent or the
// low-state changes, and only on the main screen (mirrors the DUAL display's
// batteryDrawnPercent tracking). The glyph is drawn over the header fill.
void serviceBatteryIndicator() {
  if (screen != SCREEN_MAIN) return;
  const int16_t shown =
      batteryReadingValid ? static_cast<int16_t>(batteryPercent) : -1;
  if (shown == batteryDrawnPercent && batteryLow == batteryDrawnLow) return;
  drawBatteryIndicator(COLOR_PANEL);
}

// The four known-weight adjust buttons, matching the DUAL display's calibrate
// page: -1 / -0.1 / +0.1 / +1 in a single row. Each entry's x/y/w/h is also
// used verbatim as the touch hit-box in handleTouch(), so the drawn button and
// its hit-box always agree. The DUAL row sits at y=194 under a 42px header;
// the QUAD's header is 28px, so the whole body is shifted up 14px (y=180).
struct CalAdjustButton {
  int x;
  int y;
  int w;
  int h;
  float deltaKg;
  const char* label;
};
constexpr CalAdjustButton CAL_ADJUST_BUTTONS[] = {
  {4, 180, 55, 32, -1.0f, "-1"},
  {63, 180, 55, 32, -0.1f, "-0.1"},
  {122, 180, 55, 32, 0.1f, "+0.1"},
  {181, 180, 55, 32, 1.0f, "+1"},
};
constexpr uint8_t CAL_ADJUST_BUTTON_COUNT =
  sizeof(CAL_ADJUST_BUTTONS) / sizeof(CAL_ADJUST_BUTTONS[0]);

// Calibration cell-picker page, rebuilt 1:1 from the DUAL display's calibrate
// page (drawCalibrationPage / drawCalibrationDynamic). Top-to-bottom: a BACK
// button in the header, two instruction lines, a KNOWN WEIGHT panel with the
// big value, a LOAD CELL A / B selector, the four -1/-0.1/+0.1/+1 adjust
// buttons, a full-width TARE button and a full-width CALIBRATE button. Because
// the QUAD header is 28px (vs the DUAL's 42px) the body is shifted up 14px
// while keeping the same visual order and proportions. Only the two real cells
// A and B are selectable.
void drawCalibrateStatic() {
  lcd.fillScreen(COLOR_BACKGROUND);
  drawHeader("CALIBRATE");

  // BACK button at the top-left of the header (DUAL has one at (5,7,55,28) in
  // its 42px header; the QUAD's 28px header is tighter, so use a small button).
  drawButton(4, 2, 48, 24, "BACK", COLOR_PANEL_LIGHT, true, COLOR_MUTED);

  // Two instruction lines (COLOR_MUTED, size 1) on the screen background. DUAL
  // draws these at y=50 and y=65; shifted up 14px to y=36 and y=51.
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_MUTED, COLOR_BACKGROUND);
  lcd.drawString("1. Select a scale and tare it empty", 8, 36);
  lcd.drawString("2. Load selected cell only, then CAL", 8, 51);

  // KNOWN WEIGHT panel: a filled COLOR_PANEL rounded panel with the caption at
  // its top-left and the big value centred below. DUAL panel is (6,84,228,72);
  // shifted up 14px to (6,70,228,72). Caption DUAL y=94 -> y=80, value DUAL
  // y=130 -> y=116.
  lcd.fillRoundRect(6, 70, 228, 72, 9, COLOR_PANEL);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_MUTED, COLOR_PANEL);
  char caption[32];
  snprintf(caption, sizeof(caption), "KNOWN WEIGHT - LOAD CELL %c",
           selectedNode == 0 ? 'A' : 'B');
  lcd.drawString(caption, 16, 80);

  char text[20];
  snprintf(text, sizeof(text), "%.1f kg", calibrationWeightKg);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_WHITE, COLOR_PANEL);
  lcd.setTextSize(3);
  lcd.drawString(text, 120, 116);

  // LOAD CELL A / B selector (two 111-wide buttons). The selected one is
  // highlighted (COLOR_PRIMARY fill); the other is COLOR_PANEL_LIGHT / muted.
  // DUAL row is y=158; shifted up 14px to y=144.
  drawButton(6, 144, 111, 30, "LOAD CELL A",
             selectedNode == 0 ? COLOR_PRIMARY : COLOR_PANEL_LIGHT, true,
             selectedNode == 0 ? 0 : COLOR_MUTED);
  drawButton(123, 144, 111, 30, "LOAD CELL B",
             selectedNode == 1 ? COLOR_PRIMARY : COLOR_PANEL_LIGHT, true,
             selectedNode == 1 ? 0 : COLOR_MUTED);

  // Four adjust buttons (-1 / -0.1 / +0.1 / +1). Drawn from the shared table so
  // their positions match the hit-boxes in handleTouch() exactly.
  for (uint8_t i = 0; i < CAL_ADJUST_BUTTON_COUNT; i++) {
    const CalAdjustButton& b = CAL_ADJUST_BUTTONS[i];
    drawButton(b.x, b.y, b.w, b.h, b.label, COLOR_PANEL_LIGHT, true,
               COLOR_MUTED);
  }

  // Full-width TARE and CALIBRATE buttons for the selected cell. DUAL rows are
  // y=232 (h=36) and y=276 (h=37); shifted up 14px to y=218 and y=262.
  char tareLabel[24];
  snprintf(tareLabel, sizeof(tareLabel), "TARE LOAD CELL %c",
           selectedNode == 0 ? 'A' : 'B');
  char calLabel[28];
  snprintf(calLabel, sizeof(calLabel), "CALIBRATE LOAD CELL %c",
           selectedNode == 0 ? 'A' : 'B');
  bool connected = nodeConnected(selectedNode);
  drawButton(6, 218, 228, 36, tareLabel, COLOR_PANEL_LIGHT, connected,
             COLOR_PRIMARY);
  drawButton(6, 262, 228, 37, calLabel, COLOR_PRIMARY, connected,
             COLOR_PRIMARY);

  resetPaintCache();
}

void drawSimpleMessage(const char* title, const char* line1,
                       const char* line2 = nullptr) {
  lcd.fillScreen(COLOR_BACKGROUND);
  drawHeader(title);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(TFT_WHITE, COLOR_BACKGROUND);
  lcd.setTextSize(2);
  lcd.drawString(line1, 120, 70);
  if (line2) {
    lcd.setTextSize(1);
    lcd.setTextColor(COLOR_WARNING, COLOR_BACKGROUND);
    lcd.drawString(line2, 120, 120);
  }
}

void drawCalWeight() {
  drawSimpleMessage(selectedNode == 0 ? "CALIBRATE A" : "CALIBRATE B",
                    "Choose known weight");

  char text[20];
  snprintf(text, sizeof(text), "%.1f kg", calibrationWeightKg);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(COLOR_WARNING, COLOR_BACKGROUND);
  lcd.setTextSize(4);
  lcd.drawString(text, 120, 165);

  drawButton(6, 230, 68, 54, "-", COLOR_PRIMARY);
  drawButton(86, 230, 68, 54, "+", COLOR_PRIMARY);
  drawButton(166, 230, 68, 54, "NEXT", COLOR_SUCCESS);
}

void drawCalRemove() {
  drawSimpleMessage("STEP 1 OF 2",
                    waitingForTare ? "Taring..." : "Remove all weight",
                    waitingForTare ? "Please wait" : "Then press TARE");
  if (!waitingForTare) drawButton(30, 210, 180, 54, "TARE", COLOR_PRIMARY);
}

void drawCalPlace() {
  char line[30];
  snprintf(line, sizeof(line), "Place %.1f kg", calibrationWeightKg);
  drawSimpleMessage("STEP 2 OF 2",
                    waitingForCalibration ? "Calibrating..." : line,
                    waitingForCalibration ? "Keep weight still" : "Then press START");
  if (!waitingForCalibration)
    drawButton(30, 210, 180, 54, "START", COLOR_SUCCESS);
}

void drawCalDone() {
  drawSimpleMessage("CALIBRATION", "COMPLETE", "Calibration saved");
  drawButton(50, 210, 140, 50, "DONE", COLOR_PRIMARY);
}

void drawExitConfirm() {
  drawSimpleMessage("EXIT CALIBRATION", "Return to main menu?",
                    "The current load-cell operation may finish.");
  drawButton(18, 210, 96, 54, "NO", COLOR_PANEL_LIGHT, true, COLOR_MUTED);
  drawButton(126, 210, 96, 54, "YES", COLOR_DANGER);
}

// One label/value pair inside a LINK STATUS panel (Display-Load-A-B style): a
// small muted caption with the coloured value centred just below it. `centerX`
// is the panel's horizontal centre; `labelY` the caption baseline-top.
void drawLinkItem(int centerX, int labelY, const char* label,
                  const char* value, uint16_t valueColor) {
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_MUTED, COLOR_PANEL);
  lcd.drawString(label, centerX, labelY);
  lcd.setTextColor(valueColor, COLOR_PANEL);
  lcd.drawString(value, centerX, labelY + 15);
}

// Draw one real-cell panel (A=0, B=1) in the DUAL connectivity style: a filled
// COLOR_PANEL panel titled "LOAD CELL x" with LINK / STATUS / DATA AGE rows.
void drawLinkCellPanel(uint8_t i, int px, int centerX) {
  constexpr int PANEL_Y = 34;
  constexpr int PANEL_W = 111;
  constexpr int PANEL_H = 214;
  lcd.fillRoundRect(px, PANEL_Y, PANEL_W, PANEL_H, 9, COLOR_PANEL);

  lcd.setTextDatum(middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, COLOR_PANEL);
  lcd.drawString(i == 0 ? "LOAD CELL A" : "LOAD CELL B", centerX, PANEL_Y + 16);
  lcd.drawFastHLine(px + 8, PANEL_Y + 28, PANEL_W - 16, COLOR_PANEL_LIGHT);

  bool connected = nodeConnected(i);

  // LINK row.
  drawLinkItem(centerX, PANEL_Y + 48, "LINK",
               connected ? "CONNECTED" : "OFFLINE",
               connected ? COLOR_SUCCESS : COLOR_DANGER);

  // STATUS row (the node's live status text).
  drawLinkItem(centerX, PANEL_Y + 96, "STATUS", statusText(i),
               statusColor(i));

  // DATA AGE row: seconds since the last packet, green if fresh.
  char age[16];
  uint16_t ageColor;
  if (nodes[i].lastPacketMs == 0) {
    snprintf(age, sizeof(age), "--");
    ageColor = COLOR_DANGER;
  } else {
    float seconds = (millis() - nodes[i].lastPacketMs) / 1000.0f;
    if (seconds > 999.0f) seconds = 999.0f;
    snprintf(age, sizeof(age), "%.1f s", seconds);
    ageColor = (seconds < 2.5f) ? COLOR_SUCCESS : COLOR_DANGER;
  }
  drawLinkItem(centerX, PANEL_Y + 144, "DATA AGE", age, ageColor);
}

// LINK status page in the Display-Load-A-B connectivity style. The two real
// nodes A and B each get a filled panel with LINK / STATUS / DATA AGE rows;
// the decorative C and D cells are shown as a muted "N/A (demo)" strip.
void drawLinkStatus() {
  lcd.fillScreen(COLOR_BACKGROUND);
  drawHeader("LINK STATUS");

  drawLinkCellPanel(0, 6, 61);
  drawLinkCellPanel(1, 123, 178);

  // Decorative C / D cells: a muted strip advertising the 4-cell capability.
  lcd.fillRoundRect(6, 252, 228, 14, 4, COLOR_PANEL);
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_MUTED, COLOR_PANEL);
  lcd.drawString("CELLS C / D: N/A (demo)", 120, 259);

  drawButton(65, 270, 110, 40, "HOME", COLOR_PANEL_LIGHT, true, COLOR_MUTED);
}

// One settings row in the Display-Load-A-B style: a filled COLOR_PANEL rounded
// panel with a small COLOR_MUTED caption at the top-left and the current value
// shown large to its left of the control buttons (buttons are drawn by the
// caller). Returns nothing; the panel occupies (6, y, 228, SETTINGS_ROW_H).
constexpr int SETTINGS_ROW_H = 54;
void drawSettingsPanel(int y, const char* caption, const char* value,
                       uint16_t valueColor) {
  lcd.fillRoundRect(6, y, 228, SETTINGS_ROW_H, 9, COLOR_PANEL);
  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(COLOR_MUTED, COLOR_PANEL);
  lcd.drawString(caption, 16, y + 7);
  lcd.setTextDatum(middle_left);
  lcd.setTextSize(2);
  lcd.setTextColor(valueColor, COLOR_PANEL);
  lcd.drawString(value, 16, y + 34);
}

// SETTINGS page restyled to match the DUAL display: each adjustable setting
// gets a filled COLOR_PANEL panel with a muted caption, a large value and
// palette-styled +/- or TOGGLE controls. The QUAD keeps its own settings
// (lock threshold, auto-lock, brightness, decimals) and HOME/SAVE - there is
// no CPU-clock control here. Button coordinates match the SCREEN_SETTINGS
// hit-boxes in handleTouch().
void drawSettings() {
  lcd.fillScreen(COLOR_BACKGROUND);
  drawHeader("SETTINGS");

  char value[24];

  // Lock threshold: panel + / - controls.
  snprintf(value, sizeof(value), "%.1f kg", settingLockThresholdKg);
  drawSettingsPanel(34, "LOCK ABOVE", value, TFT_WHITE);
  drawButton(130, 44, 46, 34, "-", COLOR_PRIMARY);
  drawButton(182, 44, 46, 34, "+", COLOR_PRIMARY);

  // Auto-lock: panel + TOGGLE.
  drawSettingsPanel(92, "AUTO-LOCK", settingAutoLock ? "ON" : "OFF",
                    settingAutoLock ? COLOR_SUCCESS : COLOR_MUTED);
  drawButton(130, 102, 98, 34, "TOGGLE",
             settingAutoLock ? COLOR_SUCCESS : COLOR_PANEL_LIGHT, true,
             settingAutoLock ? 0 : COLOR_MUTED);

  // Display brightness: panel + / - controls.
  snprintf(value, sizeof(value), "%u%%", settingBrightnessPercent);
  drawSettingsPanel(150, "DISPLAY BRIGHTNESS", value, TFT_WHITE);
  drawButton(130, 160, 46, 34, "-", COLOR_PRIMARY);
  drawButton(182, 160, 46, 34, "+", COLOR_PRIMARY);

  // Decimals: panel + TOGGLE.
  snprintf(value, sizeof(value), "%u", settingDecimals);
  drawSettingsPanel(208, "DECIMALS", value, TFT_WHITE);
  drawButton(130, 218, 98, 34, "TOGGLE", COLOR_PRIMARY);

  // Navigation.
  drawButton(6, 270, 111, 40, "HOME", COLOR_PANEL_LIGHT, true, COLOR_MUTED);
  drawButton(123, 270, 111, 40, "SAVE", COLOR_SUCCESS);
}

void renderScreen() {
  if (staticScreenRequired) {
    switch (screen) {
      case SCREEN_MAIN: drawMainStatic(); break;
      case SCREEN_LINK: drawLinkStatus(); break;
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
    updateDashboardLink();
    updateMainNode(0);
    updateMainNode(1);
    updateMainTotal();
  } else if (screen == SCREEN_LINK) {
    // Redraw only when a real cell's connection state changes, so the page
    // reflects live link status without flickering every loop.
    bool a = nodeConnected(0);
    bool b = nodeConnected(1);
    if (paintedHeaderConnected[0] != a || paintedHeaderConnected[1] != b) {
      paintedHeaderConnected[0] = a;
      paintedHeaderConnected[1] = b;
      staticScreenRequired = true;
    }
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
    // Header LINK / SET buttons (shifted left to make room for the battery
    // indicator; hit-boxes match drawDashboardHeader()).
    if (inside(x, y, 70, 2, 52, 24)) {
      changeScreen(SCREEN_LINK);
    } else if (inside(x, y, 126, 2, 44, 24)) {
      changeScreen(SCREEN_SETTINGS);
    }
    // TARE ALL: tare both real nodes (A and B). C/D are decorative.
    else if (inside(x, y, 6, 270, 111, 40)) {
      sendCommand(0, COMMAND_TARE);
      delay(30);
      sendCommand(1, COMMAND_TARE);
    } else if (inside(x, y, 123, 270, 111, 40)) {
      changeScreen(SCREEN_CALIBRATE);
    }
    return;
  }

  if (screen == SCREEN_LINK) {
    if (inside(x, y, 65, 270, 110, 40)) {
      changeScreen(SCREEN_MAIN);
    }
    return;
  }

  if (screen == SCREEN_CALIBRATE) {
    // BACK button in the header -> return to the dashboard.
    if (inside(x, y, 4, 2, 48, 24)) {
      changeScreen(SCREEN_MAIN);
      return;
    }

    // LOAD CELL A / B selector (hit-boxes match the drawCalibrateStatic()
    // selector buttons).
    if (inside(x, y, 6, 144, 111, 30)) {
      selectedNode = 0;
      staticScreenRequired = true;
      return;
    }
    if (inside(x, y, 123, 144, 111, 30)) {
      selectedNode = 1;
      staticScreenRequired = true;
      return;
    }

    // Four adjust buttons (-1 / -0.1 / +0.1 / +1). Each entry's rect is used as
    // both the drawn button and its hit-box, so they always agree. A tap
    // applies the button's delta, clamped to [0.1, 100.0] and rounded to one
    // decimal, then redraws the page.
    for (uint8_t i = 0; i < CAL_ADJUST_BUTTON_COUNT; i++) {
      const CalAdjustButton& b = CAL_ADJUST_BUTTONS[i];
      if (inside(x, y, b.x, b.y, b.w, b.h)) {
        float next = calibrationWeightKg + b.deltaKg;
        next = roundf(next * 10.0f) / 10.0f;
        calibrationWeightKg = constrain(next, 0.1f, 100.0f);
        staticScreenRequired = true;
        return;
      }
    }

    // Full-width TARE: tare only the selected cell.
    if (inside(x, y, 6, 218, 228, 36) && nodeConnected(selectedNode)) {
      sendCommand(selectedNode, COMMAND_TARE);
      return;
    }

    // Full-width CALIBRATE: enter the guided calibration wizard for the
    // selected cell (preserves the QUAD's existing SCREEN_CAL_WEIGHT flow and
    // the onDataReceived handshake).
    if (inside(x, y, 6, 262, 228, 37) && nodeConnected(selectedNode)) {
      changeScreen(SCREEN_CAL_WEIGHT);
      return;
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
    // Hit-boxes match the panel button coordinates in drawSettings().
    if (inside(x, y, 130, 44, 46, 34)) {
      settingLockThresholdKg = max(0.0f, settingLockThresholdKg - 0.5f);
    } else if (inside(x, y, 182, 44, 46, 34)) {
      settingLockThresholdKg = min(100.0f, settingLockThresholdKg + 0.5f);
    } else if (inside(x, y, 130, 102, 98, 34)) {
      settingAutoLock = !settingAutoLock;
    } else if (inside(x, y, 130, 160, 46, 34)) {
      settingBrightnessPercent =
        max(25, (int)settingBrightnessPercent - 25);
      setBacklightPercent(settingBrightnessPercent);
    } else if (inside(x, y, 182, 160, 46, 34)) {
      settingBrightnessPercent =
        min(100, (int)settingBrightnessPercent + 25);
      setBacklightPercent(settingBrightnessPercent);
    } else if (inside(x, y, 130, 218, 98, 34)) {
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
  lcd.fillScreen(COLOR_BACKGROUND);

  // Battery ADC setup (Arduino core; unaffected by LovyanGFX) and one priming
  // read so the indicator shows a value on the first dashboard draw.
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_SENSE_PIN, ADC_11db);
  updateBatteryReading(true);

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

  updateBatteryReading();
  renderScreen();
  serviceDashboardBorders();
  serviceBatteryIndicator();
  delay(10);
}
