# Display-Load-A-B-C-D

Firmware for the **handheld touchscreen display** of the wireless dual
load-cell scale. A Guition JC2432W328C (240x320 ST7789 TFT + CST820 capacitive
touch) shows the live weight of two load-cell nodes (A and B) plus their total,
and sends tare / calibrate / settings commands to them over **ESP-NOW**.

Pairs with the load-cell node project
[`ESP32-Load-A-B-C-D`](https://github.com/zhongzhi527/ESP32-Load-A-B-C-D).

## Hardware

| Part | Detail |
|------|--------|
| Board | Guition JC2432W328C |
| Screen | 2.8" ST7789 240x320 TFT (driven by LovyanGFX) |
| Touch | CST820 capacitive, I2C |
| Wireless | ESP-NOW (Wi-Fi STA) |
| Buttons | Top / BOOT button (GPIO0) |

### Pin connections

| Signal | GPIO |
|--------|------|
| TFT backlight | 27 |
| TFT SCLK / MOSI / DC / CS | 14 / 13 / 2 / 15 |
| Touch SDA / SCL | 33 / 32 |
| Touch RST / INT | 25 / 21 |
| Top button (BOOT) | 0 |

## How it works

- Listens for `WeightPacket`s from the two nodes and shows each weight, status
  and the combined total on the main screen.
- A per-node menu offers **TARE**, **CONTINUE** (clear a lock), and
  **CALIBRATE** (a guided known-weight wizard), plus **HOME**.
- A **Settings** screen (long-press the top button) sets the lock threshold,
  auto-lock, brightness and decimal places; these are saved and pushed to both
  nodes.
- Node connection is shown by the A/B indicators in the header (green =
  connected, red = offline, based on a 2.5 s packet timeout).

The packet layouts are shared with the nodes in
[`include/EspNowProtocol.h`](include/EspNowProtocol.h).

## Pairing

The display sends to each node's MAC address, set in `src/main.cpp`:

```cpp
uint8_t nodeMac[2][6] = {
  {0xE0, 0xE2, 0xE6, 0xD7, 0x3E, 0xD4}, // A
  {0xE0, 0xE2, 0xE6, 0xCC, 0xFE, 0xE0}  // B
};
```

Each node sends to the display's MAC (set in the node project). On boot, each
device prints its own MAC to the serial monitor (115200 baud); configure each
side with the other's MAC.

## Build & flash (PlatformIO)

1. Open this folder in VS Code with the **PlatformIO IDE** extension.
2. **Build**, then **Upload** to the Guition display over USB.
3. Open the **Serial Monitor** at `115200` baud to read the display's MAC.
