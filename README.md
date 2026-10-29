# Display-Load-A-B-C-D — QUAD SCALE handheld display

Handheld touchscreen display for the **QUAD SCALE** system. A Guition
JC2432W328C (240×320 ST7789 TFT + CST820 capacitive touch) shows the live
weight of the load-cell nodes plus their total, and sends tare / calibrate
commands over **ESP-NOW**.

This is the **display** half of the QUAD system. It pairs with the load-cell
node firmware
**[ESP32-Load-A-B-C-D](https://github.com/zhongzhi527/ESP32-Load-A-B-C-D)**.

> There are two load-cell systems. This repo is the handheld for the **4-cell
> ESP-NOW** one. The separate **2-cell Bluetooth** system lives in
> `Display-Load-A-B` (display) + `ESP32-Load-A-B` (scale).

## Cells A, B, C, D

The UI is laid out for four cells (A/B/C/D), but only **A and B are real**
hardware nodes. **C and D are decorative placeholders** — they always read
OFFLINE and their tare/calibrate actions are disabled. They exist to show the
4-cell layout; the live weighing is A + B.

## Hardware

| Part | Detail |
|------|--------|
| Board | Guition JC2432W328C |
| Screen | 2.8" ST7789 240×320 TFT (driven by LovyanGFX) |
| Touch | CST820 capacitive, I²C |
| Wireless | ESP-NOW (Wi-Fi STA) |
| Button | Top / BOOT button (GPIO0) |

### Pin connections

| Signal | GPIO |
|--------|------|
| TFT backlight | 27 |
| TFT SCLK / MOSI / DC / CS | 14 / 13 / 2 / 15 |
| Touch SDA / SCL | 33 / 32 |
| Touch RST / INT | 25 / 21 |
| Top button (BOOT) | 0 |

## What it does

- **Dashboard** — live weight of Load Cell A and B (and decorative C/D) plus
  the A+B total, with a battery gauge in the header.
- **Connection status** — each cell shows a clear lifecycle:
  **OFFLINE** (red) → **SYNCING** (yellow) → **CONNECTED** (green), with a
  matching pulsing panel border. Weights are shown to 2 decimal places.
- **Global actions** — **TARE ALL** and **CALIBRATE** from the dashboard.
- **CALIBRATE page** — pick a cell, set the known weight, then TARE / CALIBRATE
  (a guided wizard). Buttons are grey and light up blue when pressed; tapping an
  action on a disconnected cell shows "Load Cell X is not connected".
- **LINK page** — per-cell link status.
- **SETTINGS** — display brightness and ESP32 CPU clock.
- **Idle standby (snooze)** — after 5 min with no touch the screen sleeps; the
  last few seconds show a SNOOZING countdown. Any touch wakes it.

The packet layouts are shared with the nodes in
[`include/EspNowProtocol.h`](include/EspNowProtocol.h).

## Pairing (MAC addresses)

The display sends to each node's MAC address, set in `src/main.cpp`:

```cpp
uint8_t nodeMac[2][6] = {
  {0x38, 0x18, 0x2B, 0xA7, 0x8A, 0x10}, // A
  {0x80, 0x7D, 0x3A, 0xB9, 0x27, 0x2C}  // B
};
```

Each node, in turn, sends to the display's MAC (configured in the node
project as `displayMacAddress`, `70:4B:CA:76:C4:5C`). On boot every device
prints its own MAC to the serial monitor (115200 baud) — if you swap hardware,
read the new MACs there and update both sides to match.

## Build & flash (PlatformIO)

One environment, `jc2432w328`:

```sh
pio run -e jc2432w328 -t upload
```

Or in the VS Code PlatformIO sidebar: `jc2432w328` → **Upload**. Open the
**Serial Monitor** at `115200` baud to read the display's MAC.

### Platform version is pinned on purpose

`platformio.ini` pins `platform = espressif32@6.5.0`. Leaving it unpinned pulls
the newest platform, whose toolchain can fail to build. If a build fails right
after a clean checkout, confirm the pin is present and delete the `.pio` folder
before rebuilding.
