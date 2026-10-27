# Display-Load-A-B-C-D

Firmware for a **single load-cell weighing display** built on an ESP32 with a
small OLED screen and four push-buttons. The display pairs with one remote
load-cell node and talks to it over **ESP-NOW**: it receives live weight
readings and sends tare / calibrate commands back.

> Note: this is a separate design from the `Display` (dual Bluetooth load-cell)
> project. This one uses an **OLED + physical buttons + ESP-NOW**, not a colour
> touchscreen over Bluetooth.

## Hardware

| Part | Detail |
|------|--------|
| MCU | ESP32 (`esp32dev`) |
| Screen | SSD1306 128x64 monochrome OLED, I2C |
| Wireless | ESP-NOW (Wi-Fi STA) |
| Scale | One remote load-cell node (paired by MAC address) |

### Pin connections

| Signal | GPIO |
|--------|------|
| OLED SDA | 21 |
| OLED SCL | 22 |
| Button: Up | 25 |
| Button: Down | 26 |
| Button: OK | 27 |
| Button: Back | 32 |

Buttons are wired active-low to GND (internal pull-ups enabled in firmware).

## How it works

- The remote load-cell node sends `WeightPacket` (weight, locked flag, status)
  to this display over ESP-NOW.
- The display sends `CommandPacket` (known weight, command) back to request a
  tare or calibration.
- Both packet layouts are defined once in [`include/EspNowProtocol.h`](include/EspNowProtocol.h)
  so the two devices stay in sync.

### Screens

```
Main  --OK-->  Menu  --Calibrate-->  Cal Weight  --OK-->  Cal Remove (tare)
                                                               |
                                                               v
                                        Done  <--complete--  Cal Place
```

- **Main** — shows the live weight, connection state, and status. `OK` opens the
  menu; `Back` sends a quick tare.
- **Menu** — Tare / Calibrate / Back, navigated with Up/Down and `OK`.
- **Calibration wizard** — set the known weight, remove weight to tare, then
  place the known weight to calibrate. Shows "Calibration Complete" when done.

## Pairing with a load-cell node

The target node's MAC address is set in `src/main.cpp`:

```cpp
uint8_t loadCellMacAddress[] = {0xE0, 0xE2, 0xE6, 0xD7, 0x3E, 0xD4};
```

Change these six bytes to match your load-cell node's Wi-Fi STA MAC address.

## Build & flash (PlatformIO)

1. Open this folder in VS Code with the **PlatformIO IDE** extension.
2. **Build**, then **Upload** to the ESP32 over USB.
3. Open the **Serial Monitor** at `115200` baud.

Dependencies and the pinned framework are declared in
[`platformio.ini`](platformio.ini); PlatformIO installs them automatically.

## Project layout

```
include/EspNowProtocol.h   Shared ESP-NOW packet definitions
src/main.cpp               Display firmware (UI, buttons, ESP-NOW)
platformio.ini             Board, framework and library configuration
```
