# MAC printer (temporary diagnostic)

A throwaway tool to read each board's **Wi-Fi STA MAC address** — the value the
ESP-NOW pairing between the display and the load-cell nodes needs.

## Why

The display only accepts weight packets from the node MACs hardcoded in its
firmware, and the nodes only send to the display MAC hardcoded in theirs. Those
values are from the original author's boards, so on different hardware nothing
shows up. This tool reads the **real** MAC of each of your boards so the
addresses can be filled in correctly.

## How to use

Do this for **each** of the three boards, one at a time:

1. Open **this** folder (`diagnostics/mac_printer`) in VS Code as its own
   PlatformIO project (File -> Open Folder -> pick this folder).
2. Plug in the board and **Upload**.
3. Open the **Serial Monitor** at **115200**. The MAC prints every 2 seconds:
   ```
   ==================================================
                 THIS DEVICE'S MAC ADDRESS
   ==================================================
           >>>   XX:XX:XX:XX:XX:XX   <<<
   ==================================================
   ```
4. Write down which board it was (Load Cell A / Load Cell B / Display) and its
   MAC.
5. **Flash the real firmware back** onto that board (the load-cell project for
   the scales, this display project for the display).

## Then

Give the three MACs (Load Cell A, Load Cell B, Display) to update:
- the display's `nodeMac[0]` (A) and `nodeMac[1]` (B)
- each load-cell node's `displayMacAddress`

Once those match the real hardware, the display will show the live weights.
