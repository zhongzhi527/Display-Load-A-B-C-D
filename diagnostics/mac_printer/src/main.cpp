// MAC printer - a tiny temporary tool to read a device's Wi-Fi STA MAC
// address, which is what the ESP-NOW display <-> load-cell pairing needs.
//
// Flash this to ANY of the three boards (Load Cell A, Load Cell B, or the
// Display), open the Serial Monitor at 115200, and the MAC is printed big and
// repeatedly so it cannot be missed. Note it down, then flash the real
// firmware back onto that board.

#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(500);

  // ESP-NOW uses the station-interface MAC, so report that one.
  WiFi.mode(WIFI_STA);
  delay(100);
}

void loop() {
  String mac = WiFi.macAddress();

  Serial.println();
  Serial.println("==================================================");
  Serial.println("              THIS DEVICE'S MAC ADDRESS");
  Serial.println("==================================================");
  Serial.print  ("        >>>   ");
  Serial.print  (mac);
  Serial.println("   <<<");
  Serial.println("==================================================");
  Serial.println("Copy the address above. Then flash the real");
  Serial.println("firmware back onto this board.");
  Serial.println();

  delay(2000);  // reprint every 2 seconds
}
