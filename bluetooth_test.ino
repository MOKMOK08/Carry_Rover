#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  SerialBT.begin("ESP32test"); //Bluetooth device name
}

void loop() {
  SerialBT.println("Hello World!");
  delay(1000);
}