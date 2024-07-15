#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  SerialBT.begin("cansatESP32"); // デバイス名

  // Bluetoothシリアルが利用可能になるまで待機
  while(!SerialBT.hasClient()) {
    delay(1000);
  }
  SerialBT.println("Bluetooth connected!");
}

void loop() {
  SerialBT.println("Hello World!");
  delay(1000);
}