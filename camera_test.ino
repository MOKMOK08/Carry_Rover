#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

//カメラの設定
const int pix = 320; //画素数

void setup() {
  Serial.begin(115200, SERIAL_8N1, 3, 1);

  // Bluetoothの初期化
  SerialBT.begin("ESP32_keitaro"); //デバイス名
  while (!SerialBT.hasClient()) {
    delay(1000);
  }

  SerialBT.println("Bluetooth connected!");
}

void loop() {
  Camera();
  delay(1000);
}

void Camera() {
  const double Kp =0.03;
  int x = -1, y = 0;
  float percentage;

  if(!Serial.available()) {
    SerialBT.println("no signal");
    delay(1000);
  }

  // データが受信されている場合
  while(1){
    if (Serial.available()) {
      // 受信データを読み取る
      String receivedData = Serial.readStringUntil('\n');
      
      // 受信データをコンマで分割
      int firstComma = receivedData.indexOf(',');
      int secondComma = receivedData.indexOf(',', firstComma + 1);
      int spaceAfterSecondComma = receivedData.indexOf(' ', secondComma + 1);
      
      // X座標
      String xStr = receivedData.substring(1, firstComma); // "R"の後から最初のカンマまで
      x = xStr.toInt();
      
      // Y座標
      String yStr = receivedData.substring(firstComma + 1, secondComma);
      y = yStr.toInt();
      
      // 割合
      String percentageStr = receivedData.substring(secondComma + 1);
      percentageStr.trim();
      percentage = percentageStr.toFloat();

      SerialBT.println(xStr);
      SerialBT.println(percentageStr);
    }
  }
}

