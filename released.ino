#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// BNO055の設定
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ログの設定
File file;
String gps_time;
String progress = "準備中";
String file_name = "CarryRover";

void setup() {
  // BNO055の初期化
  pinMode(21, INPUT_PULLUP); // SDAピン21のプルアップ設定
  pinMode(22, INPUT_PULLUP); // SDAピン22のプルアップ設定

  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  Serial.begin(115200);

  // ログの初期化
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  file_name = String("/") + file_name + ".csv";
  int counter = 1;

  while (SD.exists(file_name.c_str())) {
    file_name = String("/") + file_name + String(counter) + ".csv";
    counter++;
  }

  Serial.printf("Creating file: %s\n", file_name.c_str());

  file = SD.open(file_name.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create file");
  }

  delay(1000);
}

void loop() {
  Released();
  exit(0);
}

// ログの書き込み
void WriteLog(String data_name1 = "", String data1 = "", String data_name2 = "", String data2 = "") {
  file = SD.open(file_name.c_str(), FILE_APPEND);
  file.print(gps_time);
  file.print(',');
  file.print(progress);
  file.print(',');
  file.print(data_name1);
  file.print(',');
  file.print(data1);
  file.print(',');
  file.print(data_name2);
  file.print(',');
  file.println(data2);
  file.close();
}

// クオータニオンをオイラー角に変換
double Euler(int axis) {
  imu::Quaternion quat = bno.getQuat();
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();

  double ysqr = y * y;

  // ロール角
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  double roll = atan2(t0, t1);

  // ピッチ角
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = asin(t2);

  // ヨー角
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);
  double yaw = atan2(t3, t4);

  //ラジアンから度に変換
  roll *= 57.2957795131;
  pitch *= 57.2957795131;
  yaw *= 57.2957795131;

  // 軸指定
  if(axis == 0) {
    return roll;
  }
  else if(axis == 1) {
    return pitch;
  }
  else if(axis == 2) {
    return yaw;
  }
}

//放出判定
void Released() {
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度

  while(1) {
    for(i = 0; i < 10; i++) {
      ave_roll += fabs(Euler(0));
      delay(10);
    }
    ave_roll /= 10;

    if(ave_roll < 45) {
      j++;
    }
    else {
      j = 0;
    }

    if(j >= 5) {
      progress = "放出";
      break;
    }

    WriteLog("ロール角", String(ave_roll));
  }
}