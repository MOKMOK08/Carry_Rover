#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "ESP32_BME280_SPI.h"

// BNO055の設定
double eulerdata[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // BNO055の初期化
  pinMode(21, INPUT_PULLUP); // SDAピン21のプルアップ設定
  pinMode(22, INPUT_PULLUP); // SDAピン22のプルアップ設定

  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  Serial.begin(115200);

  delay(1000);
}

void loop() {
  getEuler();
  Serial.print("φ = ");
  Serial.println(eulerdata[0]);
  Serial.print("θ = ");
  Serial.println(eulerdata[1]);
  Serial.print("𝜓 = ");
  Serial.println(eulerdata[2]);
}

void getEuler() {
  imu::Quaternion quat = bno.getQuat();
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();

  double ysqr = y * y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  double roll = atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);
  double yaw = atan2(t3, t4);

  //ラジアンから度に変換
  roll *= 57.2957795131;
  pitch *= 57.2957795131;
  yaw *= 57.2957795131;

  eulerdata[0] = roll;
  eulerdata[1] = pitch;
  eulerdata[2] = yaw;
}
