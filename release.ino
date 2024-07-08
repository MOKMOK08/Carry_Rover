#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "ESP32_BME280_SPI.h"

// BNO055の設定
double euler_data[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BME280の設定
double pressure;
const uint8_t SCLK_bme280 = 14;
const uint8_t MOSI_bme280 = 13;
const uint8_t MISO_bme280 = 12;
const uint8_t CS_bme280 = 15;
ESP32_BME280_SPI bme280spi(SCLK_bme280, MOSI_bme280, MISO_bme280, CS_bme280, 10000000);

void setup() {
  // BNO055の初期化
  pinMode(21, INPUT_PULLUP); // SDAピン21のプルアップ設定
  pinMode(22, INPUT_PULLUP); // SDAピン22のプルアップ設定

  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  Serial.begin(115200);

  // BME280の初期化
  uint8_t t_sb = 5; // スタンバイ時間 1000ms
  uint8_t filter = 0; // フィルター無効
  uint8_t osrs_t = 4; // 温度オーバーサンプリング x4
  uint8_t osrs_p = 4; // 圧力オーバーサンプリング x4
  uint8_t osrs_h = 4; // 湿度オーバーサンプリング x4
  uint8_t Mode = 3; // 通常モード

  bme280spi.ESP32_BME280_SPI_Init(t_sb, filter, osrs_t, osrs_p, osrs_h, Mode);

  delay(1000);
}

void loop() {
  detectRelease();
  exit(0);
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

  euler_data[0] = roll;
  euler_data[1] = pitch;
  euler_data[2] = yaw;
}

void detectRelease() { 
  double ave_roll;
  double ave_pressure;
  double pre_pressure;
  double diff_pressure;
  int i, j = 0;

  Serial.println("release_start");

  for(i = 0; i < 10; i++) {
    pressure = (uint16_t)round(bme280spi.Read_Pressure());
    pre_pressure += pressure;
    delay(10);
  }

  pre_pressure /= 10;

  while(1) {
    for(i = 0; i < 10; i++) {
      getEuler();
      ave_roll += fabs(euler_data[0]);
      pressure = (uint16_t)round(bme280spi.Read_Pressure());
      ave_pressure += pressure;
      delay(10);
    }

    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - pre_pressure;
    pre_pressure = ave_pressure;

    if(ave_roll > 45 && ave_roll > 135 && diff_pressure > 0.3) {
      j++;
    }
    else {
      j = 0;
    }

    if(j == 10) {
      break;
    }
  }

  Serial.println("top");

  //放出判定
  while(1) {
    for(i = 0; i < 10; i++) {
      getEuler();
      ave_roll += fabs(euler_data[0]);
      pressure = (uint16_t)round(bme280spi.Read_Pressure());
      ave_pressure += pressure;
      delay(10);
    }

    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - pre_pressure;
    pre_pressure = ave_pressure;

    if(ave_roll < 90 && diff_pressure > 0.3) {
      j++;
    }
    else {
      j = 0;
    }

    if(j == 10) {
      break;
    }
  }

  Serial.println("release");
}

