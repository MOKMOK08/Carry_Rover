#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "ESP32_BME280_SPI.h"

// BNO055の設定
double eulerdata[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BME280の設定
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
  Landing();
  exit(0);
}

// 着地判定
void Landing() { 
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  unsigned long start_time = millis();
  unsigned long current_time = millis();
  int i = 0, j = 0;
  double ave_gyro_x = 0.0; // 平均ロール角度
  double ave_pressure = 0.0; // 平均気圧
  double init_pressure = 0.0; // 初期気圧
  double diff_pressure = 0.0; // 差圧

  for(i = 0; i < 10; i++) {
    init_pressure += bme280spi.Read_Pressure();
    delay(10);
  }

  init_pressure /= 10;
  
  while(1) {
    for(i = 0; i < 10; i++) {
      ave_gyro_x += fabs(gyroscope.x());
      ave_pressure += bme280spi.Read_Pressure();
      delay(100);
    }

    ave_gyro_x /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - init_pressure;
    init_pressure = ave_pressure;

    if(ave_gyro_x < 0.1 && diff_pressure < 0.1) {
      j++;
    }
    else{
      j = 0;
    }

    if(j == 5 || current_time - start_time > 30000) {
      break;
    }
    else{
      current_time = millis();
    }
  }
}