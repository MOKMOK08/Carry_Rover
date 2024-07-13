#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "ESP32_BME280_SPI.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// BNO055の設定
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BME280の設定
const uint8_t SCLK_bme280 = 14;
const uint8_t MOSI_bme280 = 13;
const uint8_t MISO_bme280 = 12;
const uint8_t CS_bme280 = 15;
ESP32_BME280_SPI bme280spi(SCLK_bme280, MOSI_bme280, MISO_bme280, CS_bme280, 10000000);

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

  // BME280の初期化
  uint8_t t_sb = 5; // スタンバイ時間 1000ms
  uint8_t filter = 0; // フィルター無効
  uint8_t osrs_t = 4; // 温度オーバーサンプリング x4
  uint8_t osrs_p = 4; // 圧力オーバーサンプリング x4
  uint8_t osrs_h = 4; // 湿度オーバーサンプリング x4
  uint8_t Mode = 3; // 通常モード

  bme280spi.ESP32_BME280_SPI_Init(t_sb, filter, osrs_t, osrs_p, osrs_h, Mode);

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
  Landing();
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

// 着地判定
void Landing() { 
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  unsigned long start_time = millis();
  unsigned long current_time = millis();
  unsigned long diff_time = 0.0;
  int i = 0, j = 0;
  double ave_gyro_x = 0.0; // 平均ロール角速度
  double ave_pressure = 0.0; // 平均気圧
  double init_pressure = 0.0; // 初期気圧
  double diff_pressure = 0.0; // 差圧

  for(i = 0; i < 10; i++) {
    init_pressure += bme280spi.Read_Pressure();
    delay(10);
  }

  init_pressure /= 10;
  
  delay(500);

  while(1) {
    for(i = 0; i < 10; i++) {
      ave_gyro_x += fabs(gyroscope.x());
      ave_pressure += bme280spi.Read_Pressure();
      delay(10);
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

    diff_time = current_time - start_time;

    if(j == 5 || diff_time > 30000) {
      progress = "着地";
      WriteLog("ロール角速度", String(ave_gyro_x));
      break;
    }
    else{
      current_time = millis();
    }

    delay(500);
  }
}