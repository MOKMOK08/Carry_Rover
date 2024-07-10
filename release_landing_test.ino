#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "ESP32_BME280_SPI.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// SDカードの設定
File file;
const char* FILE_NAME = "release_landing_test";

// BNO055の設定
double eulerdata[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BME280の設定
const uint8_t SCLK_bme280 = 14;
const uint8_t MOSI_bme280 = 13;
const uint8_t MISO_bme280 = 12;
const uint8_t CS_bme280 = 15;
ESP32_BME280_SPI bme280spi(SCLK_bme280, MOSI_bme280, MISO_bme280, CS_bme280, 10000000);

// 溶断のGPIOピン
const int FUSE_GPIO = 2;

void setup() {
  // BNO055の初期化
  pinMode(21, INPUT_PULLUP); // SDAピン21のプルアップ設定
  pinMode(22, INPUT_PULLUP); // SDAピン22のプルアップ設定

  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // BME280の初期化
  uint8_t t_sb = 5; // スタンバイ時間 1000ms
  uint8_t filter = 0; // フィルター無効
  uint8_t osrs_t = 4; // 温度オーバーサンプリング x4
  uint8_t osrs_p = 4; // 圧力オーバーサンプリング x4
  uint8_t osrs_h = 4; // 湿度オーバーサンプリング x4
  uint8_t Mode = 3; // 通常モード

  bme280spi.ESP32_BME280_SPI_Init(t_sb, filter, osrs_t, osrs_p, osrs_h, Mode);

  //  SDカードの初期化
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

 String filename = String("/") + FILE_NAME + ".txt";
  int counter = 1;

  while (SD.exists(filename.c_str())) {
    filename = String("/") + FILE_NAME + String(counter) + ".txt";
    counter++;
  }

  Serial.printf("Creating file: %s\n", filename.c_str());

  file = SD.open(filename.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create file");
  }

  // 溶断の初期化
  pinMode(FUSE_GPIO, OUTPUT);
  digitalWrite(FUSE_GPIO, LOW);

  delay(1000);
}

void loop() {
  Start();
  Release();
  Landing();
  Fusing();
  exit(0);
}

// SDカード書き込み
void WriteFile(const char *message) {
  if (!file) {
    Serial.println("No file opened for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message written to file");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// クオータニオンをオイラー角に変換
void Euler() {
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

  eulerdata[0] = roll;
  eulerdata[1] = pitch;
  eulerdata[2] = yaw;
}

// スタート判定
void Start() {
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度
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
      Euler();
      ave_roll += fabs(eulerdata[0]);
      ave_pressure += bme280spi.Read_Pressure();
      delay(10);
    }

    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - init_pressure;

    if(ave_roll > 45 && ave_roll < 135) {
      j++;
    }
    else {
      j = 0;
    }

    if(j == 10 && diff_pressure > 1.0) {
      break;
    }
  }
}

//放出判定
void Release() {
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度
  double ave_pressure = 0.0; // 平均気圧
  double init_pressure = 0.0; // 初期気圧
  double diff_pressure = 0.0; // 差圧

  for(i = 0; i < 10; i++) {
    init_pressure += (uint16_t)round(bme280spi.Read_Pressure());
    delay(10);
  }

  init_pressure /= 10;

  while(1) {
    for(i = 0; i < 10; i++) {
      Euler();
      ave_roll += fabs(eulerdata[0]);
      ave_pressure += bme280spi.Read_Pressure();
      delay(10);
    }

    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - init_pressure;

    if(ave_roll < 45) {
      j++;
    }
    else {
      j = 0;
    }

    if(j == 10 && diff_pressure > 1.0) {
      break;
    }
  }
}

// 着地判定
void Landing() { 
  unsigned long start_time = millis();
  unsigned long current_time = millis();
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度
  double ave_pressure = 0.0; // 平均気圧
  double pre_pressure = 0.0; // 前の気圧
  double diff_pressure = 0.0; // 差圧

  for(i = 0; i < 10; i++) {
    pre_pressure += bme280spi.Read_Pressure();
    delay(10);
  }

  pre_pressure /= 10;
  
  while(1) {
    for(i = 0; i < 10; i++) {
      Euler();
      ave_roll += fabs(eulerdata[0]);
      ave_pressure += bme280spi.Read_Pressure();
      delay(10);
    }

    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - pre_pressure;
    pre_pressure = ave_pressure;

    if(ave_roll < 45 && diff_pressure < 0.1) {
      j++;
    }
    else{
      j = 0;
    }

    if(j == 10 || current_time - start_time > 30000) {
      break;
    }
    else{
      current_time = millis();
    }
  }
}

// 溶断
void Fusing() {
  digitalWrite(FUSE_GPIO, HIGH);
  delay(500);
  digitalWrite(FUSE_GPIO, LOW);
}