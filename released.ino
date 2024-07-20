#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "ESP32_BME280_SPI.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h>
#include "BluetoothSerial.h"

// BNO055の設定
double eulerdata[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BME280の設定
const uint8_t SCLK_bme280 = 14;
const uint8_t MOSI_bme280 = 13;
const uint8_t MISO_bme280 = 12;
const uint8_t CS_bme280 = 15;
ESP32_BME280_SPI bme280spi(SCLK_bme280, MOSI_bme280, MISO_bme280, CS_bme280, 10000000);

// SDカードの設定
File file;
String progress = "Ready";
String file_name;

// GPSの設定
TinyGPSPlus gps;

// Bluetoothの設定
BluetoothSerial SerialBT;

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

  SerialBT.begin("cansatESP32"); //デバイス名

  // Bluetoothシリアルが利用可能になるまで待機
  while(!SerialBT.hasClient()) {
    delay(1000);
  }
  SerialBT.println("Bluetooth connected!");

  if(!SD.begin()) {
    SerialBT.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE) {
    SerialBT.println("No SD card attached");
    return;
  }

  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX = 16, TX = 17

  // GPSデータが有効でない場合、待機する
  while(!gps.location.isValid() && !gps.date.isValid() && !gps.time.isValid()) {
    SerialBT.println("Waiting for GPS signal...");
    delay(1000);
  }
  SerialBT.println("GPS signal acquired!"); 

  CreateFile("cansat");
}

void loop() {
  Released();
  exit(0);
}

void CreateFile(String FILE_NAME) {
  file_name = String("/") + FILE_NAME + ".csv";
  int counter = 2;

  while(SD.exists(file_name.c_str())) {
    file_name = String("/") + FILE_NAME + String(counter) + ".csv";
    counter++;
  }

  file = SD.open(file_name.c_str(), FILE_WRITE);
  if(!file) {
    SerialBT.println("Failed to create file");
  }
  SerialBT.println("Creating file: " + file_name);
}

void WriteLog(String data_name1 = "", String data1 = "", String data_name2 = "", String data2 = "") {
  String gps_time;

  while(Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
  }

  if(gps.date.isValid() && gps.time.isValid()) {
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    // JSTに変換
    hour += 9;
    if(hour >= 24) {
      hour -= 24;
      day += 1;

      if((month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12) && day > 31) {
        day = 1;
        month += 1;
      } 
      else if((month == 4 || month == 6 || month == 9 || month == 11) && day > 30) {
        day = 1;
        month += 1;
      } 
      else if(month == 2) {
        if((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
          if(day > 29) {
            day = 1;
            month += 1;
          }
        } 
        else {
          if(day > 28) {
            day = 1;
            month += 1;
          }
        }
      }

      if(month > 12) {
        month = 1;
        year += 1;
      }
    }

    char buffer[30];
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
    gps_time = String(buffer);
  }

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

  SerialBT.print(gps_time);
  SerialBT.print(',');
  SerialBT.print(progress);
  SerialBT.print(',');
  SerialBT.print(data_name1);
  SerialBT.print(',');
  SerialBT.print(data1);
  SerialBT.print(',');
  SerialBT.print(data_name2);
  SerialBT.print(',');
  SerialBT.println(data2);
}

// クオータニオンをオイラー角に変換
void Euler(){
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

//放出判定
void Released() {
  double ave_roll = 0;

  while(1) {
    for(int i = 0; i < 10; i++) {
      Euler();
      ave_roll += eulerdata[0];
      delay(10);
    }
    ave_roll /= 10;

    WriteLog("roll angle", String(ave_roll));

    int j = 0;
    if(fabs(ave_roll) < 45) {
      j++;
    }
    else {
      j = 0;
    }

    if(j >= 5) {
      progress = "Released";
      WriteLog();
      break;
    }
  }
}