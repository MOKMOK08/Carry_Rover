#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "ESP32_BME280_SPI.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h>
#include "BluetoothSerial.h"

// BNO055の設定
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BME280の設定
const uint8_t SCLK_bme280 = 14;
const uint8_t MOSI_bme280 = 13;
const uint8_t MISO_bme280 = 12;
const uint8_t CS_bme280 = 15;
ESP32_BME280_SPI bme280spi(SCLK_bme280, MOSI_bme280, MISO_bme280, CS_bme280, 10000000);

// GPSの設定
TinyGPSPlus gps;
const double Kp_gps = 0.3;
const double goal_location[2] = {35.9181513330, 139.9077351670};// ゴール座標

// SDカードの設定
File file;
String progress = "Ready"; // シーケンス
String file_name;

// Bluetoothの設定
BluetoothSerial SerialBT;
String DEVICE_NAME = "ESP32_keitaro"; // デバイス名

// モーターのピン
const int STBY = 0;     // モータードライバの制御の準備
const int AIN1 = 26;     // 1つ目のDCモーターの制御
const int AIN2 = 27;     // 1つ目のDCモーターの制御
const int BIN1 = 25;     // 2つ目のDCモーターの制御
const int BIN2 = 33;     // 2つ目のDCモーターの制御
const int PWMA = 4;     // 1つ目のDCモーターの回転速度
const int PWMB = 32;    // 2つ目のDCモーターの回転速度

// 溶断GPIOピン
const int FUSE_GPIO = 2;

//カメラの設定
const int pix = 320; //画素数

void setup() {
  // Bluetoothの初期化
  SerialBT.begin(DEVICE_NAME);
  while(!SerialBT.hasClient()) {
    delay(1000);
  }

  SerialBT.println("Bluetooth connected!");

  Serial.print("start");
  // BNO055の初期化
  pinMode(21, INPUT_PULLUP); // SDAピン21のプルアップ設定
  pinMode(22, INPUT_PULLUP); // SDAピン22のプルアップ設定

  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // モーターの初期化
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);

  Serial.print("setup");

  // BME280の初期化
  uint8_t t_sb = 5; // スタンバイ時間 1000ms
  uint8_t filter = 0; // フィルター無効
  uint8_t osrs_t = 4; // 温度オーバーサンプリング x4
  uint8_t osrs_p = 4; // 圧力オーバーサンプリング x4
  uint8_t osrs_h = 4; // 湿度オーバーサンプリング x4
  uint8_t Mode = 3; // 通常モード

  bme280spi.ESP32_BME280_SPI_Init(t_sb, filter, osrs_t, osrs_p, osrs_h, Mode);

  //溶断回路の初期化
  pinMode(FUSE_GPIO, OUTPUT);
  digitalWrite(FUSE_GPIO, LOW);

  // SDカードの初期化
  if(!SD.begin()) {
    SerialBT.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE) {
    SerialBT.println("No SD card attached");
    return;
  }

  // GPSの初期化
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX = 16, TX = 17

  CreateFile("gps"); // ファイル名
  delay(1000);
}

void loop() {
  GPS();
  exit(0); //loopを1回で終了
}

// ファイルの作成
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

// ログの書き込み
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

// 前進
void Forward(int i, int j) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  delay(1000);
}

// 回転（0:右回転. 1:左回転）
void Turn(int a, int i, int j) {
  if(a == 0) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  }

  if(a == 1) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  }
}

// 後退
void Back(int i, int j) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

// 停止
void Stop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// ２点間の距離計算
double Distance(double lat1, double long1, double lat2, double long2){
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);

  return delta * 6372795;
}

// 方位角
double Azimuth(double lat1, double lng1, double lat2, double lng2) {
  double x1 = lng1 * M_PI / 180.0;
  double y1 = lat1 * M_PI / 180.0;
  double x2 = lng2 * M_PI / 180.0;
  double y2 = lat2 * M_PI / 180.0;
  double x_dif = x2 - x1;
  double azimuth = atan2(sin(x_dif), (cos(y1) * tan(y2) - sin(y1) * cos(x_dif))) * 180 / M_PI;

  return azimuth;
}

// GPS誘導
void GPS() {
  double current_location[2];
  progress = "GPS sequence";
  WriteLog();

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
    
    if (gps.location.isUpdated()){
      current_location[0] = gps.location.lat();
      current_location[1] = gps.location.lng();
    }
  }
      
  double goal_azimuth = Azimuth(current_location[0], current_location[1], goal_location[0], goal_location[1]);

  // 旋回
  while(1) {
    imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    double mag_x = magnetmetor.x();
    double mag_y = magnetmetor.y();
    double current_azimuth = atan2(mag_y, mag_x) * 180 / M_PI;
    if(current_azimuth < 0) {
      current_azimuth += 360;
    }

    double diff_azimuth = goal_azimuth - current_azimuth;
    if(diff_azimuth > 180) {
      diff_azimuth -= 360;
    }

    if(fabs(diff_azimuth) > 10) {
      Turn(0, 100, 100); // 右回転
    }
    else {
      Stop();
      break;
    }

    WriteLog("azimuth", String(diff_azimuth));
  }

  // 直進
  while(1) {
    while(Serial2.available() > 0) {
      char c = Serial2.read();
      gps.encode(c);

      if (gps.location.isUpdated()) {
        current_location[0] = gps.location.lat();
        current_location[1] = gps.location.lng();
      }
    }

    double distance = Distance(current_location[0], current_location[1], goal_location[0], goal_location[1]);
    double goal_azimuth = Azimuth(current_location[0], current_location[1], goal_location[0], goal_location[1]);

    if(distance < 5){
      Stop();
      progress = "GPS guidance completed";
      WriteLog();
      break;
    }
    else {
      imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      double mag_x = magnetmetor.x();
      double mag_y = magnetmetor.y();
      double current_azimuth = atan2(mag_y, mag_x) * 180 / M_PI;
      if(current_azimuth < 0) {
        current_azimuth += 360;
      }

      double diff_azimuth = goal_azimuth - current_azimuth;
      if(diff_azimuth > 180) {
        diff_azimuth -= 360;
      }

      int pwma = constrain(Kp_gps * diff_azimuth + 207, 0, 255);
      int pwmb = constrain(- Kp_gps * diff_azimuth + 207, 0, 255);
      Forward(pwma, pwmb);
      WriteLog("distance", String(distance), "azimuth", String(diff_azimuth));
    }
  }
}