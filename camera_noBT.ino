#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "ESP32_BME280_SPI.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h>

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
const double goalGPSdata2[2] = {35.7631874, 139.8962477};// 早川の家
double azidata[2] = {0,0};

// SDカードの設定
File file;
String progress = "Ready"; // シーケンス
String FILE_NAME = "0717camera";
String file_name;

// モーターのピン
const int STBY = 2;     // モータードライバの制御の準備
const int AIN1 = 26;     // 1つ目のDCモーターの制御
const int AIN2 = 27;     // 1つ目のDCモーターの制御
const int BIN1 = 25;     // 2つ目のDCモーターの制御
const int BIN2 = 33;     // 2つ目のDCモーターの制御
const int PWMA = 4;     // 1つ目のDCモーターの回転速度
const int PWMB = 32;    // 2つ目のDCモーターの回転速度
//int RX_PIN = 16;
//int TX_PIN = 17;

// 溶断GPIOピン
const int FUSE_GPIO = 2;

//カメラの設定
const int pix=320; //画素数

void setup() {
  // BNO055の初期化
  pinMode(21, INPUT_PULLUP); // SDAピン21のプルアップ設定
  pinMode(22, INPUT_PULLUP); // SDAピン22のプルアップ設定

  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // BME280の初期化
  uint8_t t_sb = 5; // スタンバイ時間 1000ms
  uint8_t filter = 0; // フィルター無効
  uint8_t osrs_t = 4; // 温度オーバーサンプリング x4
  uint8_t osrs_p = 4; // 圧力オーバーサンプリング x4
  uint8_t osrs_h = 4; // 湿度オーバーサンプリング x4
  uint8_t Mode = 3; // 通常モード

  bme280spi.ESP32_BME280_SPI_Init(t_sb, filter, osrs_t, osrs_p, osrs_h, Mode);

  // モーターの初期化
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);

  // カメラの初期化
  Serial.begin(115200, SERIAL_8N1, 3, 1);

  //溶断回路の初期化
  pinMode(FUSE_GPIO, OUTPUT);
  digitalWrite(FUSE_GPIO, LOW);

  // SDカードの初期化
  if(!SD.begin()) {
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE) {
    return;
  }

  // GPSの初期化
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX = 16, TX = 1

  CreateFile("0718camera");
  delay(1000);
}

void loop() {
  Camera();
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
}

// 前進
void Forward(int i, int j) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
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
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

// 停止
void Stop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void Camera() {
  const double Kp =0.01;
  int x = -1, y = 0;
  float percentage;
  progress = "Camera guidance";

  if(!Serial.available()) {
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
      }
    
    // 短い遅延
    delay(10);

    if(x == -1) {
      //検出されないため、左右どちらかに旋回し続ける
      Turn(1, 50, 50);
    }
    else {
      Stop();
      int p_pwma= constrain(Kp * (x - pix) + 200, 0, 255);
      int p_pwmb = constrain(-Kp * (x - pix) + 200, 0, 255);
      Forward(p_pwma, p_pwmb);
    }

    if(percentage > 80.0){
      //ゴール判定
      Stop();
      progress = "Goal!";
      break;
    }
  }
}