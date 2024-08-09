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
const double Kp_gps = 0.6; // P制御の比例ゲイン
const double goal_location[2] = {35.9247450, 139.9118707};// ゴール座標

// カメラの設定
const double Kp_camera = 0.07; // P制御の比例ゲイン
const int pix = 320; //画素数

// SDカードの設定
File file;
const String FILE_NAME = "etoe"; // ファイル名
String progress = "Ready"; // シーケンス
String file_name;

// Bluetoothの設定
BluetoothSerial SerialBT;
const String DEVICE_NAME = "ESP32_keitaro"; // デバイス名

// モーターのピン
const int STBY = 2;     // モータードライバの制御の準備
const int AIN1 = 26;     // 1つ目のDCモーターの制御
const int AIN2 = 27;     // 1つ目のDCモーターの制御
const int BIN1 = 25;     // 2つ目のDCモーターの制御
const int BIN2 = 33;     // 2つ目のDCモーターの制御
const int PWMA = 4;     // 1つ目のDCモーターの回転速度
const int PWMB = 32;    // 2つ目のDCモーターの回転速度

// 溶断GPIOピン
const int FUSE_GPIO = 2;

void setup() {
  // Bluetoothの初期化
  SerialBT.begin(DEVICE_NAME);
  while(!SerialBT.hasClient()) {
    delay(1000);
  }

  SerialBT.println("Bluetooth connected!");

  // カメラの初期化
  Serial.begin(115200, SERIAL_8N1, 3, 1);

  // GPSの初期化
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX = 16, TX = 17
  while(1) {
    while (Serial2.available() > 0){
      char c = Serial2.read();
      gps.encode(c);
    }

    if (gps.location.isValid() && gps.location.isUpdated()) {
      delay(10000);
      break;
    }
    else if(!gps.location.isValid() && !gps.location.isUpdated()) {
      SerialBT.println("Waiting for GPS signal...");
      delay(1000);
    }
  }

  // BNO055の初期化
  pinMode(21, INPUT_PULLUP); // SDAピン21のプルアップ設定
  pinMode(22, INPUT_PULLUP); // SDAピン22のプルアップ設定

  if (!bno.begin()) {
    SerialBT.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
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
  
  CreateFile(FILE_NAME);
  delay(1000);
}

void loop() {
  Start();
  Released();
  Landing();
  Fusing();
  GPS();
  Camera();
  while(1) {
  }
}

// ファイルの作成
void CreateFile(String FILE_NAME) {
  file_name = String("/") + FILE_NAME + ".txt";
  int counter = 2;

  while(SD.exists(file_name.c_str())) {
    file_name = String("/") + FILE_NAME + String(counter) + ".txt";
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
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  if(a == 1) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
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

// クオータニオンをオイラー角に変換
double Euler(int axis) {
  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  imu::Quaternion quat = bno.getQuat();
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();
  double ysqr = y * y;

  if(axis == 0) {
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    roll = atan2(t0, t1) * 57.2957795131;
    return roll;
  }
  else if(axis == 1){
    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = asin(t2) * 57.2957795131;
    return pitch;
  }
  else if(axis == 2){
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);  
    yaw = atan2(t3, t4) * 57.2957795131;
    return yaw;
  }
}

// スタート判定
void Start() {
  double init_pressure = bme280spi.Read_Pressure(); 

  while(1) {
    double current_roll = Euler(0);
    double current_pressure = bme280spi.Read_Pressure();
    double diff_pressure = current_pressure - init_pressure;

    WriteLog("roll angle", String(current_roll), "differential pressure", String(diff_pressure));

    if(fabs(current_roll) > 45 && fabs(current_roll) < 135 /*&& diff_pressure < -0.5*/) { // etoe用にコメントアウト
      progress = "Start";
      WriteLog();
      break;
    }
    delay(1000);
  }
}

//放出判定
void Released() {
  int j = 0;

  while(1) {
    double current_roll = Euler(0);

    WriteLog("roll angle", String(current_roll));

    if(fabs(current_roll) < 45) {
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
    delay(100);
  }
}

// 着地判定
void Landing() { 
  int j = 0;
  double init_roll = Euler(0);
  double init_pressure = bme280spi.Read_Pressure();
  unsigned long start_time = millis();

  delay(1000);

  while(1) {
    double current_roll = Euler(0);
    double current_pressure = bme280spi.Read_Pressure();
    double diff_roll = current_roll - init_roll;
    double diff_pressure = current_pressure - init_pressure;
    init_roll = current_roll;
    init_pressure = current_pressure;

    if(fabs(diff_roll) < 5 && fabs(diff_pressure) < 0.1) {
      j++;
    }
    else{
      j = 0;
    }
    double elapsed_time = (millis() - start_time) / 1000;

    WriteLog("roll angle change", String(diff_roll), "differential pressure", String(diff_pressure));

    if(j == 3 || elapsed_time > 30) {
      progress = "Landing";
      WriteLog();
      break;
    }
    else{
      WriteLog("time", String(elapsed_time));
    }

    delay(1000);
  }
}

// 溶断
void Fusing() {
  digitalWrite(FUSE_GPIO, HIGH);
  delay(100);
  digitalWrite(FUSE_GPIO, LOW);
  progress = "Fusing";
  WriteLog();
  Forward(200, 200);
  delay(2000);
  Stop();
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
  progress = "GPS guidance";
  WriteLog();

  while(1) {
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      gps.encode(c);
    }
    if (gps.location.isUpdated()){
      current_location[0] = gps.location.lat();
      current_location[1] = gps.location.lng();
      break;
    }
  }

  double distance = Distance(current_location[0], current_location[1], goal_location[0], goal_location[1]);
  double goal_azimuth = Azimuth(current_location[0], current_location[1], goal_location[0], goal_location[1]);
  if(goal_azimuth < 0) {
    goal_azimuth += 360;
  }

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
    if(diff_azimuth < -180) {
      diff_azimuth += 360;
    }

    if(fabs(diff_azimuth) > 30) {
      Turn(0, 100, 100);
    }
    else {
      Stop();
      break;
    }

    WriteLog("distance", String(distance), "azimuth", String(diff_azimuth));
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

    distance = Distance(current_location[0], current_location[1], goal_location[0], goal_location[1]);
    goal_azimuth = Azimuth(current_location[0], current_location[1], goal_location[0], goal_location[1]);

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
      if(diff_azimuth < -180) {
        diff_azimuth += 360;
      }

      int pwma = constrain(-Kp_gps * diff_azimuth + 200, 0, 255);
      int pwmb = constrain(Kp_gps * diff_azimuth + 200, 0, 255);
      Forward(pwma, pwmb);

      WriteLog("distance", String(distance), "azimuth", String(diff_azimuth));
    }
  }
}

// 画像誘導 
void Camera() {
  int x = -1, y = 0;
  double percentage;
  progress = "Image guidance";
  WriteLog();

  while(1) {
    while(Serial.available()) {
      Serial.read();
    }
    while(!Serial.available()) {
    }
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

      WriteLog("x", xStr, "percentage", percentageStr);
    }

    if(x == -1) {
      // 検出されないため、左右どちらかに旋回し続ける
      Turn(0, 80, 80);
    }
    else if(x >= 0) {
      int p_pwma= constrain(-Kp_camera * (x - pix) + 200, 0, 255);
      int p_pwmb = constrain(Kp_camera * (x - pix) + 200, 0, 255);
      Forward(p_pwma, p_pwmb);
    }

    // ゴール判定
    if(percentage > 40.0) {
      Forward(255, 255);
      delay(1000);
      Stop();
      progress = "Goal!";
      WriteLog();
      break;
    }
  }
}