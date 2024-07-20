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
const double goalGPSdata2[2] = {35.7631874, 139.8962477};// 早川の家
double azidata[2] = {0,0};

// SDカードの設定
File file;
String progress = "Ready"; // シーケンス
String file_name;

// Bluetoothの設定
BluetoothSerial SerialBT;

//PIN
//PID制御のための定数
#define Kp 1

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

  Serial.begin(115200);

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

  // Bluetoothの初期化
  SerialBT.begin("cansatESP32"); //デバイス名
  while(!SerialBT.hasClient()) {
    delay(1000);
  }

  SerialBT.println("Bluetooth connected!");

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
  while(!gps.location.isValid() && !gps.date.isValid() && !gps.time.isValid()) {
    SerialBT.println("Waiting for GPS signal...");
    delay(1000);
  }

  SerialBT.println("GPS signal acquired!"); 

  CreateFile("cansat"); // ファイル名
  delay(1000);
}

void loop() {
  Start();
  Released();
  Landing();
  Fusing();
  P_GPS_Moter();
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
  if(a = 0) {
  analogWrite(PWMA, i);
  analogWrite(PWMB, j);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  }

  if(a = 1) {
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

// スタート判定
void Start() {
  int i = 0;
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
      ave_roll += Euler(0);
      ave_pressure += bme280spi.Read_Pressure();
      delay(10);
    }
    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - init_pressure;

    if(fabs(ave_roll) > 45 && fabs(ave_roll) < 135 && diff_pressure < -0.5) {
      progress = "Start";
      WriteLog();
      break;
    }
  
    WriteLog("roll angle", String(ave_roll), "differential pressure", String(diff_pressure));
  }
}

//放出判定
void Released() {
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度

  while(1) {
    for(i = 0; i < 10; i++) {
      ave_roll += Euler(0);
      delay(10);
    }
    ave_roll /= 10;

    if(fabs(ave_roll) < 45) {
      j++;
    }
    else {
      j = 0;
    }

    if(j >= 5) {
      progress = "Released";
      break;
    }

    WriteLog("roll angle", String(ave_roll));
  }
}

// 着地判定
void Landing() { 
  int i = 0, j = 0;
  unsigned long start_time = millis();
  unsigned long current_time = millis();
  unsigned long elapsed_time = 0.0; // 経過時間
  double ave_roll = 0.0; // 平均ロール角度
  double init_roll = 0.0; // 初期ロール角度
  double diff_roll = 0.0; // 角度変化
  double ave_pressure = 0.0; // 平均気圧
  double init_pressure = 0.0; // 初期気圧
  double diff_pressure = 0.0; // 差圧

  for(i = 0; i < 10; i++) {
    init_roll += Euler(0);
    init_pressure += bme280spi.Read_Pressure();
    delay(10);
  }

  init_roll /= 10;
  init_pressure /= 10;
  
  delay(500);

  while(1) {
    for(i = 0; i < 10; i++) {
      ave_roll += Euler(0);
      ave_pressure += bme280spi.Read_Pressure();
      delay(10);
    }

    ave_roll /= 10;
    diff_roll = ave_roll - init_roll;
    init_roll = ave_roll;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - init_pressure;
    init_pressure = ave_pressure;

    if(fabs(diff_roll) < 0.1 && fabs(diff_pressure) < 0.1) {
      j++;
    }
    else{
      j = 0;
    }

    WriteLog("roll angle change", String(diff_roll), "differential pressure", String(diff_pressure));
    elapsed_time = current_time - start_time;

    if(j == 5 || elapsed_time > 30000) {
      progress = "Landing";
      WriteLog();
      break;
    }
    else{
      WriteLog("time", String(elapsed_time));
      current_time = millis();
    }

    delay(500);
  }
}

// 溶断
void Fusing() {
  digitalWrite(FUSE_GPIO, HIGH);
  delay(500);
  digitalWrite(FUSE_GPIO, LOW);
  progress = "Fusing";
  WriteLog();
}

// ２点間の距離計算
double distanceBetween(double lat1, double long1, double lat2, double long2){
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

void AzimuthDistance() {
  imu::Quaternion quat = bno.getQuat();
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
  }
  double currentlocation[2]={gps.location.lat(),gps.location.lng()};

    Euler();
    //方位角取得
    double x1 = currentlocation[0] * M_PI / 180.0;
    double y1 = currentlocation[1] * M_PI / 180.0;
    double x2 = goalGPSdata2[0] * M_PI / 180.0;
    double y2 = goalGPSdata2[1] * M_PI / 180.0;

    // x と y の計算
    double x_dif = x2 - x1;
    double y_dif = y2 - y1;

    // 方位角の計算
    double azimuth_rad = atan2(sin(y_dif),(cos(x1)*tan(x2)-sin(x1)*cos(y_dif)));
    double azimuth = azimuth_rad * 180.0 / M_PI;
     
    double turnpower;
    turnpower = azimuth -eulerdata[2] ;  
    azidata[0] = turnpower;
    azidata[1] = distanceBetween(goalGPSdata2[0],goalGPSdata2[1],currentlocation[0],currentlocation[1]);
}

// GPS誘導
void P_GPS_Moter() { 
  progress = "GPS guidance";
  WriteLog();
  
  while(true){
    AzimuthDistance();

    if(azidata[1] < 5){
      Stop();
      progress = "GPS guidance completed";
      WriteLog();
      break;
    }
    else{
      int PID_left = 0.65 * azidata[0] + 126; // 0.65は車輪半径
      int PID_right = - 0.65 * azidata[0] + 126;
      Forward(PID_left, PID_right);
      WriteLog("distance to goal", String(azidata[1]), "azimuth", String(azidata[0]));
      delay(250);
    }
  }
}

void Camera() {
  int x, y;
  float percentage;
  progress = "Camera guidance";
  WriteLog();

  // データが受信されている場合
  while(1){
    if (Serial.available()) {
      // 受信データを読み取る
      String receivedData = Serial.readStringUntil('\n');
      
      // 受信データをコンマで分割
      int firstComma = receivedData.indexOf(',');
      int secondComma = receivedData.indexOf(',', firstComma + 1);
      int spaceAfterSecondComma = receivedData.indexOf(' ', secondComma + 1);
      
      if (firstComma > 0 && secondComma > 0 && spaceAfterSecondComma > 0) {
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
        
        WriteLog("x coordinate", xStr, "screen occupancy", percentageStr);
      }
    }
  
    // 短い遅延
    delay(10);
    
    if (x - pix > 10){
      //モーターを左右どちらかに回転させるプログラムを書く
      Turn(0, 100, 100);
    }
    else if(x - pix < -10){
      //モーターを左右どちらかに回転させるプログラムを書く
      Turn(1, 100, 100);
    }
    else if(x = -1){
      //検出されないため、左右どちらかに旋回し続ける
      Turn(1, 100, 100);
    }
    else if(abs(x - pix) <= 10){
    //回転を停止して前進
      Stop();
      for(int i = 0; i < 256; i++){
        Forward(i,i);
      }
      if(percentage > 80.0){
        //ゴール判定
        Stop();
        progress = "Goal!";
        WriteLog();
        break;
      }
    }
  }
}
