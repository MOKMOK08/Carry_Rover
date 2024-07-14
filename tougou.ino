#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>
#include <ESP32_BME280_SPI.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// GPSの設定
TinyGPSPlus gps;
double goalGPSdata2[2] = {35.7631874, 139.8962477};// 早川の家
double currentlocation[2]={gps.location.lat(),gps.location.lng()};
double azidata[2] = {0,0};

// BNO055の設定
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// BME280の設定
const uint8_t SCLK_bme280 = 14;
const uint8_t MOSI_bme280 = 13;
const uint8_t MISO_bme280 = 12;
const uint8_t CS_bme280 = 15;
ESP32_BME280_SPI bme280spi(SCLK_bme280, MOSI_bme280, MISO_bme280, CS_bme280, 10000000);

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

  //溶断回路の初期化
  pinMode(FUSE_GPIO, OUTPUT);
  digitalWrite(FUSE_GPIO, LOW);
  
  delay(1000);
}

void loop() {
  Start();
  Released();
  Landing();
  Fusing();
  P_GPS_Moter();
  camera();
  exit(0); //loopを1回で終了
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
      ave_roll += fabs(Euler(0));
      ave_pressure += bme280spi.Read_Pressure();
      delay(10);
    }

    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - init_pressure;

    if(ave_roll > 45 && ave_roll < 135 && diff_pressure < -0.5) {
      progress = "開始";
      WriteLog();
      break;
    }
  
    WriteLog("ロール角", String(ave_roll), "差圧", String(diff_pressure));
  }
}

//放出判定
void Released() {
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度

  while(1) {
    for(i = 0; i < 10; i++) {
      ave_roll += fabs(Euler(0));
      delay(10);
    }
    ave_roll /= 10;

    if(ave_roll < 45) {
      j++;
    }
    else {
      j = 0;
    }

    if(j >= 5) {
      progress = "放出";
      break;
    }

    WriteLog("ロール角", String(ave_roll));
  }
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

// 溶断
void Fusing() {
  digitalWrite(FUSE_GPIO, HIGH);
  delay(500);
  digitalWrite(FUSE_GPIO, LOW);
  progress = "溶断完了";
  WriteLog();
}

double distanceBetween(double lat1, double long1, double lat2, double long2){
    // returns distance in meters between two positions, both specified
    // as signed decimal-degrees latitude and longitude. Uses great-circle
    // distance computation for hypothetical sphere of radius 6372795 meters.
    // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    // Courtesy of Maarten Lamers
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
void GPSget(){
double currentlocation[2]={gps.location.lat(),gps.location.lng()};
}

void AzimuthDistance(){
  imu::Quaternion quat = bno.getQuat();
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
  //方位角取得
  double azimuth= atan2(y,x)*180.0/M_PI;
    Serial.print(azimuth);
    Serial.println("degree");
  GPSget;
  double turnpower;
    turnpower = currentlocation[2] - Euler(2);
    
    azidata[0] = turnpower;
    azidata[1] = distanceBetween(goalGPSdata2[0],goalGPSdata2[1],currentlocation[0],currentlocation[1]);
    Serial.print("\tDistance: ");
    Serial.println(azidata[1]);
}

void P_GPS_Moter(){ 
    Serial.println("P_GPS_Moter");
    while(true){
    AzimuthDistance();
    if(azidata[1] < 5){
        Stop();
        progress="GPSシーケンス終了"
        WriteLog("赤コーンとの距離", String(azidata[1]), "方位角", String(azidata[0]));
        break;
        }
    else{
        int PID_left = 0.65 * azidata[0] + 126; // 0.65は車輪半径
        int PID_right = - 0.65 * azidata[0] + 126;
        Forward(PID_left, PID_right);
        WriteLog("赤コーンとの距離", String(azidata[1]), "方位角", String(azidata[0]));
        delay(250);
        }
    }
}



void camera(){
  int x, y;
  float percentage;
  progress = "画像誘導";
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
        
        WriteLog("x座標", xStr, "画面占有率", percentageStr);
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
        progress = "ゴール";
        WriteLog();
        break;
      }
    }
  }
}
