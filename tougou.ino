#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <TinyGPSPlus.h>
#include <ESP32_BME280_SPI.h>
TinyGPSPlus gps;

// BNO055の設定
double eulerdata[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;  //from hayakawa, 不要なら削除

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


double goalGPSdata2[2] = {35.7631874, 139.8962477};// 早川の家
double currentlocation[2]={gps.location.lat(),gps.location.lng()};
double azidata[2] = {0,0};

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
  
  delay(1000);
}

void loop() {
  Release();
  Landing();
  Fusing();
  P_GPS_Moter();
  exit(0); //loopを1回で終了
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

//放出判定
void Release() {
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度
  double ave_pressure = 0.0; // 平均気圧
  double pre_pressure = 0.0; // 前の気圧
  double diff_pressure = 0.0; // 差圧

  for(i = 0; i < 10; i++) {
    pre_pressure += (uint16_t)round(bme280spi.Read_Pressure());
    delay(10);
  }

  pre_pressure /= 10;

  // 頂点判定
  while(1) {
    for(i = 0; i < 10; i++) {
      Euler();
      ave_roll += fabs(eulerdata[0]);
      ave_pressure += (uint16_t)round(bme280spi.Read_Pressure());
      delay(10);
    }

    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - pre_pressure;
    pre_pressure = ave_pressure;

    if(ave_roll > 90 && diff_pressure > 0.3) {
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

  // 放出判定
  while(1) {
    for(i = 0; i < 10; i++) {
      Euler();
      ave_roll += fabs(eulerdata[0]);
      ave_pressure += (uint16_t)round(bme280spi.Read_Pressure());
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

// 着地判定
void Landing(){ 
  unsigned long start_time = millis();
  unsigned long current_time = millis();
  int i = 0, j = 0;
  double ave_roll = 0.0; // 平均ロール角度
  double ave_pressure = 0.0; // 平均気圧
  double pre_pressure = 0.0; // 前の気圧
  double diff_pressure = 0.0; // 差圧

  for(i = 0; i < 10; i++){
    pre_pressure += (uint16_t)round(bme280spi.Read_Pressure());
    delay(10);
  }

  pre_pressure /= 10;
  
  while(1) {
    for(i = 0; i < 10; i++) {
      Euler();
      ave_roll += fabs(eulerdata[0]);
      ave_pressure += (uint16_t)round(bme280spi.Read_Pressure());
      delay(10);
    }

    ave_roll /= 10;
    ave_pressure /= 10;
    diff_pressure = ave_pressure - pre_pressure;
    pre_pressure = ave_pressure;

    if(ave_roll < 45 && diff_pressure < 0.01) {
      j++;
    }
    else{
      j = 0;
    }

    if(j == 10 || current_time - start_time > 20000){
      break;
    }
    else{
      current_time = millis();
    }
  }

  Serial.println("landing");
}

void Fusing() {
  digitalWrite(FUSE_GPIO, HIGH);
  delay(500);
  digitalWrite(FUSE_GPIO, LOW);
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
  double turnpower;
    turnpower = currentlocation[2] - eulerdata[2];
    
    azidata[0] = turnpower;
    azidata[1] = distanceBetween(goalGPSdata2[0],goalGPSdata2[1],currentlocation[0],currentlocation[1]);
    Serial.print("\tDistance: ");
    Serial.println(azidata[1]);
  
}

//左右の回転速度を0基準に設定(v∈[-255,255])y
void MoterControl( int left,int right) {
    int absleft = abs(left);
    int absright = abs(right);

    if(left >= 0 && right >= 0){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
    else if(left >= 0 && right < 0){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
    else if(left < 0 && right >= 0){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
    else{
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMA, absleft);
        analogWrite(PWMB, absright);
    }
}

void P_GPS_Moter(){ 
    Serial.println("P_GPS_Moter");
    while(true){
    AzimuthDistance();
    if(azidata[1] < 5){
        break;
        }
    else{
        int PID_left = 0.65 * azidata[0] + 126;  \\0.65は車輪半径
        int PID_right = - 0.65 * azidata[0] + 126;
        MoterControl(PID_left, PID_right);
        delay(250);
        }
    } 
}
