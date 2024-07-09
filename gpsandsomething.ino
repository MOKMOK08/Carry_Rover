//GPS誘導に関するいろいろ
//20240709

#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPSPlus.h>
TinyGPSPlus gps;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

double goalGPSdata2[2] = {35.7631874, 139.8962477};// 早川の家
double eulerdata[3] = {0,0,0};
double currentlocation[2]={gps.location.lat(),gps.location.lng()};
double azidata[2] = {0,0};

//正しく動かない場合、0x28→0x29に変える
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);//ICSの名前, デフォルトアドレス



//PIN
//PID制御のための定数
#define Kp 1
//モータードライバピン番号未変更
const int STBY = 2;     // モータードライバの制御の準備
const int AIN1 = 3;     // 1つ目のDCモーターの制御
const int AIN2 = 4;     // 1つ目のDCモーターの制御
const int BIN1 = 7;     // 2つ目のDCモーターの制御
const int BIN2 = 8;     // 2つ目のDCモーターの制御
const int PWMA = 5;     // 1つ目のDCモーターの回転速度
const int PWMB = 6;    // 2つ目のDCモーターの回転速度
//int RX_PIN = 16;
//int TX_PIN = 17;

void setup(void)
{
  // シリアルポート開始
  Serial.begin(115200);
  //Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(21, INPUT_PULLUP); //SDA 
  pinMode(22, INPUT_PULLUP); //SDL
  
  //BNO setup
  while (!Serial) delay(10);  // wait for serial port to open!
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* BNO エラー*/
    Serial.print("BNO ERROR");
    while (1);
  }

  delay(1000);

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

double AzimuthDistance(){
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
void P_GPS_Moter(){ 
    Serial.println("P_GPS_Moter");
    while(true){
    AzimuthDistance();
    if(azidata[1] < 5){
        break;
        }
    else{
        int PID_left = 0.65 * azidata[0] + 126;
        int PID_right = - 0.65 * azidata[0] + 126;
        MoterControl(PID_left, PID_right);
        delay(250);
        }
    } 
}
/*
double GPS_motor(goallat2,goallon2){
  Serial.println("P_GPS_motor");

  GetAzimuthDistance(golat,goallon2);
  turn_data.dataAdd(azidata[0]);
  dit_data.dataAdd(azidata[1]);
}
*/

void loop(void){
  int i,j,k;
  double lat_off;
  double longt_off;
  double xi,yi,xi2,yi2,xiyi,W1,W2,W3,cx,cy,r;
  double T[2][2],X[2],W[2];
  double Mag_x[200],Mag_y[200];
  double  mag_x,mag_y,mag_z;
  double gps_lat, gps_lon;
  double goalGPSdata2[2] = {35.7631874, 139.8962477};// 早川の家
  double distance;
  double currentlocation[2]={gps.location.lat(),gps.location.lng()};

  //GPSの値から目的地との距離算出
  distance=distanceBetween(goalGPSdata2[0],goalGPSdata2[1],currentlocation[0],currentlocation[1]);
  Serial.println(distance);

  //方位角(azimuth)算出
  imu::Quaternion quat = bno.getQuat();
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
  //
    
// 磁力センサ値の取得と表示
/*
  imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print("Mag_x");
  Serial.println(magnetmetor .x());
  Serial.print(" Mg_y:");
  Serial.println(magnetmetor .y());
*/

  //GPS取得
  Serial.print("GPS Lat");
  gps_lat = gps.location.lat();
  Serial.println(gps_lat);
  Serial.print("GPS Lng");
  gps_lon = gps.location.lng();
  Serial.println(gps_lon);

 

  
}
