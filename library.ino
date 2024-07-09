//ライブラリだけ記載
//hayakawa
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPSPlus.h>
TinyGPSPlus gps;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

//正しく動かない場合、0x28→0x29に変える
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);//ICSの名前, デフォルトアドレス

//
