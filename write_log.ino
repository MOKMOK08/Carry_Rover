#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>

File file;
String gps_time;
String progress = "ready";
String file_name = "CarryRover";

HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

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

  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
}

void WriteLog(String data_name1 = "", String data1 = "", String data_name2 = "", String data2 = "") {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();
    int centisecond = gps.time.centisecond();

    // JSTに変換
    hour += 9;
    if (hour >= 24) {
      hour -= 24;
      day += 1;

      if ((month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12) && day > 31) {
        day = 1;
        month += 1;
      } 
      else if ((month == 4 || month == 6 || month == 9 || month == 11) && day > 30) {
        day = 1;
        month += 1;
      } 
      else if (month == 2) {
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
          if (day > 29) {
            day = 1;
            month += 1;
          }
        } 
        else {
          if (day > 28) {
            day = 1;
            month += 1;
          }
        }
      }

      if (month > 12) {
        month = 1;
        year += 1;
      }
    }

  char buffer[30];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d.%02d", year, month, day, hour, minute, second, centisecond);
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

void loop() {
  WriteLog("1", "2", "3", "4");
}