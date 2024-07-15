#include <TinyGPS++.h>

String gps_time;
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX = 16, TX = 17
}

void loop() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

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
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
    gps_time = String(buffer);
  }
  else {
    Serial.println("Waiting for GPS signal...");
  }
}

