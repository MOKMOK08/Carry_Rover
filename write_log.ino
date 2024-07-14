#include "FS.h"
#include "SD.h"
#include "SPI.h"

File file;
String gps_time;
String progress = "ready";
String file_name = "CarryRover";

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
}

void WriteLog(String data_name1 = "", String data1 = "", String data_name2 = "", String data2 = "") {
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
