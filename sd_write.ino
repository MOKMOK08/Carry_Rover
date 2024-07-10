#include "FS.h"
#include "SD.h"
#include "SPI.h"

File file;
const char* FILE_NAME = "CarryRover";

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

  CreateFile(SD, FILE_NAME);
  WriteFile("Hello ");
}

void loop() {
}

void CreateFile(fs::FS &fs, const char *baseFilename) {
  String filename = String("/") + baseFilename + ".txt";
  int counter = 1;

  while (fs.exists(filename.c_str())) {
    filename = String("/") + baseFilename + String(counter) + ".txt";
    counter++;
  }

  Serial.printf("Creating file: %s\n", filename.c_str());

  file = fs.open(filename.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create file");
  }
}

void WriteFile(const char *message) {
  if (!file) {
    Serial.println("No file opened for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message written to file");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
