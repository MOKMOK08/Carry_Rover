#include "FS.h"
#include "SD.h"
#include "SPI.h"

const char* FILE_NAME = "example.txt";

File file;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }

  if (!createFile(FILE_NAME)) {
    Serial.println("Failed to create file");
    return;
  }

  writeFile("Hello, this is a new file!");
}

void loop() {
}

bool createFile(const char* filename) {
  if (SD.exists(filename)) {
    Serial.println("File already exists");
    return false;
  }

  file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create file");
    return false;
  }
  return true;
}

void writeFile(const char* message) {
  if (!file) {
    Serial.println("No file opened for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
}