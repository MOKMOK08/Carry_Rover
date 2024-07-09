#include "FS.h"
#include "SD.h"
#include "SPI.h"

const char* FILE_NAME = "example";

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

  createFile(FILE_NAME);

  writeFile("Hello, this is a new file!");
}

void loop() {
}

void createFile(const char* baseFilename) {
  String filename = String(baseFilename) + ".txt";
  int counter = 1;

  while (SD.exists(filename.c_str())) {
    filename = String(baseFilename) + String(counter) + ".txt";
    counter++;
  }

  file = SD.open(filename.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create file");
    return;
  }
  Serial.print("Created file: ");
  Serial.println(filename);
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
