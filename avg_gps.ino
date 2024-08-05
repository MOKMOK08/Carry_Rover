#include <TinyGPS++.h>
#include <SoftwareSerial.h>
 
TinyGPSPlus gps;
SoftwareSerial mySerial(16, 17); // RX, TX
//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);
 
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Goodnight moon!");
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
}
 
void loop() {
  int i = 0;
  double lat; 
  double lng;
  
  while(1) {
    while (mySerial.available() > 0){
      char c = mySerial.read();
      gps.encode(c);
    }

    if (gps.location.isValid() && gps.location.isUpdated()) {
      lat += gps.location.lat();
      lng += gps.location.lng();
      i++;
    }    
    else if(!gps.location.isValid() && !gps.location.isUpdated()) {
      Serial.println("Waiting for GPS signal...");
    }

    if(i == 19) {
      lat /= 20;
      lng /= 20;
      Serial.print("LAT ");
      Serial.println(lat, 7);
      Serial.print("LNG ");
      Serial.println(lng, 7);
      i = 0;
    }
  }
}