// Test code for Adafruit GPS That Support Using I2C
//
// This code shows how to test a passthru between USB and I2C
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Wire.h>
#include <Adafruit_GPS.h>

Adafruit_GPS GPS(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Use working I2C pins
  Wire.begin(25, 33); // SDA, SCL

  Serial.println("Adafruit GPS library basic I2C test!");
  GPS.begin(0x10);  // GPS I2C address
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    GPS.write(c);
  }
  if (GPS.available()) {
    char c = GPS.read();
    Serial.write(c);
  }
}
