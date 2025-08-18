#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BNO08x UART Test");

  // Initialize Serial1 for the BNO08x (pins depend on your board!)
  Serial1.begin(3000000, SERIAL_8N1, 16, 17);  // most BNO085 boards default to 3Mbaud

  if (!bno.begin_UART(&Serial1)) {
    Serial.println("Failed to find BNO08x chip over UART");
  }

  Serial.println("BNO08x Found!");

  // Enable linear acceleration report
  if (!bno.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration!");
  }
}

void loop() {
  sh2_SensorValue_t sensorValue;
  if (bno.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      Serial.print("Lin Accel X: ");
      Serial.print(sensorValue.un.linearAcceleration.x);
      Serial.print("  Y: ");
      Serial.print(sensorValue.un.linearAcceleration.y);
      Serial.print("  Z: ");
      Serial.println(sensorValue.un.linearAcceleration.z);
    }
  }
}
