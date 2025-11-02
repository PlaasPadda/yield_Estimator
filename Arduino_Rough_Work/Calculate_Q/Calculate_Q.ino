#include "Adafruit_BNO08x.h"
Adafruit_BNO08x bno;
const int BNO_RX = 16; // ESP32 RX2  (connect to BNO085 TX)
const int BNO_TX = 17; // ESP32 TX2  (connect to BNO085 RX)

void measureAccelNoiseDensity(HardwareSerial &dbgSerial, uint16_t N = 2000, float sample_dt = 0.01f) {
  // N samples at ~1/sample_dt => fs ~ 1/sample_dt
  const float fs = 1.0f / sample_dt;
  float sumx=0, sumy=0, sumz=0;
  float sumx2=0, sumy2=0, sumz2=0;

  dbgSerial.println("Collecting accel samples...");
  for (uint16_t i=0; i<N; ++i) {
    // read your sensor sample - replace with your read call
    sh2_SensorValue_t sv;
    if (!bno.getSensorEvent(&sv)) {
      delay((int)(sample_dt*1000));
      --i; // try again
      continue;
    }
    float ax = sv.un.linearAcceleration.x; // m/s^2
    float ay = sv.un.linearAcceleration.y;
    float az = sv.un.linearAcceleration.z;

    sumx += ax; sumy += ay; sumz += az;
    sumx2 += ax*ax; sumy2 += ay*ay; sumz2 += az*az;

    delay((int)(sample_dt*1000));
  }

  float meanx = sumx / N;
  float meany = sumy / N;
  float meanz = sumz / N;

  float varx = (sumx2 / N) - (meanx*meanx);
  float vary = (sumy2 / N) - (meany*meany);
  float varz = (sumz2 / N) - (meanz*meanz);

  float sigma_x = sqrtf(varx);
  float sigma_y = sqrtf(vary);
  float sigma_z = sqrtf(varz);

  // noise density n_a = sigma_samples / sqrt(fs/2)
  float n_x = sigma_x / sqrtf(fs * 0.5f);
  float n_y = sigma_y / sqrtf(fs * 0.5f);
  float n_z = sigma_z / sqrtf(fs * 0.5f);

  float S_a_x = n_x * n_x; // m^2 / s^3
  float S_a_y = n_y * n_y;
  float S_a_z = n_z * n_z;

  dbgSerial.print("fs: "); dbgSerial.println(fs);
  dbgSerial.print("sigma_x (m/s^2): "); dbgSerial.println(sigma_x, 6);
  dbgSerial.print("sigma_y (m/s^2): "); dbgSerial.println(sigma_y, 6);
  dbgSerial.print("sigma_z (m/s^2): "); dbgSerial.println(sigma_z, 6);

  dbgSerial.print("noise density x (m/s/√Hz): "); dbgSerial.println(n_x, 9);
  dbgSerial.print("noise density y (m/s/√Hz): "); dbgSerial.println(n_y, 9);
  dbgSerial.print("noise density z (m/s/√Hz): "); dbgSerial.println(n_z, 9);

  dbgSerial.print("Power Spectral Density x (m^2/s^3): "); dbgSerial.println(S_a_x, 9);
  dbgSerial.print("Power Spectral Density y (m^2/s^3): "); dbgSerial.println(S_a_y, 9);
  dbgSerial.print("Power Spectral Density z (m^2/s^3): "); dbgSerial.println(S_a_z, 9);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Initialize Serial1 for the BNO08x (pins depend on your board!)
  Serial1.begin(3000000, SERIAL_8N1, BNO_RX, BNO_TX);  // most BNO085 boards default to 3Mbaud

  if (!bno.begin_UART(&Serial1)) {
    Serial.println("Failed to find BNO08x chip over UART");
  }

  Serial.println("BNO08x OK");

  // Enable linear acceleration report
  if (!bno.enableReport(SH2_LINEAR_ACCELERATION)) 
  {
    Serial.println("Could not enable linear acceleration!");
  }

  if (!bno.enableReport(SH2_ROTATION_VECTOR))            // 9DoF fused
  {
    Serial.println("Could not enable Rotation Vector!");
  }

  measureAccelNoiseDensity(Serial, 2000, 0.01f);

}

void loop() {
  // put your main code here, to run repeatedly:

}
