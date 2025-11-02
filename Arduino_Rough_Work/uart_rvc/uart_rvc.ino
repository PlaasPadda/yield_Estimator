/* Test sketch for Adafruit BNO08x sensor in UART-RVC mode */

#include "Adafruit_BNO08x_RVC.h"

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
uint32_t timer = millis();
const float G = 9.80665;

bool cal_done = false;
uint32_t cal_start_ms = 0;
const uint32_t CAL_TIME_MS = 2000; // 2 s still on boot

// Baselines/biases
float yaw0=0, pitch0=0, roll0=0;        // angle reference at boot
float ax_bias=0, ay_bias=0, az_bias=0;  // accel bias (after gravity removal)

// Running sums for bias calc
float sum_ax=0, sum_ay=0, sum_az=0;
uint32_t sample_count = 0;

void startCalibrationWindow() 
{
  cal_start_ms = millis();
  cal_done = false;
  sum_ax = sum_ay = sum_az = 0;
  sample_count = 0;
  Serial.println(F("Calibrating..."));
}

float processSample(const BNO08x_RVC_Data& h) 
{
  // Degrees -> radians
  float rad = 3.14159265358979f / 180.0f;
  float roll  = h.roll  * rad;   // φ
  float pitch = h.pitch * rad;   // θ
  // yaw doesn’t affect gravity direction relative to sensor axes

  // Gravity projected into body (sensor) frame using ZYX (yaw-pitch-roll) convention:
  float gx = -G * sinf(pitch);
  float gy = G * sinf(roll) * cosf(pitch);
  float gz = G * cosf(roll) * cosf(pitch);

  // Remove gravity to get "linear" acceleration
  float ax_lin = h.x_accel - gx;
  float ay_lin = h.y_accel - gy;
  float az_lin = h.z_accel - gz;

  // During first CAL_TIME_MS, accumulate baseline/bias while the unit is still
  if (!cal_done) 
  {
    if (millis() - cal_start_ms < CAL_TIME_MS) 
    {
      // take angle reference at the very first sample
      if (sample_count == 0) {
        yaw0 = h.yaw;
        pitch0 = h.pitch;
        roll0 = h.roll;
      }
      sum_ax += ax_lin; 
      sum_ay += ay_lin; 
      sum_az += az_lin;
      sample_count++;
    }
    else 
    {
      if (sample_count > 0)
      {
        ax_bias = sum_ax / sample_count;
        ay_bias = sum_ay / sample_count;
        az_bias = sum_az / sample_count;
      }
      cal_done = true;
      Serial.println(F("Calibration Complete"));
      Serial.println(F("[CAL] Baseline set. Biases (m/s^2):"));
      Serial.print(F("ax: ")); Serial.print(ax_bias, 4);
      Serial.print(F(" ay: ")); Serial.print(ay_bias, 4);
      Serial.print(F(" az: ")); Serial.println(az_bias, 4);
    }
  }
  else 
  {
    // Apply bias removal once calibrated
    ax_lin -= ax_bias;
    ay_lin -= ay_bias;
    az_lin -= az_bias;
  
    float yaw_rel   = h.yaw   - yaw0;
    float pitch_rel = h.pitch - pitch0;
    float roll_rel  = h.roll  - roll0;
  
    // ---- Print results ----
    if ((millis()-timer) >= 500)
    {
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Angles (relative to boot, deg)"));
    Serial.print(F("Yaw: "));   Serial.print(yaw_rel, 2);
    Serial.print(F("  Pitch: ")); Serial.print(pitch_rel, 2);
    Serial.print(F("  Roll: "));  Serial.println(roll_rel, 2);
  
    Serial.println(F("Linear Accel (gravity removed, m/s^2)"));
    Serial.print(F("X: ")); Serial.print(ax_lin, 3);
    Serial.print(F("  Y: ")); Serial.print(ay_lin, 3);
    Serial.print(F("  Z: ")); Serial.println(az_lin, 3);
    timer = millis();
    }
  }

  

  
}


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Adafruit BNO08x IMU - UART-RVC mode");

  // Change RX/TX pins to match your wiring!
  Serial1.begin(115200, SERIAL_8N1, 16, 17);

  if (!rvc.begin(&Serial1)) {
    Serial.println("Could not find BNO08x!");
    while (1) delay(10);
  }

  Serial.println("BNO08x found!");

  startCalibrationWindow();
  
}

void loop() {
  BNO08x_RVC_Data heading;

  if (!rvc.read(&heading)) {
    return; // no data yet
  }

  processSample(heading);

}
