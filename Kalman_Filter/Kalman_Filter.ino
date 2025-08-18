/* Test sketch for Kalman Filter */
#include <Wire.h>
#include <Adafruit_GPS.h>
#include "Adafruit_BNO08x_RVC.h"
#include <ukf.h>
#include <math.h>


Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
uint32_t timer_IMU = millis();
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

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer_GPS = millis();

// ------------------- Define constants -------------------
const int STATE_DIM = 5;       // pos_x, pos_y, vel_x, vel_y, heading
const int MEAS_DIM_GPS = 2;    // GPS: pos_x, pos_y
const int MEAS_DIM_HEADING = 1;// IMU: heading

// ------------------- Function prototypes -------------------
bool StateFunction(Matrix& X_dot, const Matrix& X, const Matrix& U);
bool MeasurementFunction(Matrix& Y_Est, const Matrix& X, const Matrix& U);

// ------------------- Global UKF variables -------------------
Matrix XInit(STATE_DIM,1);
Matrix PInit(STATE_DIM,STATE_DIM);
Matrix Rv(STATE_DIM,STATE_DIM);      // process noise
Matrix RnGPS(MEAS_DIM_GPS,MEAS_DIM_GPS);
Matrix RnHeading(MEAS_DIM_HEADING,MEAS_DIM_HEADING);

// Initialize UKF
UKF ukf(XInit, PInit, Rv, RnGPS, StateFunction, MeasurementFunction);

// Timing
unsigned long lastUpdate = 0;

void SPEW_THE_ERROR(const char* msg) {
  Serial.print("Matrix error: ");
  Serial.println(msg);
  while(1); // halt
}

void startCalibrationWindow() 
{
  cal_start_ms = millis();
  cal_done = false;
  sum_ax = sum_ay = sum_az = 0;
  sample_count = 0;
  Serial.println(F("Calibrating..."));
}

void processSample(const BNO08x_RVC_Data& h, float &yaw_out, float &ax_out, float &ay_out) 
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

  float yaw_rel;
  float pitch_rel;
  float roll_rel;

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
  
    yaw_rel   = h.yaw   - yaw0;
    pitch_rel = h.pitch - pitch0;
    roll_rel  = h.roll  - roll0;
  
    // ---- Print results ----
    if ((millis()-timer_IMU) >= 500)
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
    timer_IMU = millis();
    }
  }
  //convert to radians
  yaw_rel = yaw_rel * rad;
  yaw_out = yaw_rel;
  ax_out = ax_lin;
  ay_out = ay_lin;

  
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Adafruit BNO08x IMU - UART-RVC mode");

  Serial1.begin(115200, SERIAL_8N1, 16, 17);

  if (!rvc.begin(&Serial1)) {
    Serial.println("Could not find BNO08x!");
    while (1) delay(10);
  }

  Serial.println("BNO08x found!");

  startCalibrationWindow();

  while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  Serial.begin(115200);
  Serial.println("Adafruit I2C GPS library basic test!");
  Wire.begin(25, 33); // SDA, SCL


  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  GPS.println(PMTK_Q_RELEASE);

  // Initial state
  XInit(0,0) = 0; // pos_x
  XInit(1,0) = 0; // pos_y
  XInit(2,0) = 0; // vel_x
  XInit(3,0) = 0; // vel_y
  XInit(4,0) = 0; // heading

  // Initial covariance
  for(int i=0;i<STATE_DIM;i++)
      PInit(i,i) = 0.01;

  // Process noise (Rv) - uncertainty in state evolution
  Rv(0,0)=0.01; Rv(1,1)=0.01;
  Rv(2,2)=0.1;  Rv(3,3)=0.1;
  Rv(4,4)=0.001;

  // GPS measurement noise
  RnGPS(0,0)=2.0; RnGPS(1,1)=2.0;

  // Heading measurement noise
  RnHeading(0,0)=0.01;

  // Reset the filter with initial state/covariances
  ukf.vReset(XInit, PInit, Rv, RnGPS);

}

void loop() {
  // put your main code here, to run repeatedly:
  BNO08x_RVC_Data heading;

  if (!rvc.read(&heading)) {
    return; // no data yet
  }

  float yaw, ax, ay;
  processSample(heading, yaw, ax, ay);

  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer_GPS > 2000) {
    timer_GPS = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (!GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
     }
    
  }

  float dt = (millis() - lastUpdate)/1000.0;
  lastUpdate = millis();

  // ---- Read IMU ----
  float accel_x_body = ax;
  float accel_y_body = ay;
  float heading_yaw    = yaw;

  // Rotate acceleration to world frame
  float accel_x_world = accel_x_body * cos(heading_yaw) - accel_y_body * sin(heading_yaw);
  float accel_y_world = accel_x_body * sin(heading_yaw) + accel_y_body * cos(heading_yaw);

  // ---- Prediction input ----
  Matrix U(4,1); // control/input: [accel_x, accel_y]
  U(0,0) = accel_x_world;
  U(1,0) = accel_y_world;
  U(2,0) = dt;
  U(3,0) = heading_yaw;

  // ---- GPS Update (if new GPS data available) ----

  float gps_x = GPS.lat+10;
  float gps_y = GPS.lon+100;

  Matrix Y(MEAS_DIM_GPS,1);
  Y(0,0) = gps_x;
  Y(1,0) = gps_y;
  
  ukf.bUpdate(Y, U);


  // ---- Heading Update (optional) ----
  
//  Matrix Yheading(MEAS_DIM_HEADING,1);
//  Yheading(0,0) = heading_yaw;
//
//  // temporarily swap measurement noise to heading noise
//  Matrix oldRn = ukf.GetErr(); // reuse to store old R if needed
//  ukf.vReset(ukf.GetX(), ukf.GetP(), Rv, RnHeading);
//  ukf.bUpdate(Yheading, U);
//  // optionally reset R back to GPS noise
//  ukf.vReset(ukf.GetX(), ukf.GetP(), Rv, RnGPS);


  // ---- Debug Output ----
  Matrix Xest = ukf.GetX();
  Serial.print("X: "); Serial.print(Xest(0,0));
  Serial.print(" Y: "); Serial.print(Xest(1,0));
  Serial.print(" Vx: "); Serial.print(Xest(2,0));
  Serial.print(" Vy: "); Serial.print(Xest(3,0));
  Serial.print(" Heading: "); Serial.println(Xest(4,0));
  Serial.println(U(3,0));

}

// ------------------- Nonlinear state function -------------------
bool StateFunction(Matrix& X_pred, const Matrix& X, const Matrix& U) {
    float dt = U(2,0);
    float accel_x = U(0,0);
    float accel_y = U(1,0);
    float heading = X(4,0); // previous heading

    // Predict next state
    X_pred(0,0) = X(0,0) + X(2,0)*dt + 0.5f*accel_x*dt*dt; // pos_x
    X_pred(1,0) = X(1,0) + X(3,0)*dt + 0.5f*accel_y*dt*dt; // pos_y
    X_pred(2,0) = X(2,0) + accel_x*dt;                     // vel_x
    X_pred(3,0) = X(3,0) + accel_y*dt;                     // vel_y
    X_pred(4,0) = U(3,0);                                  // heading from IMU

    return true;
}

// ------------------- Nonlinear measurement function -------------------
bool MeasurementFunction(Matrix& Y_Est, const Matrix& X, const Matrix& U) {
  // We only estimate GPS position here (can extend for heading)
  Y_Est(0,0) = X(0,0); // pos_x
  Y_Est(1,0) = X(1,0); // pos_y
  Y_Est(2,0) = X(4,0); // heading
  return true;
}
