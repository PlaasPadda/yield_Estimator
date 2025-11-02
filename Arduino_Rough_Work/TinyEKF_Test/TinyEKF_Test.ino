// ==== TinyEKF config (must come BEFORE including tinyekf.h) ====
#define EKF_N 5    // state dim: [x,y,vx,vy,heading]
#define EKF_M 3    // meas  dim: [gps_x, gps_y, heading]
#define _float_t float

#include <Wire.h>
#include <Adafruit_GPS.h>
#include "Adafruit_BNO08x_RVC.h"

// Put the TinyEKF header you pasted into your project as "tinyekf.h"
#include "tinyekf.h"

// ================== Hardware ==================
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
Adafruit_GPS GPS(&Wire);

// ESP32 pins you said work for you:
static const int BNO_RX = 16; // ESP32 RX2  (connect to BNO085 TX)
static const int BNO_TX = 17; // ESP32 TX2  (connect to BNO085 RX)
static const int GPS_SDA = 25;
static const int GPS_SCL = 33;

// ================ Noise Reduction ==================
float deadband(float x, float th) 
{
  if (fabsf(x) < th)
  {
    return 0.0f;
  }
  else
  {
    return x;
  }
}

float emaAlpha(float dt, float fc) {
  float RC = 1.0f / (2.0f * 3.14159265f * fc);
  return dt / (RC + dt);
}

struct EMA {
  float alpha;   // 0..1 (smaller = smoother)
  float y;       // filtered output
  bool init;
  void begin(float alpha_) { alpha = alpha_; init = false; y = 0; }
  float update(float x) {
    if (!init) { y = x; init = true; return y; }
    y += alpha * (x - y);
    return y;
  }
};

// ================= Calibraton ====================
uint32_t timer_IMU = millis();
const float G = 9.80665;

bool cal_done = false;
uint32_t cal_start_ms = 0;
const uint32_t CAL_TIME_MS = 5000; // 5 s still on boot

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

      //convert to radians
    yaw_rel = yaw_rel * rad;

    ax_out = deadband(ax_lin, 0.05f);
    ay_out = deadband(ay_lin, 0.05f);
  
    // ---- Print results ----
    if ((millis()-timer_IMU) >= 500)
    {
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Angles (relative to boot, deg)"));
    Serial.print(F("Yaw: "));   Serial.print(yaw_rel, 2);
    Serial.print(F("  Pitch: ")); Serial.print(pitch_rel, 2);
    Serial.print(F("  Roll: "));  Serial.println(roll_rel, 2);
  
    Serial.println(F("Linear Accel (gravity removed, m/s^2)"));
    Serial.print(F("X: ")); Serial.print(ax_out, 3);
    Serial.print(F("  Y: ")); Serial.print(ay_out, 3);
    Serial.print(F("  Z: ")); Serial.println(az_lin, 3);
    timer_IMU = millis();
    }
  }
    yaw_out = yaw_rel;
}

// ================== EKF instance ==================
ekf_t ekf;

// Covariances (tune these!)
_float_t Q[EKF_N*EKF_N] = {0};  // process noise
_float_t R[EKF_M*EKF_M] = {0};  // measurement noise

// ================== Timing ==================
unsigned long last_ms = 0;

// ================== GPS origin for local frame ==================
bool   have_origin = false;
double lat0_deg = 0.0, lon0_deg = 0.0;
double lat0_rad = 0.0, cos_lat0 = 1.0;

// quick and decent local meters conversion (ENU-ish)
static inline void ll_to_local_m(double lat_deg, double lon_deg, float &x_m, float &y_m) {
  // constants (good enough for small areas)
  const double m_per_deg_lat = 110540.0;   // meters/deg latitude
  const double m_per_deg_lon = 111320.0 * cos_lat0; // meters/deg longitude at origin

  x_m = (float)((lon_deg - lon0_deg) * m_per_deg_lon);
  y_m = (float)((lat_deg - lat0_deg) * m_per_deg_lat);
}

// ================== helpers ==================

// Build F (Jacobian of f wrt state) and fx (predicted state) for constant-heading model
// State x = [ x, y, vx, vy, heading ]
// Inputs   : ax_w, ay_w, dt, heading_meas (heading_meas not used in prediction; we keep heading constant here)
static void ekf_build_fx_F(const _float_t *x, _float_t ax_w, _float_t ay_w, _float_t dt,
                           _float_t *fx, _float_t *F)
{
  // Predicted state (discrete):
  // x'  = x  + vx*dt + 0.5*ax*dt^2
  // y'  = y  + vy*dt + 0.5*ay*dt^2
  // vx' = vx + ax*dt
  // vy' = vy + ay*dt
  // hd' = hd (keep)
  fx[0] = x[0] + x[2]*dt + 0.5f*ax_w*dt*dt;
  fx[1] = x[1] + x[3]*dt + 0.5f*ay_w*dt*dt;
  fx[2] = x[2] + ax_w*dt;
  fx[3] = x[3] + ay_w*dt;
  fx[4] = x[4];

  // F = df/dx
  // rows by cols (5x5)
  // [1, 0, dt, 0, 0]
  // [0, 1, 0, dt, 0]
  // [0, 0, 1,  0, 0]
  // [0, 0, 0,  1, 0]
  // [0, 0, 0,  0, 1]
  memset(F, 0, EKF_N*EKF_N*sizeof(_float_t));
  F[0*EKF_N + 0] = 1; F[0*EKF_N + 2] = dt;
  F[1*EKF_N + 1] = 1; F[1*EKF_N + 3] = dt;
  F[2*EKF_N + 2] = 1;
  F[3*EKF_N + 3] = 1;
  F[4*EKF_N + 4] = 1;
}

// Measurement model h(x) = [ x, y, heading ]
static void ekf_build_h_H(const _float_t *x, _float_t *hx, _float_t *H)
{
  hx[0] = x[0]; // gps x
  hx[1] = x[1]; // gps y
  hx[2] = x[4]; // heading

  // H is constant for this measurement
  // [1,0,0,0,0]
  // [0,1,0,0,0]
  // [0,0,0,0,1]
  memset(H, 0, EKF_M*EKF_N*sizeof(_float_t));
  H[0*EKF_N + 0] = 1;
  H[1*EKF_N + 1] = 1;
  H[2*EKF_N + 4] = 1;
}

// rotate body accel to world using heading (rad)
static inline void body_to_world_accel(float ax_b, float ay_b, float heading_rad,
                                       float &ax_w, float &ay_w)
{
  float c = cosf(heading_rad);
  float s = sinf(heading_rad);
  ax_w = ax_b * c - ay_b * s;
  ay_w = ax_b * s + ay_b * c;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // ---- IMU (BNO085 RVC) ----
  Serial1.begin(115200, SERIAL_8N1, BNO_RX, BNO_TX);
  if (!rvc.begin(&Serial1)) {
    Serial.println("ERROR: BNO08x RVC not found");
    while (1) delay(10);
  }
  Serial.println("BNO08x RVC OK");
  
  startCalibrationWindow();

  // ---- GPS (I2C) ----
  Wire.begin(GPS_SDA, GPS_SCL);
  GPS.begin(0x10);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(200);

  // ---- EKF init ----
  const _float_t Pdiag[EKF_N] = {
    1.0f,  // x  (m^2)
    1.0f,  // y
    1.0f,  // vx ((m/s)^2)
    1.0f,  // vy
    0.5f   // heading (rad^2)
  };
  ekf_initialize(&ekf, Pdiag);

  // Process noise Q (will scale by dt each step below; init here with something small)
  memset(Q, 0, sizeof(Q));
  Q[0*EKF_N+0] = 0.01f;
  Q[1*EKF_N+1] = 0.01f;
  Q[2*EKF_N+2] = 0.1f;
  Q[3*EKF_N+3] = 0.1f;
  Q[4*EKF_N+4] = 0.01f;

  // Measurement noise R
  memset(R, 0, sizeof(R));
  R[0*EKF_M+0] = 4.0f;    // gps x variance (m^2)  ~2m 1-sigma
  R[1*EKF_M+1] = 4.0f;    // gps y variance (m^2)
  R[2*EKF_M+2] = 0.01f;   // heading variance (rad^2) ~ ~3.2 deg^2

  last_ms = millis();
  Serial.println("TinyEKF ready");
}

void loop() {
  // ---------- Time step ----------
  unsigned long now = millis();
  float dt = (now - last_ms) * 0.001f;
  if (dt <= 0) dt = 0.001f;
  last_ms = now;

  // ---------- Read IMU (RVC) ----------
  BNO08x_RVC_Data imu;
  if (!rvc.read(&imu)) {
    // no new IMU data; still try to feed GPS parsing below
  }

  // Use IMU yaw as absolute heading (degrees -> radians)
  float heading_rad;

  // If your RVC gives linear acceleration (m/s^2), use directly:
  float ax_b;
  float ay_b;

  processSample(imu, heading_rad, ax_b, ay_b);

  // Rotate accel to world frame using IMU yaw
  float ax_w = 0, ay_w = 0;
  body_to_world_accel(ax_b, ay_b, heading_rad, ax_w, ay_w);

  // ---------- Build prediction (fx, F, Qscaled) ----------
  _float_t fx[EKF_N];
  _float_t F[EKF_N*EKF_N];

  ekf_build_fx_F(ekf.x, ax_w, ay_w, dt, fx, F);

  // Scale Q gently with dt (very simple scheme)
  _float_t Qscaled[EKF_N*EKF_N];
  memcpy(Qscaled, Q, sizeof(Qscaled));
  Qscaled[0*EKF_N+0] *= dt;
  Qscaled[1*EKF_N+1] *= dt;
  Qscaled[2*EKF_N+2] *= dt;
  Qscaled[3*EKF_N+3] *= dt;
  Qscaled[4*EKF_N+4] *= dt;

  // ---------- Predict ----------
  ekf_predict(&ekf, fx, F, Qscaled);

  // ---------- GPS parsing ----------
  // keep feeding the parser
  char c = GPS.read();
  (void)c; // silence unused warning

  bool have_new_gps = false;
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      if (GPS.fix) {
        if (!have_origin) {
          lat0_deg = GPS.latitudeDegrees;
          lon0_deg = GPS.longitudeDegrees;
          lat0_rad = lat0_deg * M_PI / 180.0;
          cos_lat0 = cos(lat0_rad);
          have_origin = true;
          Serial.println("GPS origin lat/lon set");
        }
        have_new_gps = true;
      }
    }
  }

  // ---------- Build measurement when we have GPS fix ----------
  if (have_origin && have_new_gps) {

    float gps_x_m = 0, gps_y_m = 0;
    ll_to_local_m(GPS.latitudeDegrees, GPS.longitudeDegrees, gps_x_m, gps_y_m);

    // z = [gps_x, gps_y, heading_meas]
    _float_t z[EKF_M];
    z[0] = gps_x_m;
    z[1] = gps_y_m;
    z[2] = heading_rad;

    // h(x) and H
    _float_t hx[EKF_N];     // NOTE: tinyekf uses EKF_N for hx buffer size
    _float_t H[EKF_M*EKF_N];
    ekf_build_h_H(ekf.x, hx, H);

    // ---------- Update ----------
    (void)ekf_update(&ekf, z, hx, H, R);
  }

  // ---------- Debug ----------
  Serial.print("x: ");  Serial.print(ekf.x[0], 3);
  Serial.print("  y: ");  Serial.print(ekf.x[1], 3);
  Serial.print("  vx: "); Serial.print(ekf.x[2], 3);
  Serial.print("  vy: "); Serial.print(ekf.x[3], 3);
  Serial.print("  hd(rad): "); Serial.println(ekf.x[4], 3);

  delay(20); // ~50 Hz loop
}
