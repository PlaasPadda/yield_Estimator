// ==== TinyEKF config (must come BEFORE including tinyekf.h) ====
#define EKF_N 4    // state dim: [x,y,vx,vy]
#define EKF_M 2    // meas  dim: [gps_x, gps_y]
#define _float_t float

#include <Wire.h>
#include <Adafruit_GPS.h>
#include "Adafruit_BNO08x.h"

// Put the TinyEKF header you pasted into your project as "tinyekf.h"
#include "tinyekf.h"

// ================== Hardware ==================
Adafruit_BNO08x bno;
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
// State x = [ x, y, vx, vy ]
// Inputs   : ax_w, ay_w, dt
static void ekf_build_fx_F(const _float_t *x, _float_t ax_w, _float_t ay_w, _float_t dt,
                           _float_t *fx, _float_t *F)
{
  // Predicted state (discrete):
  // x'  = x  + vx*dt + 0.5*ax*dt^2
  // y'  = y  + vy*dt + 0.5*ay*dt^2
  // vx' = vx + ax*dt
  // vy' = vy + ay*dt
  fx[0] = x[0] + x[2]*dt + 0.5f*ax_w*dt*dt;
  fx[1] = x[1] + x[3]*dt + 0.5f*ay_w*dt*dt;
  fx[2] = x[2] + ax_w*dt;
  fx[3] = x[3] + ay_w*dt;

  // F = df/dx
  // rows by cols (5x5)
  // [1, 0, dt, 0, 0]
  // [0, 1, 0, dt, 0]
  // [0, 0, 1,  0, 0]
  // [0, 0, 0,  1, 0]
  memset(F, 0, EKF_N*EKF_N*sizeof(_float_t));
  F[0*EKF_N + 0] = 1; F[0*EKF_N + 2] = dt;
  F[1*EKF_N + 1] = 1; F[1*EKF_N + 3] = dt;
  F[2*EKF_N + 2] = 1;
  F[3*EKF_N + 3] = 1;
}

// Measurement model h(x) = [ x, y ]
static void ekf_build_h_H(const _float_t *x, _float_t *hx, _float_t *H)
{
  hx[0] = x[0]; // gps x
  hx[1] = x[1]; // gps y

  // H is constant for this measurement
  // [1,0,0,0,0]
  // [0,1,0,0,0]
  memset(H, 0, EKF_M*EKF_N*sizeof(_float_t));
  H[0*EKF_N + 0] = 1;
  H[1*EKF_N + 1] = 1;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Initialize Serial1 for the BNO08x (pins depend on your board!)
  Serial1.begin(3000000, SERIAL_8N1, BNO_RX, BNO_TX);  // most BNO085 boards default to 3Mbaud

  if (!bno.begin_UART(&Serial1)) {
    Serial.println("Failed to find BNO08x chip over UART");
  }

  Serial.println("BNO08x OK");

  // Enable linear acceleration report
  if (!bno.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration!");
  }

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
  };
  ekf_initialize(&ekf, Pdiag);

  // Process noise Q (will scale by dt each step below; init here with something small)
  memset(Q, 0, sizeof(Q));
  Q[0*EKF_N+0] = 0.01f;
  Q[1*EKF_N+1] = 0.01f;
  Q[2*EKF_N+2] = 0.1f;
  Q[3*EKF_N+3] = 0.1f;

  // Measurement noise R
  memset(R, 0, sizeof(R));
  R[0*EKF_M+0] = 4.0f;    // gps x variance (m^2)  ~2m 1-sigma
  R[1*EKF_M+1] = 4.0f;    // gps y variance (m^2)
  
  last_ms = millis();
  Serial.println("TinyEKF ready");
}

void loop() {
  // ---------- Time step ----------
  unsigned long now = millis();
  float dt = (now - last_ms) * 0.001f;
  if (dt <= 0) dt = 0.001f;
  last_ms = now;

  // ---------- Read IMU ----------
  
  sh2_SensorValue_t sensorValue;
  if (bno.getSensorEvent(&sensorValue)) {    
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      float ax = deadband(sensorValue.un.linearAcceleration.x, 0.05f);
      float ay = deadband(sensorValue.un.linearAcceleration.y, 0.05f);
      float az = deadband(sensorValue.un.linearAcceleration.z, 0.05f);
      Serial.print("Lin Accel X: ");
      Serial.print(ax);
      Serial.print("  Y: ");
      Serial.print(ay);
      Serial.print("  Z: ");
      Serial.println(az);

      // ---------- Build prediction (fx, F, Qscaled) ----------
      _float_t fx[EKF_N];
      _float_t F[EKF_N*EKF_N];
    
      ekf_build_fx_F(ekf.x, ax, ay, dt, fx, F);
    
      // Scale Q gently with dt (very simple scheme)
      _float_t Qscaled[EKF_N*EKF_N];
      memcpy(Qscaled, Q, sizeof(Qscaled));
      Qscaled[0*EKF_N+0] *= dt;
      Qscaled[1*EKF_N+1] *= dt;
      Qscaled[2*EKF_N+2] *= dt;
      Qscaled[3*EKF_N+3] *= dt;
    
      // ---------- Predict ----------
      ekf_predict(&ekf, fx, F, Qscaled);
    }
    
  }


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

    // z = [gps_x, gps_y]
    _float_t z[EKF_M];
    z[0] = gps_x_m;
    z[1] = gps_y_m;

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
  Serial.print("  vy: "); Serial.println(ekf.x[3], 3);

  delay(20); // ~50 Hz loop
}
