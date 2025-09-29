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

// ================ Heading Helpers =================
float rad2deg(float r)
{ 
  return r * 57.2957795f; 
}

bool firstHeading = true;
float headingBias = 0.0f;

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
float S_x, S_y;
_float_t R[EKF_M*EKF_M] = {0};  // measurement noise

void measureAccelNoiseDensity(HardwareSerial &dbgSerial, float* S_a_x, float* S_a_y, uint16_t N = 2000, float sample_dt = 0.01f) {
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

  *S_a_x = n_x * n_x; // m^2 / s^3
  *S_a_y = n_y * n_y;
  float S_a_z = n_z * n_z;

  dbgSerial.print("fs: "); dbgSerial.println(fs);
  dbgSerial.print("sigma_x (m/s^2): "); dbgSerial.println(sigma_x, 6);
  dbgSerial.print("sigma_y (m/s^2): "); dbgSerial.println(sigma_y, 6);
  dbgSerial.print("sigma_z (m/s^2): "); dbgSerial.println(sigma_z, 6);

  dbgSerial.print("noise density x (m/s/√Hz): "); dbgSerial.println(n_x, 9);
  dbgSerial.print("noise density y (m/s/√Hz): "); dbgSerial.println(n_y, 9);
  dbgSerial.print("noise density z (m/s/√Hz): "); dbgSerial.println(n_z, 9);

  dbgSerial.print("Power Spectral Density x (m^2/s^3): "); dbgSerial.println(*S_a_x, 9);
  dbgSerial.print("Power Spectral Density y (m^2/s^3): "); dbgSerial.println(*S_a_y, 9);
  dbgSerial.print("Power Spectral Density z (m^2/s^3): "); dbgSerial.println(S_a_z, 9);
}

// ================== Timing ==================
unsigned long last_ms = 0;

// ================== GPS origin for local frame ==================
bool   have_origin = false;
double lat0_deg = 0.0, lon0_deg = 0.0;
double lat0_rad = 0.0, cos_lat0 = 1.0;
float last_gps_x = 0;
float last_gps_y = 0;
const float gps_threshold = 0; // meters
float theta_angle = 0, theta_bias = 1.35;
float x_bias = 0, y_bias = 0;

// quick and decent local meters conversion (ENU-ish)
static void ll_to_local_m(double lat_deg, double lon_deg, float &x_m, float &y_m) {
  // constants (good enough for small areas)
  const double m_per_deg_lat = 110540.0;   // meters/deg latitude
  const double m_per_deg_lon = 111320.0 * cos_lat0; // meters/deg longitude at origin

  x_m = (float)((lon_deg - lon0_deg) * m_per_deg_lon);
  y_m = (float)((lat_deg - lat0_deg) * m_per_deg_lat);
}

void getTheta(float x_m, float y_m, float* theta) {
  float skuins = sqrtf(x_m*x_m + y_m*y_m);
  *theta = asinf(x_m/skuins);
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
  fx[0] = x[0] + x[2]*dt + ax_w*dt*dt;
  fx[1] = x[1] + x[3]*dt + ay_w*dt*dt;
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
uint16_t ack = 0;
void printStateStdDevs() {
  //Serial.println("State 1-sigma (sqrt of diag(P)):");
  for (int i = 0; i < EKF_N; ++i) {
    float sigma = sqrtf(ekf.P[i*EKF_N + i]);
    //Serial.print("x["); Serial.print(i); Serial.print("] σ = ");
    
  while (!Serial.available() >= 2) {
    // Block until something arrives
  }
  
      Serial.readBytes((char*)&ack, sizeof(ack));
      if (ack == 1) {
        Serial.println(sigma, 6);
        ack = 0;
        delay(2);
      }
  }
}

void setup() {
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
  
  // ---- GPS (I2C) ----
  Wire.begin(GPS_SDA, GPS_SCL);
  GPS.begin(0x10);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 10 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(100);
  GPS.sendCommand("$PMTK386,0*23");           //Turn off static Nav
  delay(1000);
  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);

  // ---- EKF init ----
  const _float_t Pdiag[EKF_N] = {
    1.0f,  // x  (m^2)
    1.0f,  // y
    1.0f,  // vx ((m/s)^2)
    1.0f,  // vy
  };
  ekf_initialize(&ekf, Pdiag);

  // Power Spectral density
  measureAccelNoiseDensity(Serial, &S_x, &S_y, 2000, 0.01f);
  S_x = 1000*S_x;
  S_y = 1000*S_y;

  // Measurement noise R
  memset(R, 0, sizeof(R));
  float std_dev = 3 / 1.177;  // std = CEP/1.177
  R[0*EKF_M+0] = std_dev * std_dev;    // gps x variance (m^2)  
  R[1*EKF_M+1] = std_dev * std_dev;    // gps y variance (m^2)
  
  last_ms = millis();
  Serial.println("TinyEKF ready");

  uint16_t asscount = 0;
  while (GPS.available()) {
    GPS.read();
  
    asscount++;
    if (asscount>=2000) {
      break;
    }
  }
}

void loop() {
  // ---------- Time step ----------
  unsigned long now = millis();
  float dt = (now - last_ms) * 0.001f;
  if (dt <= 0) dt = 0.001f;
  last_ms = now;


  // ---------- Read IMU ----------
  
  sh2_SensorValue_t sensorValue;
  float yaw;
  if (bno.getSensorEvent(&sensorValue)) {    
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {

      // Quaternion (w,x,y,z)
      float w = sensorValue.un.rotationVector.real;
      float x = sensorValue.un.rotationVector.i;
      float y = sensorValue.un.rotationVector.j;
      float z = sensorValue.un.rotationVector.k;

      // Yaw (Z heading), aerospace sequence
      yaw = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));

      if (firstHeading)
      {
        headingBias = yaw;
        firstHeading = false;
      }  

      // Wrap to 0..360°
      yaw -= headingBias;
      float heading_deg = fmodf(rad2deg(yaw) + 360.0f, 360.0f);

      //Serial.print("Heading: "); Serial.println(heading_deg, 1);
      //delay(1);   //DO NOT REMOVE DELAY, OTHERWISE X DOES NOT PRINT
      
    }
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      float ax = deadband(sensorValue.un.linearAcceleration.x, 0.05f);
      float ay = deadband(sensorValue.un.linearAcceleration.y, 0.05f);
      float az = deadband(sensorValue.un.linearAcceleration.z, 0.05f);
//      Serial.print("Lin Accel X: ");
//      Serial.print(ax);
//      Serial.print("  Y: ");
//      Serial.print(ay);
//      Serial.print("  Z: ");
//      Serial.println(az);
      
      float ay_world = ax * cosf(yaw);
      float ax_world = -ax * sinf(yaw);
      
//      Serial.print("  X World: ");
//      Serial.print(ax_world);
//      Serial.print("  Y World: ");
//      Serial.println(ay_world);
      

      // ---------- Build prediction (fx, F, Qscaled) ----------
      _float_t fx[EKF_N];
      _float_t F[EKF_N*EKF_N];
    
      ekf_build_fx_F(ekf.x, ax_world, ay_world, dt, fx, F);
    
      // Scale Q with dt
      _float_t Qscaled[EKF_N*EKF_N];
      memset(Qscaled, 0, sizeof(Qscaled));

      float dt2 = dt*dt;
      float dt3 = dt2*dt;
      float dt4 = dt2*dt2;
      
      // 1D blocks
      float Q11x = S_x * (dt4 * 0.25f);
      float Q12x = S_x * (dt3 * 0.5f);
      float Q22x = S_x * dt2;
      
      float Q11y = S_y * (dt4 * 0.25f);
      float Q12y = S_y * (dt3 * 0.5f);
      float Q22y = S_y * dt2;

      // X block
      Qscaled[0*EKF_N + 0] = Q11x;
      Qscaled[0*EKF_N + 2] = Q12x;
      Qscaled[2*EKF_N + 0] = Q12x;
      Qscaled[2*EKF_N + 2] = Q22x;
      
      // Y block
      Qscaled[1*EKF_N + 1] = Q11y;
      Qscaled[1*EKF_N + 3] = Q12y;
      Qscaled[3*EKF_N + 1] = Q12y;
      Qscaled[3*EKF_N + 3] = Q22y;
    
      // ---------- Predict ----------
      ekf_predict(&ekf, fx, F, Qscaled);
    }
    
  }


  // ---------- GPS parsing ----------
  // keep feeding the parser

// read data from the GPS in the 'main loop'
  uint16_t asscount = 0;
  while (GPS.available()) {
    char c = GPS.read();
  
    asscount++;
    if (asscount>=50) {
      break;
    }
  }

  bool have_new_gps = false;
  if (GPS.newNMEAreceived()) {
    have_new_gps = true;
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
      
    //Serial.print("GPS UTC seconds: ");
    //Serial.println(GPS.seconds);
    //Serial.print("Millis: ");
    //Serial.println(millis());
  }

  // ---------- Build measurement when we have GPS fix ----------
  
  if (GPS.fix && have_new_gps) {
  
    float gps_x_m = 0, gps_y_m = 0;
    ll_to_local_m(GPS.latitudeDegrees, GPS.longitudeDegrees, gps_x_m, gps_y_m);
    
    if (!have_origin) {
      x_bias = gps_x_m;
      y_bias = gps_y_m;
      have_origin = true;
    }
  
    gps_x_m = gps_x_m - x_bias;
    gps_y_m = gps_y_m - y_bias;
  
    getTheta(gps_x_m, gps_y_m, &theta_angle);
    float skuins = sqrtf(gps_x_m*gps_x_m + gps_y_m*gps_y_m);
    gps_x_m = skuins*sinf(theta_angle-theta_bias);
    gps_y_m = skuins*cosf(theta_angle-theta_bias);

    // z = [gps_x, gps_y]
    _float_t z[EKF_M];
    if ((gps_x_m == gps_x_m) && (gps_y_m == gps_y_m)) {
      z[0] = gps_x_m;
      z[1] = gps_y_m;
  
      //Serial.print("-------------------gps_x_m:  "); Serial.print(gps_x_m,7);
      //Serial.print("-------------------gps_y_m:  "); Serial.println(gps_y_m,7);
  
      // h(x) and H
      _float_t hx[EKF_N];     // NOTE: tinyekf uses EKF_N for hx buffer size
      _float_t H[EKF_M*EKF_N];
      ekf_build_h_H(ekf.x, hx, H);
  
      // ---------- Update ----------
      (void)ekf_update(&ekf, z, hx, H, R);
    }
  }



  // ---------- Debug ----------
  while (!Serial.available() >= 2) {
    // Block until something arrives
  }
  
    Serial.readBytes((char*)&ack, sizeof(ack));
    if (ack == 1) {
      //Serial.print("x: ");  
      Serial.println(ekf.x[0], 3);
      ack = 0;
      delay(2);
    }

  while (!Serial.available() >= 2) {
    // Block until something arrives
  }
  
    Serial.readBytes((char*)&ack, sizeof(ack));
    if (ack == 1) {
      //Serial.print("y: ");  
      Serial.println(ekf.x[1], 3);
      ack = 0;
      delay(2);
    }
  
  while (!Serial.available() >= 2) {
    // Block until something arrives
  }
  
    Serial.readBytes((char*)&ack, sizeof(ack));
    if (ack == 1) {
      //Serial.print("  vx: ");  
      Serial.println(ekf.x[2], 3);
      ack = 0;
      delay(2);
    }

  while (!Serial.available() >= 2) {
    // Block until something arrives
  }
  
    Serial.readBytes((char*)&ack, sizeof(ack));
    if (ack == 1) {
      //Serial.print("  vy: ");  
      Serial.println(ekf.x[3], 3);
      ack = 0;
      delay(2);
    }
  printStateStdDevs();

  //delay(20); // ~50 Hz loop
}
