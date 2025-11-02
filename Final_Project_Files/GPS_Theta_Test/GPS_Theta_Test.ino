#include <Wire.h>
#include <Adafruit_GPS.h>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

#define GPSECHO false
bool   have_origin = false;
float x_bias = 0, y_bias = 0;
float theta_angle = 0, theta_bias = 0;
uint16_t counter = 0;

uint32_t timer = millis();
double lat0_deg = 0.0, lon0_deg = 0.0;
double lat0_rad = 0.0, cos_lat0 = 1.0;

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

void setup()
{
  Serial.begin(115200);
  Serial.println("Adafruit I2C GPS library basic test!");
  Wire.begin(25, 33); // SDA, SCL

  GPS.begin(0x10);  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  GPS.sendCommand(PGCMD_ANTENNA);
  delay(100);

  GPS.sendCommand("$PMTK386,0*23");           //Turn off static Nav

  delay(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);
}

void loop() // run over and over again
{
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
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
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 10); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 10); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
  
      float gps_x_m = 0, gps_y_m = 0;
      ll_to_local_m(GPS.latitudeDegrees, GPS.longitudeDegrees, gps_x_m, gps_y_m);
      
      if (!have_origin) {
        x_bias = gps_x_m;
        y_bias = gps_y_m;
        have_origin = true;
      }

      gps_x_m = gps_x_m - x_bias;
      gps_y_m = gps_y_m - y_bias;
      
      Serial.print("X pos: "); Serial.println(gps_x_m,7);
      Serial.print("Y pos: "); Serial.println(gps_y_m,7);

      getTheta(gps_x_m, gps_y_m, &theta_angle);
      float skuins = sqrtf(gps_x_m*gps_x_m + gps_y_m*gps_y_m);
      gps_x_m = skuins*sinf(theta_angle-theta_bias);
      gps_y_m = skuins*cosf(theta_angle-theta_bias);

      Serial.print("X pos corrected: "); Serial.println(gps_x_m,7);
      Serial.print("Y pos corrected: "); Serial.println(gps_y_m,7);
      counter++;

      if (counter >= 15) {
        getTheta(gps_x_m, gps_y_m, &theta_bias);
        Serial.print("-------------Bias angle--------------: "); Serial.println(theta_bias);
        counter = -100;
      }
     
    }
  }
}
