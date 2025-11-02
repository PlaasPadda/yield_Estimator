// Test code for Adafruit GPS That Support Using I2C
//
// This code shows how to parse data from the I2C GPS
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Wire.h>
#include <Adafruit_GPS.h>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
bool   have_origin = false;
float x_bias = 0, y_bias = 0;
float theta_angle = 0, theta_bias = -0.6;

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
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit I2C GPS library basic test!");
  Wire.begin(25, 33); // SDA, SCL

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(0x10);  // The I2C address to use is 0x10
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(100);

  GPS.sendCommand("$PMTK386,0*23");           //Turn off static Nav

  delay(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
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

      getTheta(gps_x_m, gps_y_m, &theta_angle);
      float skuins = sqrtf(gps_x_m*gps_x_m + gps_y_m*gps_y_m);
      gps_x_m = skuins*sinf(theta_angle-theta_bias);
      gps_y_m = skuins*cosf(theta_angle-theta_bias);
      
      Serial.print("X pos: "); Serial.println(gps_x_m,7);
      Serial.print("Y pos: "); Serial.println(gps_y_m,7);
     
    }
  }
}
