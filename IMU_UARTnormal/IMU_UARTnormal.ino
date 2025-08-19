#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno;

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
float rad2deg(float r)
{ 
  return r * 57.2957795f; 
}

bool firstHeading = true;
float headingBias = 0.0f;

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

  bno.enableReport(SH2_ROTATION_VECTOR);            // 9DoF fused
}

void loop() {
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

      Serial.print("Heading: "); Serial.println(heading_deg, 1);
      
    }
    
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      
      Serial.print("Lin Accel X: ");
      Serial.print(deadband(sensorValue.un.linearAcceleration.x, 0.05f));
      Serial.print("  Y: ");
      Serial.print(deadband(sensorValue.un.linearAcceleration.y,0.05f));
      Serial.print("  Z: ");
      Serial.println(deadband(sensorValue.un.linearAcceleration.z,0.05f));

      float a_fwd = deadband(sensorValue.un.linearAcceleration.x, 0.05f);
      float ay_world = a_fwd * cosf(yaw);
      float ax_world = -a_fwd * sinf(yaw);
      
      Serial.print("  X World: ");
      Serial.print(ax_world);
      Serial.print("  Y World: ");
      Serial.println(ay_world);
      
    }
    
  }
}
