// Motor pins
#define DIR1   12
#define DIR2   14
#define PWM1   26
#define PWM2   27

// PWM setup
#define FREQ       18000
#define RESOLUTION 8  // 8-bit = 0..255
#define CH1        0
#define CH2        1

float steering;
int16_t power; // use int16_t if signed

// ======= FailSafe ========
uint32_t lastCommandTime = 0;
const uint32_t COMMAND_TIMEOUT = 500;

// ===== Motor control =====
void motorStop(){
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);
}

void motorForward(uint16_t torq){
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  ledcWrite(CH1, (2*torq));
  ledcWrite(CH2, (2*torq));
}

void motorBack(uint16_t torq){
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  ledcWrite(CH1, (2*torq));
  ledcWrite(CH2, (2*torq));
}

void motorLeft(uint16_t torq){
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  ledcWrite(CH1, (1*torq));
  ledcWrite(CH2, (1*torq));
}

void motorRight(uint16_t torq){
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  ledcWrite(CH1, (1*torq));
  ledcWrite(CH2, (1*torq));
}

void setup() {
    Serial.begin(115200);
    
    pinMode(DIR1, OUTPUT);
    pinMode(DIR2, OUTPUT);
  
    ledcSetup(CH1, FREQ, RESOLUTION);
    ledcAttachPin(PWM1, CH1);
    ledcSetup(CH2, FREQ, RESOLUTION);
    ledcAttachPin(PWM2, CH2);
  
    motorStop();
}

void loop() {
    if (Serial.available() >= 6) { // 4 bytes float + 2 bytes short
        Serial.readBytes((char*)&steering, sizeof(steering));
        Serial.readBytes((char*)&power, sizeof(power));

        Serial.print("Steering: ");
        Serial.print(steering, 1); // print 3 decimal places
        Serial.print("Power: ");
        Serial.println(power);
        lastCommandTime = millis();
    }

    if (power > 0) {
      if (steering < -40) {
        motorRight(abs(power));
      }
      else if (steering > 40) {
        motorLeft(abs(power));
      }
      else {
        motorForward(abs(power));
      }
    }
    else if (power < 0) {
      motorBack(abs(power));
    }
    else {
      motorStop();
    }

    if ((millis() - lastCommandTime) > COMMAND_TIMEOUT) {
      motorStop();
    }

}
