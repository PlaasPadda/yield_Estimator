
// Motor pins
#define DIR1  18
#define DIR2   5
#define PWM1   14
#define PWM2   27

// PWM setup
#define FREQ       18000
#define RESOLUTION 8  // 8-bit = 0..255
#define CH1        0
#define CH2        1

// ===== Motor control =====
void motorStop(){
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);
}

void motorForward(){
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  ledcWrite(CH1, 200);
  ledcWrite(CH2, 200);
}

void motorBack(){
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  ledcWrite(CH1, 200);
  ledcWrite(CH2, 200);
}

void motorLeft(){
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  ledcWrite(CH1, 100);
  ledcWrite(CH2, 100);
}

void motorRight(){
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  ledcWrite(CH1, 100);
  ledcWrite(CH2, 100);
}

void setup(){
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  ledcSetup(CH1, FREQ, RESOLUTION);
  ledcAttachPin(PWM1, CH1);
  ledcSetup(CH2, FREQ, RESOLUTION);
  ledcAttachPin(PWM2, CH2);

  motorStop();
}

void loop(){ 
  // failsafe: stop if no command within timeout
  if(millis() - lastCmdTime > CMD_TIMEOUT){
    motorStop();
  }
}
