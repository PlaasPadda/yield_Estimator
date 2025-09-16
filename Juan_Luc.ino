#include <WiFi.h>
#include <WebServer.h>

// ======= CONFIG =======
const char* AP_SSID  = "ForestryBotCtrl";
const char* AP_PASS  = "esp32pass";   // min 8 chars
// ======================

WebServer server(80);

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

// Failsafe
unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT = 500; // ms

// ====== HTML UI ======
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<meta name=viewport content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>ESP32 Drive</title>
<style>
  html, body {
    -webkit-text-size-adjust: 100%;
  }
  body {
    font-family: system-ui;
    margin:20px;
    max-width:480px;
    text-align:center;
  }
  .pad {
    display: grid;
    grid-template-areas:
      ".   up    ."
      "left .  right"
      ".  down   .";
    gap: 15px;
    justify-content: center;
    align-items: center;
    margin-top: 40px;
  }
  #fwd   { grid-area: up; }
  #rev   { grid-area: down; }
  #left  { grid-area: left; }
  #right { grid-area: right; }
  button {
    font-size:36px; 
    padding:30px; 
    width:90px; 
    height:90px; 
    user-select:none; 
    -webkit-user-select:none; 
    -webkit-touch-callout:none;
  }
</style>
<h2>Drive Control</h2>
<div class="pad">
  <button id=fwd   onmousedown="start('fwd')"   onmouseup="stop()" 
          ontouchstart="start('fwd')" ontouchend="stop()">forward</button>
  <button id=rev   onmousedown="start('rev')"   onmouseup="stop()" 
          ontouchstart="start('rev')" ontouchend="stop()">back</button>
  <button id=left  onmousedown="start('left')"  onmouseup="stop()" 
          ontouchstart="start('left')" ontouchend="stop()">left</button>
  <button id=right onmousedown="start('right')" onmouseup="stop()" 
          ontouchstart="start('right')" ontouchend="stop()">right</button>
</div>
<script>
let timer=null;
function start(cmd){
  send(cmd); 
  timer=setInterval(()=>send(cmd),200);
}
function stop(){
  if(timer){clearInterval(timer); timer=null;}
  send('stop');
}
async function send(cmd){
  try{await fetch('/cmd?txt='+encodeURIComponent(cmd),{method:'POST'});}catch(e){}
}
window.addEventListener("blur", stop);
window.addEventListener("beforeunload", stop);
</script>
)HTML";


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

// ===== HTTP handlers =====
void handleRoot(){ server.send_P(200, "text/html", INDEX_HTML); }

void handleCmd(){
  if(!server.hasArg("txt")){
    server.send(400, "text/plain", "missing txt");
    return;
  }
  String cmd = server.arg("txt");

  // reset failsafe timer
  lastCmdTime = millis();

  // Always stop before new command
  motorStop();

  if(cmd=="fwd") motorRight();
  else if(cmd=="rev") motorLeft();
  else if(cmd=="left") motorBack();
  else if(cmd=="right") motorForward();
  // "stop" just leaves motors stopped

  server.send(200, "text/plain", "OK");
}

void notFound(){ server.send(404, "text/plain", "Not found"); }

void setup(){
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  ledcSetup(CH1, FREQ, RESOLUTION);
  ledcAttachPin(PWM1, CH1);
  ledcSetup(CH2, FREQ, RESOLUTION);
  ledcAttachPin(PWM2, CH2);

  motorStop();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_ANY, handleCmd);
  server.onNotFound(notFound);
  server.begin();
}

void loop(){ 
  server.handleClient(); 

  // failsafe: stop if no command within timeout
  if(millis() - lastCmdTime > CMD_TIMEOUT){
    motorStop();
  }
}
