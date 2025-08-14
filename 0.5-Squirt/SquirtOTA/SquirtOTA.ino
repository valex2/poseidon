// Squirt Homebase — Multi-line mission capable, status UI, last 5 logs
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <queue>
#include <vector>

// ======= Reed Based Kill Switch =======
constexpr int LED_PIN = 2; // kill switch indicator
bool isKilled = false;
unsigned long lastBlink = 0;
bool ledState = false;

// ======= Command Execution =======
struct Cmd {
  String key;
  int value;      // % for thrusters or ° for servos
  int duration;   // seconds
};

std::queue<String> rawLines; 
bool executingQueue = false;

// ======== AP credentials =========
const char *AP_SSID = "ESP32-Squirt";
const char *AP_PASS = "squirtSwims!";

// ------- Pin assignments -------
constexpr uint8_t THRUSTER1_PIN = 25;
constexpr uint8_t THRUSTER2_PIN = 26;
constexpr uint8_t SERVO1_PIN    = 32;
constexpr uint8_t SERVO2_PIN    = 33;

// ------- Servo/ESC timing ------
constexpr uint32_t PWM_FREQ_HZ = 50;
constexpr int THR_MIN_US = 1100;
constexpr int THR_NEU_US = 1500;
constexpr int THR_MAX_US = 1900;
constexpr int SERVO_MIN_US = 1000;
constexpr int SERVO_NEU_US = 1500;
constexpr int SERVO_MAX_US = 2000;

// ------- Networking ----------
WebServer http(80);
WebSocketsServer ws(81);

String logBuffer[5]; // store last 5 logs

// ===== HTML UI =====
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>ESP32 Mission Control</title>
<style>
  body { font-family: system-ui, sans-serif; margin: 20px; max-width: 700px; }
  textarea { width: 100%; height: 150px; font-family: monospace; margin-top: 10px; }
  pre { background: #eee; padding: 8px; border-radius: 8px; height: 120px; overflow-y: auto; font-size: 12px; }
  .card { border: 1px solid #ddd; border-radius: 12px; padding: 16px; margin: 12px 0; box-shadow: 0 1px 6px rgba(0,0,0,.06); }
  .row { display: flex; gap: 12px; align-items: center; margin: 8px 0; }
  .status { font-weight: bold; padding: 4px 8px; border-radius: 6px; }
  .ready { background: #0a0; color: white; }
  .running { background: #a00; color: white; }
  button { padding: 8px 12px; border-radius: 8px; border: 1px solid #aaa; background: #f5f5f5; }
</style>
</head>
<body>
<h1>ESP32 Mission Control</h1>
<div>Status: <span id=status class="status ready">READY</span></div>
<div id=conn>WebSocket: connecting...</div>

<div class=card>
  <h3>Command Sequence</h3>
  <textarea id=cmdseq placeholder="Paste mission here\nExample:\nT1:50:3;T2:50:3\nS1:20:2;S2:-20:2\nWAIT:2"></textarea>
  <button id=runSeq>Run Mission</button>
</div>

<div class=card>
  <button id=kill>Kill (safe stop)</button>
  <button id=resetKill style="margin-left:10px">Reset Kill</button>
</div>

<div class=card>
  <h3>Last 5 Logs</h3>
  <pre id=log></pre>
</div>

<script>
const host = location.hostname || "192.168.4.1";
let sock, alive = false;
function send(cmd){ if(sock && alive){ sock.send(cmd); } }
function connectWS(){
  sock = new WebSocket(`ws://${host}:81/`);
  sock.onopen = () => { alive = true; conn.textContent = "WebSocket: connected"; };
  sock.onclose = () => { alive = false; conn.textContent = "WebSocket: disconnected"; setTimeout(connectWS, 1000); };
  sock.onmessage = (e) => {
    if(e.data.startsWith("[STATUS]")){
      const state = e.data.split(" ")[1];
      const statusEl = document.getElementById("status");
      if(state === "RUNNING"){ statusEl.textContent = "RUNNING"; statusEl.className = "status running"; }
      else { statusEl.textContent = "READY"; statusEl.className = "status ready"; }
    } else {
      const logEl = document.getElementById("log");
      logEl.textContent += e.data + "\\n";
      const lines = logEl.textContent.trim().split("\\n");
      if(lines.length > 5) logEl.textContent = lines.slice(-5).join("\\n") + "\\n";
      logEl.scrollTop = logEl.scrollHeight;
    }
  };
}
connectWS();
runSeq.onclick = () => {
  const lines = cmdseq.value.trim().split("\\n");
  for(const line of lines) send(line);
};
kill.onclick = () => send("KILL");
resetKill.onclick = () => send("RESETKILL");
</script>
</body>
</html>
)HTML";

// ===== Utility =====
void addLog(String msg) {
  for (int i = 0; i < 4; i++) logBuffer[i] = logBuffer[i+1];
  logBuffer[4] = msg;
  String out;
  for (auto &l : logBuffer) if (l.length()) out += l + "\n";
  ws.broadcastTXT(out);
}
int servoAngleToUs(int angleDeg) {
  angleDeg = constrain(angleDeg, -90, 90);
  return map(angleDeg, -90, 90, SERVO_MIN_US, SERVO_MAX_US);
}
int thrusterPctToUs(int pct) {
  pct = constrain(pct, -100, 100);
  if (pct >= 0)  return map(pct, 0, 100, THR_NEU_US, THR_MAX_US);
  else           return map(pct, -100, 0, THR_MIN_US, THR_NEU_US);
}
void setThruster1(int pct) {
  int us = thrusterPctToUs(pct);
  uint32_t duty = (us * 65536L) / 20000;
  ledcWrite(THRUSTER1_PIN, duty);
  addLog("T1 -> " + String(pct) + "%");
}
void setThruster2(int pct) {
  int us = thrusterPctToUs(pct);
  uint32_t duty = (us * 65536L) / 20000;
  ledcWrite(THRUSTER2_PIN, duty);
  addLog("T2 -> " + String(pct) + "%");
}
void setServo1(int ang) {
  int us = servoAngleToUs(ang);
  uint32_t duty = (us * 65536L) / 20000;
  ledcWrite(SERVO1_PIN, duty);
  addLog("S1 -> " + String(ang) + "°");
}
void setServo2(int ang) {
  int us = servoAngleToUs(ang);
  uint32_t duty = (us * 65536L) / 20000;
  ledcWrite(SERVO2_PIN, duty);
  addLog("S2 -> " + String(ang) + "°");
}
void safeStop() {
  uint32_t thruster_neutral = (THR_NEU_US * 65536L) / 20000;
  uint32_t servo_neutral = (SERVO_NEU_US * 65536L) / 20000;
  ledcWrite(THRUSTER1_PIN, thruster_neutral);
  ledcWrite(THRUSTER2_PIN, thruster_neutral);
  ledcWrite(SERVO1_PIN, servo_neutral);
  ledcWrite(SERVO2_PIN, servo_neutral);
  addLog("SAFE STOP: all outputs neutral");
  isKilled = true;
}
Cmd parseCommand(const String &part) {
  Cmd cmd = {"", 0, 0};
  int firstColon = part.indexOf(':');
  if (firstColon < 0) return cmd;
  cmd.key = part.substring(0, firstColon);
  String rest = part.substring(firstColon + 1);
  int secondColon = rest.indexOf(':');
  if (secondColon != -1) {
    cmd.value = rest.substring(0, secondColon).toInt();
    cmd.duration = rest.substring(secondColon + 1).toInt();
  } else {
    cmd.value = rest.toInt();
    cmd.duration = 0;
  }
  return cmd;
}
void executeQueue() {
  executingQueue = true;
  ws.broadcastTXT("[STATUS] RUNNING");
  while (!rawLines.empty()) {
    String line = rawLines.front(); rawLines.pop();
    line.trim(); if (!line.length()) continue;
    std::vector<Cmd> cmds;
    int maxDur = 0;
    int start = 0;
    while (start < line.length()) {
      int sep = line.indexOf(';', start);
      if (sep == -1) sep = line.length();
      String part = line.substring(start, sep);
      part.trim();
      if (part.length() > 0) {
        Cmd c = parseCommand(part);
        if (c.key == "WAIT") {
          addLog("WAIT " + String(c.value) + "s");
          delay(c.value * 1000);
        } else {
          cmds.push_back(c);
          if (c.duration > maxDur) maxDur = c.duration;
        }
      }
      start = sep + 1;
    }
    for (auto &c : cmds) {
      addLog("CMD " + c.key + ":" + String(c.value) + 
             (c.duration ? " for " + String(c.duration) + "s" : ""));
      if      (c.key == "T1") setThruster1(c.value);
      else if (c.key == "T2") setThruster2(c.value);
      else if (c.key == "S1") setServo1(c.value);
      else if (c.key == "S2") setServo2(c.value);
    }
    if (maxDur > 0) {
      delay(maxDur * 1000);
      for (auto &c : cmds) {
        if      (c.key == "T1") setThruster1(0);
        else if (c.key == "T2") setThruster2(0);
        else if (c.key == "S1") setServo1(0);
        else if (c.key == "S2") setServo2(0);
      }
      addLog("→ Reset to neutral after " + String(maxDur) + "s");
    }
  }
  ws.broadcastTXT("[STATUS] READY");
  executingQueue = false;
}
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  if (type == WStype_TEXT) {
    String msg((char*)payload, len);
    msg.trim();
    if (msg == "KILL") { safeStop(); while (!rawLines.empty()) rawLines.pop(); return; }
    if (msg == "RESETKILL") { isKilled = false; digitalWrite(LED_PIN, LOW); addLog("KILL Reset via UI"); return; }
    rawLines.push(msg);
    if (!executingQueue) executeQueue();
  }
}
void setupHttp() {
  http.on("/", []() { http.send_P(200, "text/html", INDEX_HTML); });
  http.begin();
}
void setupPwm() {
  ledcAttach(THRUSTER1_PIN, PWM_FREQ_HZ, 16);
  ledcAttach(THRUSTER2_PIN, PWM_FREQ_HZ, 16);
  ledcAttach(SERVO1_PIN, PWM_FREQ_HZ, 16);
  ledcAttach(SERVO2_PIN, PWM_FREQ_HZ, 16);
  safeStop();
}
void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
}
void setup() {
  Serial.begin(115200);
  setupPwm();
  startAP();
  setupHttp();
  ws.begin();
  ws.onEvent(onWsEvent);
  pinMode(LED_PIN, OUTPUT);
  pinMode(19, INPUT_PULLDOWN);
}
void loop() {
  http.handleClient();
  ws.loop();
  if (digitalRead(19) == HIGH && !isKilled) {
    addLog("HARDWARE KILL triggered");
    safeStop();
    while (!rawLines.empty()) rawLines.pop();
    executingQueue = false;
  }
  if (isKilled) {
    unsigned long now = millis();
    if (now - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = now;
    }
  }
}