// Squirt Homebase — Timed batch command version with full UI
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <queue>
#include <Ticker.h>
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
  int duration;   // in seconds (0 if none)
};

std::queue<String> rawLines; // store incoming lines to execute
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

// ===== HTML UI =====
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>ESP32 Thrusters & Servos</title>
<style>
  body { font-family: system-ui, sans-serif; margin: 20px; max-width: 700px; }
  textarea { width: 100%; height: 150px; font-family: monospace; margin-top: 10px; }
  pre { background: #eee; padding: 8px; border-radius: 8px; max-height: 200px; overflow-y: auto; font-size: 12px; }
  .card { border: 1px solid #ddd; border-radius: 12px; padding: 16px; margin: 12px 0; box-shadow: 0 1px 6px rgba(0,0,0,.06); }
  .row { display: flex; gap: 12px; align-items: center; margin: 8px 0; }
  input[type=range] { width: 100%; }
  .val { min-width: 56px; text-align: right; font-variant-numeric: tabular-nums; }
  .ok { color: #0a0; }
  .bad { color: #a00; }
  button { padding: 8px 12px; border-radius: 8px; border: 1px solid #aaa; background: #f5f5f5; }
</style>
</head>
<body>
<h1>ESP32 Live Control</h1>
<div id=conn class=bad>WebSocket: connecting...</div>

<div class=card>
  <h3>Thrusters (0 to 100%)</h3>
  <div class=row><label style="width:120px">Thruster 1</label>
    <input id=t1 type=range min=-100 max=100 step=1 value=0>
    <div class=val><span id=t1v>0</span>%</div>
  </div>
  <div class=row><label style="width:120px">Thruster 2</label>
    <input id=t2 type=range min=-100 max=100 step=1 value=0>
    <div class=val><span id=t2v>0</span>%</div>
  </div>
</div>

<div class=card>
  <h3>Servos (-90 to +90)</h3>
  <div class=row><label style="width:120px">Servo 1</label>
    <input id=s1 type=range min=-90 max=90 step=1 value=0>
    <div class=val><span id=s1v>0</span>°</div>
  </div>
  <div class=row><label style="width:120px">Servo 2</label>
    <input id=s2 type=range min=-90 max=90 step=1 value=0>
    <div class=val><span id=s2v>0</span>°</div>
  </div>
</div>

<div class=card>
  <button id=kill>Kill (safe stop)</button>
  <button id=resetKill style="margin-left:10px">Reset Kill</button>
</div>

<div class=card>
  <h3>Command Sequence</h3>
  <textarea id=cmdseq placeholder="Example:\nT1:50:5;T2:-50:5\nS1:30:3;S2:-30:3\nWAIT:2"></textarea>
  <button id=runSeq>Execute</button>
</div>

<div class=card>
  <h3>Live Logs <label style="font-weight:normal"><input id=logToggle type=checkbox checked> Show Logs</label></h3>
  <pre id="log"></pre>
</div>

<script>
const host = location.hostname || "192.168.4.1";
let sock, alive = false, showLogs = true;
function send(cmd){ if(sock && alive){ sock.send(cmd); } }
function connectWS(){
  sock = new WebSocket(`ws://${host}:81/`);
  sock.onopen = () => { alive = true; conn.textContent = "WebSocket: connected"; conn.className = 'ok'; };
  sock.onclose = () => { alive = false; conn.textContent = "WebSocket: disconnected"; conn.className = 'bad'; setTimeout(connectWS, 1000); };
  sock.onmessage = (e) => { if(showLogs){ log.textContent += e.data + "\\n"; log.scrollTop = log.scrollHeight; } };
}
connectWS();
function bindSlider(id, key){
  const el = document.getElementById(id);
  const vl = document.getElementById(id+"v");
  const update = () => { vl.textContent = el.value; send(`${key}:${el.value}`); };
  el.addEventListener('input', update);
  el.addEventListener('change', update);
}
bindSlider('t1', 'T1');
bindSlider('t2', 'T2');
bindSlider('s1', 'S1');
bindSlider('s2', 'S2');
kill.onclick = () => send("KILL");
resetKill.onclick = () => send("RESETKILL");
logToggle.onchange = (e) => showLogs = e.target.checked;
runSeq.onclick = () => {
  const lines = cmdseq.value.trim().split("\\n");
  for(const line of lines) send(line);
};
</script>
</body>
</html>
)HTML";

// ===== Utility mappers =====
int servoAngleToUs(int angleDeg) {
  angleDeg = constrain(angleDeg, -90, 90);
  return map(angleDeg, -90, 90, SERVO_MIN_US, SERVO_MAX_US);
}
int thrusterPctToUs(int pct) {
  pct = constrain(pct, -100, 100);
  if (pct >= 0)  return map(pct, 0, 100, THR_NEU_US, THR_MAX_US);
  else           return map(pct, -100, 0, THR_MIN_US, THR_NEU_US);
}

// ===== Apply outputs =====
void setThruster1(int pct) {
  int pulse_us = thrusterPctToUs(pct);
  uint32_t duty = (pulse_us * 65536L) / 20000;
  ledcWrite(THRUSTER1_PIN, duty);
  ws.broadcastTXT("T1: " + String(pct) + "%");
}
void setThruster2(int pct) {
  int pulse_us = thrusterPctToUs(pct);
  uint32_t duty = (pulse_us * 65536L) / 20000;
  ledcWrite(THRUSTER2_PIN, duty);
  ws.broadcastTXT("T2: " + String(pct) + "%");
}
void setServo1(int ang) {
  int us = servoAngleToUs(ang);
  uint32_t duty = (us * 65536L) / 20000;
  ledcWrite(SERVO1_PIN, duty);
  ws.broadcastTXT("S1: " + String(ang) + "°");
}
void setServo2(int ang) {
  int us = servoAngleToUs(ang);
  uint32_t duty = (us * 65536L) / 20000;
  ledcWrite(SERVO2_PIN, duty);
  ws.broadcastTXT("S2: " + String(ang) + "°");
}

void safeStop() {
  uint32_t thruster_neutral = (THR_NEU_US * 65536L) / 20000;
  uint32_t servo_neutral = (SERVO_NEU_US * 65536L) / 20000;
  ledcWrite(THRUSTER1_PIN, thruster_neutral);
  ledcWrite(THRUSTER2_PIN, thruster_neutral);
  ledcWrite(SERVO1_PIN, servo_neutral);
  ledcWrite(SERVO2_PIN, servo_neutral);
  ws.broadcastTXT("Safe stop: all outputs neutral");
  isKilled = true;
}

// ===== Parse ACT:VAL:DUR =====
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

// ===== Execute all lines =====
void executeQueue() {
  executingQueue = true;
  while (!rawLines.empty()) {
    String line = rawLines.front();
    rawLines.pop();
    line.trim();
    if (line.length() == 0) continue;

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
          ws.broadcastTXT("[SEQ] WAIT " + String(c.value) + "s");
          delay(c.value * 1000);
        } else {
          cmds.push_back(c);
          if (c.duration > maxDur) maxDur = c.duration;
        }
      }
      start = sep + 1;
    }

    for (auto &c : cmds) {
      ws.broadcastTXT("[CMD] " + c.key + ":" + String(c.value) +
                      (c.duration > 0 ? " for " + String(c.duration) + "s" : ""));
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
      ws.broadcastTXT("[SEQ] Duration " + String(maxDur) + "s ended → reset");
    }
  }
  ws.broadcastTXT("[SEQ] Finished command sequence");
  executingQueue = false;
}

// ===== WebSocket =====
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  if (type == WStype_TEXT) {
    String msg((char*)payload, len);
    msg.trim();
    if (msg == "KILL") {
      safeStop();
      while (!rawLines.empty()) rawLines.pop();
      return;
    }
    if (msg == "RESETKILL") {
      isKilled = false;
      digitalWrite(LED_PIN, LOW);
      ws.broadcastTXT("[KILL] Reset via UI");
      return;
    }
    rawLines.push(msg);
    if (!executingQueue) executeQueue();
  }
}

// ===== HTTP & PWM =====
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

// ===== AP =====
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
  pinMode(19, INPUT_PULLDOWN); // reed kill input
}

void loop() {
  http.handleClient();
  ws.loop();
  if (digitalRead(19) == HIGH && !isKilled) {
    ws.broadcastTXT("[KILL] Hardware kill triggered");
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