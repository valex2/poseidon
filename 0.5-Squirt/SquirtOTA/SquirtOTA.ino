// Squirt Homebase — Multi-line mission, manual controls, status UI, last 5 logs
// (servo manual persist + fast reed kill + websocket protection + servo ramp)
// FIX: advance servo ramp during mission waits so S1/S2 actually move on timed commands
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <queue>
#include <vector>

constexpr int LED_PIN = 2;
constexpr int KILL_PIN = 19;  // reed GPIO

bool isKilled = false;
unsigned long lastBlink = 0;
bool ledState = false;

// ISR flag
volatile bool killRequested = false;

struct Cmd {
  String key;
  int value;
  int duration;
};

std::queue<String> rawLines;
bool executingQueue = false;

const char *AP_SSID = "ESP32-Squirt";
const char *AP_PASS = "squirtSwims!";

constexpr uint8_t THRUSTER1_PIN = 25;
constexpr uint8_t THRUSTER2_PIN = 26;
constexpr uint8_t SERVO1_PIN    = 32;
constexpr uint8_t SERVO2_PIN    = 33;

constexpr uint32_t PWM_FREQ_HZ = 50;
constexpr int THR_MIN_US = 1100;
constexpr int THR_NEU_US = 1500;
constexpr int THR_MAX_US = 1900;
constexpr int SERVO_MIN_US = 1000;
constexpr int SERVO_NEU_US = 1500;
constexpr int SERVO_MAX_US = 2000;

WebServer http(80);
WebSocketsServer ws(81);

String logBuffer[10];

// ===== persistent manual baselines for servos =====
int baselineS1Deg = 0;  // last manual S1 angle
int baselineS2Deg = 0;  // last manual S2 angle

// ===== servo ramping (power-friendly) =====
int currentS1Deg = 0;
int currentS2Deg = 0;
int targetS1Deg  = 0;
int targetS2Deg  = 0;
constexpr int SERVO_STEP_DEG = 1;          // degrees per tick
constexpr uint32_t SERVO_TICK_MS = 25;     // ramp tick period
uint32_t lastServoTick = 0;

// ===== websocket keepalive =====
uint32_t lastWsKeepalive = 0;
constexpr uint32_t WS_KEEPALIVE_MS = 30000;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
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
    input[type=range] { flex: 1; }
    .val { min-width: 56px; text-align: right; font-variant-numeric: tabular-nums; }
    .ok { color: #0a0; font-weight: bold; }
    .bad { color: #a00; font-weight: bold; }
    button { padding: 8px 12px; border-radius: 8px; border: 1px solid #aaa; background: #f5f5f5; }
  </style>
</head>
<body>
  <h1>ESP32 Live Control</h1>
  <div id=conn class=bad>WebSocket: connecting...</div>
  <div id=state class=bad>READY</div>

  <div class=card>
    <h3>Thrusters (-100 to 100%)</h3>
    <div class=row>
      <label style="width:120px">Thruster 1</label>
      <input id=t1 type=range min=-100 max=100 step=1 value=0>
      <div class=val><span id=t1v>0</span>%</div>
    </div>
    <div class=row>
      <label style="width:120px">Thruster 2</label>
      <input id=t2 type=range min=-100 max=100 step=1 value=0>
      <div class=val><span id=t2v>0</span>%</div>
    </div>
  </div>

  <div class=card>
    <h3>Servos (-90 to 90°)</h3>
    <div class=row>
      <label style="width:120px">Servo 1</label>
      <input id=s1 type=range min=-90 max=90 step=1 value=0>
      <div class=val><span id=s1v>0</span>°</div>
    </div>
    <div class=row>
      <label style="width:120px">Servo 2</label>
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

    function send(cmd) { if (sock && alive) sock.send(cmd); }

    function connectWS(){
      sock = new WebSocket(`ws://${host}:81/`);
      sock.onopen = () => {
        alive = true;
        document.getElementById('conn').textContent = "WebSocket: connected";
        document.getElementById('conn').className = 'ok';
      };
      sock.onclose = () => {
        alive = false;
        document.getElementById('conn').textContent = "WebSocket: disconnected";
        document.getElementById('conn').className = 'bad';
        setTimeout(connectWS, 1000);
      };
      sock.onerror = () => { alive = false; };
      sock.onmessage = (e) => {
        if (!showLogs) return;
        const log = document.getElementById("log");
        log.textContent += e.data + "\\n";
        if (log.textContent.length > 5000) log.textContent = log.textContent.slice(-4000);
        log.scrollTop = log.scrollHeight;
      };
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

    document.getElementById('kill').onclick = () => { send("KILL"); };
    document.getElementById('resetKill').onclick = () => { send("RESETKILL"); };
    document.getElementById('logToggle').onchange = (e) => { showLogs = e.target.checked; };

    document.getElementById('runSeq').onclick = () => {
      const lines = document.getElementById('cmdseq').value.trim();
      if (lines) send(lines); // Send entire mission as one message
    };
  </script>
</body>
</html>
)rawliteral";

// ===== Utility / PWM helpers =====
void addLog(String msg) {
  for (int i = 0; i < 9; i++) logBuffer[i] = logBuffer[i+1];
  logBuffer[9] = msg;
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

// ===== low-level writers =====
inline void writeThrusterPct(uint8_t pin, int pct) {
  uint32_t duty = (thrusterPctToUs(pct) * 65536L) / 20000;
  ledcWrite(pin, duty);
}
inline void writeServoDegImmediate(uint8_t pin, int ang) {
  uint32_t duty = (servoAngleToUs(ang) * 65536L) / 20000;
  ledcWrite(pin, duty);
}

// Forward-declare ramp tick so waits can drive it:
void updateServoRamps();

// Public actuator APIs (thrusters immediate; servos now set targets)
void setThruster1(int pct) { writeThrusterPct(THRUSTER1_PIN, pct); addLog("T1 -> " + String(pct) + "%"); }
void setThruster2(int pct) { writeThrusterPct(THRUSTER2_PIN, pct); addLog("T2 -> " + String(pct) + "%"); }

void setServo1(int ang) {  // set target (ramped)
  targetS1Deg = constrain(ang, -90, 90);
  addLog("S1 -> " + String(targetS1Deg) + "° (ramp)");
}
void setServo2(int ang) {  // set target (ramped)
  targetS2Deg = constrain(ang, -90, 90);
  addLog("S2 -> " + String(targetS2Deg) + "° (ramp)");
}

// ===== fast neutral for safety stop =====
void safeStop() {
  uint32_t thruster_neutral = (THR_NEU_US * 65536L) / 20000;
  uint32_t servo_neutral    = (SERVO_NEU_US * 65536L) / 20000;
  ledcWrite(THRUSTER1_PIN, thruster_neutral);
  ledcWrite(THRUSTER2_PIN, thruster_neutral);
  ledcWrite(SERVO1_PIN, servo_neutral);
  ledcWrite(SERVO2_PIN, servo_neutral);

  // Also snap ramp state to neutral to avoid fighting
  currentS1Deg = targetS1Deg = 0;
  currentS2Deg = targetS2Deg = 0;

  addLog("SAFE STOP: all outputs neutral");
  isKilled = true;
}

// ===== KILL: ISR + helpers =====
void IRAM_ATTR onKillISR() {
  killRequested = true;  // set flag immediately in interrupt context
}

void processKillIfNeeded() {
  if (killRequested) {
    killRequested = false;     // consume the flag
    addLog("[KILL] Hardware kill triggered");
    safeStop();
    while (!rawLines.empty()) rawLines.pop();
    executingQueue = false;
    ws.broadcastTXT("[STATUS] READY");
  }
}

// Small-slice wait that aborts quickly on kill and services stack
// Returns true if kill was processed during the wait (caller should abort)
bool waitMsRespectKill(uint32_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms) {
    if (killRequested) { processKillIfNeeded(); return true; }
    http.handleClient();   // keep HTTP alive
    ws.loop();             // keep WS alive
    updateServoRamps();    // <<< FIX: advance servo ramp while waiting
    delay(5);              // yield often
  }
  return false;
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

// ===== Mission executor (with servo baseline persistence + fast kill) =====
void executeQueue() {
  executingQueue = true;
  ws.broadcastTXT("[STATUS] RUNNING");

  while (!rawLines.empty()) {
    if (killRequested) { processKillIfNeeded(); if (isKilled) break; }

    String line = rawLines.front(); rawLines.pop();
    line.trim(); if (!line.length()) continue;

    std::vector<Cmd> cmds;
    int maxDur = 0;

    // Split a line by ';'
    int start = 0;
    while (start < line.length()) {
      if (killRequested) { processKillIfNeeded(); if (isKilled) break; }
      int sep = line.indexOf(';', start);
      if (sep == -1) sep = line.length();
      String part = line.substring(start, sep);
      part.trim();

      if (part.length() > 0) {
        Cmd c = parseCommand(part);
        if (c.key == "WAIT") {
          addLog("WAIT " + String(c.value) + "s");
          if (waitMsRespectKill((uint32_t)c.value * 1000)) { if (isKilled) break; }
        } else {
          cmds.push_back(c);
          if (c.duration > maxDur) maxDur = c.duration;
        }
      }
      start = sep + 1;
    }
    if (isKilled) break;

    // Execute batch immediately
    for (auto &c : cmds) {
      if (killRequested) { processKillIfNeeded(); break; }
      addLog("CMD " + c.key + ":" + String(c.value) + (c.duration ? " for " + String(c.duration) + "s" : ""));

      if      (c.key == "T1") setThruster1(c.value);
      else if (c.key == "T2") setThruster2(c.value);
      else if (c.key == "S1") {
        setServo1(c.value);
        if (c.duration == 0) baselineS1Deg = c.value;  // manual persists
      }
      else if (c.key == "S2") {
        setServo2(c.value);
        if (c.duration == 0) baselineS2Deg = c.value;  // manual persists
      }
    }
    if (isKilled) break;

    // If any duration present, wait once, then reset only used actuators
    if (maxDur > 0) {
      if (waitMsRespectKill((uint32_t)maxDur * 1000)) { if (isKilled) break; }

      for (auto &c : cmds) {
        if      (c.key == "T1") setThruster1(0);
        else if (c.key == "T2") setThruster2(0);
        else if (c.key == "S1") { setServo1(baselineS1Deg); addLog("→ S1 reset to baseline " + String(baselineS1Deg) + "°"); }
        else if (c.key == "S2") { setServo2(baselineS2Deg); addLog("→ S2 reset to baseline " + String(baselineS2Deg) + "°"); }
      }
    }
  }

  ws.broadcastTXT("[STATUS] READY");
  executingQueue = false;
}

// ===== WebSocket =====
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  switch (type) {
    case WStype_TEXT: {
      String msg((char*)payload, len);
      msg.trim();

      if (msg == "KILL") { // software kill
        addLog("[KILL] UI kill requested");
        safeStop();
        while (!rawLines.empty()) rawLines.pop();
        executingQueue = false;
        ws.broadcastTXT("[STATUS] READY");
        return;
      }
      if (msg == "RESETKILL") { isKilled = false; digitalWrite(LED_PIN, LOW); addLog("KILL Reset via UI"); return; }

      // Accept full multi-line mission in one message
      if (msg.indexOf('\n') != -1) {
        int start = 0;
        while (start < msg.length()) {
          int sep = msg.indexOf('\n', start);
          if (sep == -1) sep = msg.length();
          String line = msg.substring(start, sep);
          line.trim();
          if (line.length()) rawLines.push(line);
          start = sep + 1;
        }
      } else {
        rawLines.push(msg);
      }
      if (!executingQueue) executeQueue();
      break;
    }
    case WStype_CONNECTED:
      addLog("[WS] Client connected");
      break;
    case WStype_DISCONNECTED:
      addLog("[WS] Client disconnected");
      break;
    default: break;
  }
}

// ===== Servers / setup =====
void setupHttp() {
  http.on("/", []() { http.send_P(200, "text/html", INDEX_HTML); });
  http.begin();
}

void setupPwm() {
  // Arduino-ESP32 v3+ LEDC API
  ledcAttach(THRUSTER1_PIN, PWM_FREQ_HZ, 16);
  ledcAttach(THRUSTER2_PIN, PWM_FREQ_HZ, 16);
  ledcAttach(SERVO1_PIN, PWM_FREQ_HZ, 16);
  ledcAttach(SERVO2_PIN, PWM_FREQ_HZ, 16);

  // Set neutral at boot
  uint32_t thruster_neutral = (THR_NEU_US * 65536L) / 20000;
  uint32_t servo_neutral = (SERVO_NEU_US * 65536L) / 20000;
  ledcWrite(THRUSTER1_PIN, thruster_neutral);
  ledcWrite(THRUSTER2_PIN, thruster_neutral);
  ledcWrite(SERVO1_PIN, servo_neutral);
  ledcWrite(SERVO2_PIN, servo_neutral);

  // Initialize ramp state at neutral
  currentS1Deg = targetS1Deg = 0;
  currentS2Deg = targetS2Deg = 0;
}

void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                 // keep Wi-Fi radio awake (prevents idle drop)
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // reed input + interrupt
  pinMode(KILL_PIN, INPUT_PULLDOWN);                // assumes reed brings pin HIGH when closed
  attachInterrupt(digitalPinToInterrupt(KILL_PIN), onKillISR, RISING);

  setupPwm();
  startAP();
  setupHttp();

  ws.begin();
  ws.enableHeartbeat(15000, 3000, 2);  // ping every 15s, expect pong in 3s, 2 misses => drop
  ws.onEvent(onWsEvent);

  // ArduinoOTA optional
  // ArduinoOTA.begin();
}

// ===== periodic servo ramp update =====
void updateServoRamps() {
  uint32_t now = millis();
  if (now - lastServoTick < SERVO_TICK_MS) return;
  lastServoTick = now;

  if (currentS1Deg != targetS1Deg) {
    int dir = (targetS1Deg > currentS1Deg) ? 1 : -1;
    int delta = abs(targetS1Deg - currentS1Deg);
    int step = min(SERVO_STEP_DEG, delta);
    currentS1Deg += dir * step;
    writeServoDegImmediate(SERVO1_PIN, currentS1Deg);
  }
  if (currentS2Deg != targetS2Deg) {
    int dir = (targetS2Deg > currentS2Deg) ? 1 : -1;
    int delta = abs(targetS2Deg - currentS2Deg);
    int step = min(SERVO_STEP_DEG, delta);
    currentS2Deg += dir * step;
    writeServoDegImmediate(SERVO2_PIN, currentS2Deg);
  }
}

void loop() {
  http.handleClient();
  ws.loop();
  // ArduinoOTA.handle();

  // keep WS alive even without traffic
  if (millis() - lastWsKeepalive >= WS_KEEPALIVE_MS) {
    ws.broadcastTXT("[KA]");
    lastWsKeepalive = millis();
  }

  // process kill outside executor time
  if (killRequested) processKillIfNeeded();

  // smooth servo ramps
  updateServoRamps();

  if (isKilled) {
    unsigned long now = millis();
    if (now - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastBlink = now;
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}
