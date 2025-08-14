// Squirt Homebase
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>   // Install "arduinoWebSockets" (Markus Sattler)
#include <ArduinoOTA.h>
#include <queue>
#include <Ticker.h>

// ======= Reed Based Kill Switch =======
constexpr int LED_PIN = 2; // kill switch indicator
bool isKilled = false;
unsigned long lastBlink = 0;
bool ledState = false;

// ======= Command Execution =======
struct Cmd {
  String key;
  int value;      // % for thrusters or ° for servos
  int duration;   // in seconds, optional (0 = immediate)
};

std::queue<Cmd> cmdQueue;
bool executingQueue = false;
Ticker cmdTicker;

// ======== AP credentials =========
const char *AP_SSID = "ESP32-Squirt";
const char *AP_PASS = "squirtSwims!"; // >=8 chars

// ------- Pin assignments -------
constexpr uint8_t THRUSTER1_PIN = 25;
constexpr uint8_t THRUSTER2_PIN = 26;
constexpr uint8_t SERVO1_PIN    = 32;
constexpr uint8_t SERVO2_PIN    = 33;

// ------- Servo/ESC timing ------
constexpr uint32_t PWM_FREQ_HZ = 50;     // 50 Hz for servos/ESCs

// BlueRobotics ESC range
constexpr int THR_MIN_US = 1100;
constexpr int THR_NEU_US = 1500;
constexpr int THR_MAX_US = 1900;

// Servo neutral (same as typical)
constexpr int SERVO_MIN_US = 1000;
constexpr int SERVO_NEU_US = 1500;
constexpr int SERVO_MAX_US = 2000;

// ------- Networking ----------
WebServer http(80);
WebSocketsServer ws(81); // WebSocket on port 81

// Simple inlined UI (served at /)
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
      <input id=t1 type=range min=0 max=100 step=1 value=0>
      <div class=val><span id=t1v>0</span>%</div>
    </div>
    <div class=row><label style="width:120px">Thruster 2</label>
      <input id=t2 type=range min=0 max=100 step=1 value=0>
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
    <div style="font-size:12px;color:#666;margin-top:8px">
      Hint: keep this page focused while moving sliders for smooth updates.
    </div>
  </div>

  <div class=card>
    <h3>Command Sequence</h3>
    <textarea id=cmdseq placeholder="Example:\nT1:50;S1:30;WAIT:1000\nT1:0;S1:0;WAIT:2000"></textarea>
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
        log.textContent += e.data + "\n";
        if (log.textContent.length > 5000) log.textContent = log.textContent.slice(-4000);
        log.scrollTop = log.scrollHeight;
      };
    }
    connectWS();

    function bindSlider(id, fmt, key){
      const el = document.getElementById(id);
      const vl = document.getElementById(id+"v");
      const update = () => { vl.textContent = el.value; send(`${key}:${el.value}`); };
      el.addEventListener('input', update);
      el.addEventListener('change', update);
      update();
    }

    bindSlider('t1', '%', 'T1');
    bindSlider('t2', '%', 'T2');
    bindSlider('s1', '°', 'S1');
    bindSlider('s2', '°', 'S2');

    document.getElementById('kill').onclick = () => { send("KILL"); };
    document.getElementById('resetKill').onclick = () => { send("RESETKILL"); };
    document.getElementById('logToggle').onchange = (e) => {
      showLogs = e.target.checked;
    };

    document.getElementById('runSeq').onclick = () => {
      const lines = document.getElementById('cmdseq').value.trim().split("\n");
      for (const line of lines) {
        send(line);  // let ESP32 do all timing via WAIT commands
      }
    };
  </script>
</body>
</html>
)HTML";

// ===== Utility mappers (percent/angle -> microseconds) =====
int servoAngleToUs(int angleDeg) {
  angleDeg = constrain(angleDeg, -90, 90);
  return map(angleDeg, -90, 90, SERVO_MIN_US, SERVO_MAX_US);
}
int thrusterPctToUs(int pct) {
  pct = constrain(pct, -100, 100);
  if (pct >= 0)  return map(pct, 0, 100, THR_NEU_US, THR_MAX_US); // forward
  else           return map(pct, -100, 0, THR_MIN_US, THR_NEU_US); // reverse
}

// ===== Apply outputs via ESP32Servo =====
void setThruster1(int pct) {
  int pulse_us = thrusterPctToUs(pct);
  uint32_t duty = (pulse_us * 65536L) / 20000;
  ledcWrite(THRUSTER1_PIN, duty);  // ✅ correct

  char msg[64];
  snprintf(msg, sizeof(msg), "T1: %d%% → %dus → duty %lu", pct, pulse_us, duty);
  Serial.println(msg);
  ws.broadcastTXT(msg);
}

void setThruster2(int pct) {
  int pulse_us = thrusterPctToUs(pct);
  uint32_t duty = (pulse_us * 65536L) / 20000;
  ledcWrite(THRUSTER2_PIN, duty);  // ✅ correct

  char msg[64];
  snprintf(msg, sizeof(msg), "T2: %d%% → %dus → duty %lu", pct, pulse_us, duty);
  Serial.println(msg);
  ws.broadcastTXT(msg);
}

void setServo1(int ang) {
  int us = servoAngleToUs(ang);
  uint32_t duty = (us * 65536L) / 20000;
  ledcWrite(SERVO1_PIN, duty);

  char msg[64];
  snprintf(msg, sizeof(msg), "S1: %d° → %dus → duty %lu", ang, us, duty);
  Serial.println(msg);
  ws.broadcastTXT(msg);
}

void setServo2(int ang) {
  int us = servoAngleToUs(ang);
  uint32_t duty = (us * 65536L) / 20000;
  ledcWrite(SERVO2_PIN, duty);

  char msg[64];
  snprintf(msg, sizeof(msg), "S2: %d° → %dus → duty %lu", ang, us, duty);
  Serial.println(msg);
  ws.broadcastTXT(msg);
}

void safeStop() {
  uint32_t thruster_neutral = (THR_NEU_US * 65536L) / 20000;
  uint32_t servo_neutral = (SERVO_NEU_US * 65536L) / 20000;

  ledcWrite(THRUSTER1_PIN, thruster_neutral);
  ledcWrite(THRUSTER2_PIN, thruster_neutral);
  ledcWrite(SERVO1_PIN, servo_neutral);
  ledcWrite(SERVO2_PIN, servo_neutral);

  Serial.println("Safe stop: all outputs to neutral");
  ws.broadcastTXT("Safe stop: all outputs to neutral");

  isKilled = true;   // Enable LED blinking
}

// ===== WebSocket handler =====
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = ws.remoteIP(num);
      Serial.printf("[WS] Client %u connected from %s\n", num, ip.toString().c_str());
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client %u disconnected\n", num);
      // if (ws.connectedClients() == 0) {
      //   Serial.println("[WS] No clients -> safeStop()");
      //   safeStop();
      // }
      // break;
    case WStype_TEXT: {
      String msg((char*)payload, len);
      msg.trim();

      if (msg.indexOf(';') != -1) {
        // It's a sequence of commands (multi-cmd line)
        int pos = 0;
        while ((pos = msg.indexOf(';')) != -1) {
          String part = msg.substring(0, pos);
          msg = msg.substring(pos + 1);
          part.trim();
          int colon = part.indexOf(':');
          if (colon > 0) {
            String key = part.substring(0, colon);
            int val = part.substring(colon + 1).toInt();
            cmdQueue.push({ key, val });
          }
        }
        if (msg.length()) {
          int colon = msg.indexOf(':');
          if (colon > 0) {
            String key = msg.substring(0, colon);
            int val = msg.substring(colon + 1).toInt();
            cmdQueue.push({ key, val });
          }
        }

        if (!executingQueue) {
          executeQueue();
        }

      } else {
        // Single command or KILL
        if (msg == "KILL") {
          safeStop();
          while (!cmdQueue.empty()) cmdQueue.pop();
        } else if (msg == "RESETKILL") {
          isKilled = false;
          digitalWrite(LED_PIN, LOW);
          Serial.println("[KILL] Reset via UI");
          ws.broadcastTXT("[KILL] Reset via UI");
        } else {
          int colon = msg.indexOf(':');
          if (colon > 0) {
            String key = msg.substring(0, colon);
            int val = msg.substring(colon + 1).toInt();
            handleCommand({ key, val });
          }
        }
      }
      break;
    }
    default: break;
  }
}

// ===== HTTP routes =====
void setupHttp() {
  http.on("/", []() { http.send_P(200, "text/html", INDEX_HTML); });
  http.begin();
  Serial.println("HTTP server started (/, Web UI)");
}

// ===== PWM init via ESP32Servo =====
void setupPwm() {
  // Setup pins with new LEDC API
  ledcAttach(THRUSTER1_PIN, PWM_FREQ_HZ, 16);  // CH 0 auto-assigned
  ledcAttach(THRUSTER2_PIN, PWM_FREQ_HZ, 16);  // CH 1
  ledcAttach(SERVO1_PIN,    PWM_FREQ_HZ, 16);  // CH 2
  ledcAttach(SERVO2_PIN,    PWM_FREQ_HZ, 16);  // CH 3

  // Send neutral pulses to all outputs
  ledcWrite(THRUSTER1_PIN, (THR_NEU_US * 65536L) / 20000);
  ledcWrite(THRUSTER2_PIN, (THR_NEU_US * 65536L) / 20000);
  ledcWrite(SERVO1_PIN,    (SERVO_NEU_US * 65536L) / 20000);
  ledcWrite(SERVO2_PIN,    (SERVO_NEU_US * 65536L) / 20000);

  Serial.println("PWM outputs initialized (neutral)");
  ws.broadcastTXT("PWM outputs initialized (neutral)");

  delay(500);
}

void handleCommand(const Cmd& cmd) {
  String log = "[CMD] " + cmd.key + ":" + String(cmd.value);
  ws.broadcastTXT(log);
  Serial.println(log);

  if (cmd.key == "T1") setThruster1(cmd.value);
  else if (cmd.key == "T2") setThruster2(cmd.value);
  else if (cmd.key == "S1") setServo1(cmd.value);
  else if (cmd.key == "S2") setServo2(cmd.value);
}

void executeQueue() {
  executingQueue = true;

  std::vector<Cmd> batch;

  while (!cmdQueue.empty()) {
    Cmd cmd = cmdQueue.front();
    cmdQueue.pop();

    if (cmd.key == "KILL") {
      safeStop();
      while (!cmdQueue.empty()) cmdQueue.pop(); // clear rest
      break;
    }

    if (cmd.key == "WAIT") {
      // Execute all batched commands before waiting
      for (const Cmd& c : batch) handleCommand(c);
      batch.clear();

      delay(cmd.value * 1000);
      char buf[64];
      snprintf(buf, sizeof(buf), "[SEQ] Waited %d seconds", cmd.value);
      Serial.println(buf);
      ws.broadcastTXT(buf);
    } else {
      batch.push_back(cmd);
    }
  }

  // Run any remaining commands if no WAIT after them
  for (const Cmd& c : batch) handleCommand(c);

  executingQueue = false;
  Serial.println("[SEQ] Finished command sequence");
  ws.broadcastTXT("[SEQ] Finished command sequence");
}

// ===== AP + servers =====
void startAP() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("AP start failed, rebooting...");
    delay(1500);
    ESP.restart();
  }
  delay(100);
  Serial.printf("AP SSID: %s  PASS: %s\n", AP_SSID, AP_PASS);
  Serial.printf("AP IP:   %s\n", WiFi.softAPIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\nBooting…");

  setupPwm();
  startAP();
  setupHttp();

  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("WebSocket server started (ws://<ap-ip>:81/)");

  // Optional ArduinoOTA:
  // ArduinoOTA.setHostname("esp32-ap-control");
  // ArduinoOTA.begin();

  pinMode(18, OUTPUT); // switch sensing
  digitalWrite(18, HIGH);
  pinMode(19, INPUT_PULLDOWN);

  pinMode(LED_PIN, OUTPUT); // blink detection
  digitalWrite(LED_PIN, LOW);  // LED off at startup

  Serial.println("Ready. Connect to the AP and open http://192.168.4.1/");
}

void loop() {
  http.handleClient();
  ws.loop();
  // ArduinoOTA.handle(); // if enabled

  if (digitalRead(19) == HIGH) {
    if (!isKilled) {
      Serial.println("Reed Switch Triggered");
      ws.broadcastTXT("[KILL] Hardware kill triggered");
      isKilled = true;
      safeStop();
      while (!cmdQueue.empty()) cmdQueue.pop();
      executingQueue = false;
    }
  }

  if (isKilled) {
    unsigned long now = millis();
    if (now - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastBlink = now;
    }
  } else {
    digitalWrite(LED_PIN, LOW); // off when not killed
  }
}
