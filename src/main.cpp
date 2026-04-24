/*
 * ClockSpinner — MKS DLC32 stepper firmware with WLED-compatible network API
 *
 * Direct access (AP):  Connect to "ClockSpinner" / "clockwork" → http://192.168.4.1
 * Network access (STA): Connect to your WiFi via the setup page, then the device
 *                        advertises itself via mDNS (_wled._tcp) and is discovered
 *                        automatically by the Nebula app.
 * Serial: 115200 baud
 *
 * HARDWARE: STEP/DIR/ENABLE route through 74HC595 shift registers.
 *   GPIO 16 = SR_BCK, GPIO 17 = SR_WS, GPIO 21 = SR_DATA
 *   Bit 0=ENABLE(active LOW)  Bit 1=X_STEP  Bit 2=X_DIR
 *   Bit 5=Y_STEP  Bit 6=Y_DIR
 */

// =============================================================================
// SECTION 1: INCLUDES AND DEFINES
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Shift register pins
#define SR_BCK   16
#define SR_WS    17
#define SR_DATA  21

// Shift register bit indices
#define BIT_ENABLE  0
#define BIT_X_STEP  1
#define BIT_X_DIR   2
#define BIT_Y_STEP  5
#define BIT_Y_DIR   6

// Motor config
#define STEPS_PER_REV  3200
#define MIN_RPM        0.5f
#define MAX_RPM        120.0f
#define DEFAULT_RPM    30.0f

// AP fallback (always available for direct access)
#define AP_SSID  "ClockSpinner"
#define AP_PASS  "clockwork"

// =============================================================================
// SECTION 2: GLOBALS
// =============================================================================

// Shift register
static volatile uint32_t srState = 0;
static portMUX_TYPE      srMux   = portMUX_INITIALIZER_UNLOCKED;

// Motor state
volatile float currentRpm   = DEFAULT_RPM;
volatile bool  motorRunning = false;
volatile bool  dirForward   = true;
hw_timer_t*    stepTimer    = nullptr;

// WLED state — `on` maps to motor start/stop; `bri` stored but not yet mapped
bool    wledOn  = false;
uint8_t wledBri = 128;

// Config (persisted in /cfg.json on LittleFS)
String deviceName = "MotorController";
String wifiSSID   = "";
String wifiPass   = "";

// Runtime flags
bool isSTAMode = false;

// Web server (built-in synchronous WebServer — motor runs in ISR so blocking is fine)
WebServer server(80);

// =============================================================================
// SECTION 3: SHIFT REGISTER FUNCTIONS
// =============================================================================

void IRAM_ATTR shiftOut32() {
    digitalWrite(SR_WS, LOW);
    uint32_t s = srState;
    for (int i = 31; i >= 0; i--) {
        digitalWrite(SR_DATA, (s >> i) & 1);
        digitalWrite(SR_BCK, HIGH);
        digitalWrite(SR_BCK, LOW);
    }
    digitalWrite(SR_WS, HIGH);
}

void srWrite(uint8_t bitIndex, bool val) {
    portENTER_CRITICAL(&srMux);
    if (val) srState |=  (1UL << bitIndex);
    else     srState &= ~(1UL << bitIndex);
    shiftOut32();
    portEXIT_CRITICAL(&srMux);
}

// =============================================================================
// SECTION 4: MOTOR HELPERS
// =============================================================================

uint32_t rpmToMicros(float rpm) {
    return (uint32_t)(1e6f / ((rpm / 60.0f) * STEPS_PER_REV));
}

void applyDirection() {
    srWrite(BIT_X_DIR, dirForward);
    srWrite(BIT_Y_DIR, dirForward);
}

void enableMotors() {
    srWrite(BIT_ENABLE, false);   // output LOW = active-low ENABLE asserted
    Serial.printf("enableMotors srState=0x%X\n", srState);
}

void disableMotors() {
    srWrite(BIT_ENABLE, true);    // output HIGH = ENABLE de-asserted
    Serial.printf("disableMotors srState=0x%X\n", srState);
}

// =============================================================================
// SECTION 5: STEP ISR
// =============================================================================

void IRAM_ATTR onStep() {
    portENTER_CRITICAL_ISR(&srMux);
    srState |=  (1UL << BIT_X_STEP) | (1UL << BIT_Y_STEP);
    shiftOut32();
    delayMicroseconds(10);
    srState &= ~((1UL << BIT_X_STEP) | (1UL << BIT_Y_STEP));
    shiftOut32();
    portEXIT_CRITICAL_ISR(&srMux);
}

// =============================================================================
// SECTION 6: MOTOR CONTROL
// =============================================================================

void startMotor() {
    if (motorRunning) return;
    Serial.printf("startMotor: %.1f RPM\n", (float)currentRpm);
    applyDirection();
    enableMotors();
    stepTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(stepTimer, &onStep, true);
    timerAlarmWrite(stepTimer, rpmToMicros(currentRpm), true);
    timerAlarmEnable(stepTimer);
    motorRunning = true;
    wledOn = true;
}

void stopMotor() {
    if (!motorRunning) return;
    timerAlarmDisable(stepTimer);
    timerDetachInterrupt(stepTimer);
    timerEnd(stepTimer);
    stepTimer = nullptr;
    disableMotors();
    motorRunning = false;
    wledOn = false;
}

void setSpeed(float rpm) {
    rpm = constrain(rpm, MIN_RPM, MAX_RPM);
    currentRpm = rpm;
    if (motorRunning && stepTimer != nullptr) {
        timerAlarmWrite(stepTimer, rpmToMicros(rpm), true);
    }
}

// =============================================================================
// SECTION 7: CONFIG PERSISTENCE (LittleFS /cfg.json)
// =============================================================================

void loadConfig() {
    if (!SPIFFS.exists("/cfg.json")) {
        Serial.println("No /cfg.json — using defaults");
        return;
    }
    File f = SPIFFS.open("/cfg.json", "r");
    if (!f) { Serial.println("cfg.json open failed"); return; }
    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) { Serial.printf("cfg.json parse error: %s\n", err.c_str()); return; }
    deviceName = doc["id"]["name"] | "MotorController";
    if (doc["nw"]["ins"].size() > 0) {
        wifiSSID = doc["nw"]["ins"][0]["ssid"] | "";
        wifiPass = doc["nw"]["ins"][0]["psk"]  | "";
    }
    Serial.printf("Config: name=%s ssid=%s\n", deviceName.c_str(), wifiSSID.c_str());
}

void saveConfig() {
    File f = SPIFFS.open("/cfg.json", "w");
    if (!f) { Serial.println("cfg.json write failed"); return; }
    DynamicJsonDocument doc(512);
    doc["id"]["name"] = deviceName;
    JsonObject ins0 = doc["nw"]["ins"].createNestedObject();
    ins0["ssid"] = wifiSSID;
    ins0["psk"]  = wifiPass;
    serializeJson(doc, f);
    f.close();
    Serial.println("Config saved");
}

// =============================================================================
// SECTION 8: WLED JSON BUILDERS
// =============================================================================

// Populates a JsonObject with the full WLED state.
// `on` mirrors motorRunning; `bri` is stored but not yet mapped to motor speed.
void populateState(JsonObject root) {
    root["on"]         = motorRunning;
    root["bri"]        = wledBri;
    root["transition"] = 7;
    root["ps"]         = -1;
    root["pss"]        = 0;
    root["pl"]         = -1;
    root["mainseg"]    = 0;

    JsonArray seg = root.createNestedArray("seg");
    JsonObject s  = seg.createNestedObject();
    s["id"]    = 0;   s["start"] = 0;   s["stop"]  = 1;
    s["len"]   = 1;   s["grp"]   = 1;   s["spc"]   = 0;
    s["of"]    = 0;   s["on"]    = motorRunning;
    s["frz"]   = false; s["bri"] = 255; s["cct"]   = 127; s["set"] = 0;
    JsonArray col = s.createNestedArray("col");
    JsonArray c1 = col.createNestedArray(); c1.add(128); c1.add(0); c1.add(0);
    JsonArray c2 = col.createNestedArray(); c2.add(0);   c2.add(0); c2.add(0);
    JsonArray c3 = col.createNestedArray(); c3.add(0);   c3.add(0); c3.add(0);
    s["fx"]  = 0;   s["sx"]  = 128; s["ix"]  = 128; s["pal"] = 0;
    s["c1"]  = 128; s["c2"]  = 128; s["c3"]  = 16;
    s["sel"] = true; s["rev"] = false; s["mi"] = false;
    s["o1"]  = false; s["o2"] = false; s["o3"] = false;
    s["si"]  = 0;   s["m12"] = 0;
}

// Populates a JsonObject with WLED device info. `brand:"WLED"` is required for
// Nebula app discovery. Dynamic fields are populated from ESP32 runtime values.
void populateInfo(JsonObject root) {
    String mac = WiFi.macAddress();
    mac.replace(":", ""); mac.toLowerCase();

    root["ver"]          = "0.15.0-b3";
    root["vid"]          = 2409250;
    root["name"]         = deviceName;
    root["brand"]        = "WLED";        // ← required for Nebula app discovery
    root["product"]      = "FOSS";
    root["mac"]          = mac;
    root["ip"]           = isSTAMode ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    root["arch"]         = "esp32";
    root["core"]         = "v4.4.7";
    root["clock"]        = 160;
    root["flash"]        = 4;
    root["freeheap"]     = (int)ESP.getFreeHeap();
    root["uptime"]       = (int)(millis() / 1000);
    root["opt"]          = 1;
    root["live"]         = false;
    root["liveseg"]      = -1;
    root["lm"]           = "";
    root["lip"]          = "";
    root["ws"]           = -1;
    root["fxcount"]      = 1;
    root["palcount"]     = 1;
    root["cpalcount"]    = 0;
    root["str"]          = false;
    root["simplifiedui"] = false;
    root["udpport"]      = 21324;
    root["ndc"]          = -1;
    root.createNestedArray("maps");

    JsonObject leds = root.createNestedObject("leds");
    leds["count"]  = 1;  leds["lc"]     = 1;
    leds.createNestedArray("seglc").add(1);
    leds["rgbw"]   = false; leds["wv"]  = false; leds["cct"]    = 0;
    leds["fps"]    = 40;    leds["bootps"] = 0;  leds["pwr"]    = 0;
    leds["maxpwr"] = 0;     leds["maxseg"] = 1;

    int rssi = isSTAMode ? WiFi.RSSI() : 0;
    JsonObject wf = root.createNestedObject("wifi");
    wf["bssid"]   = isSTAMode ? WiFi.BSSIDstr() : "";
    wf["rssi"]    = rssi;
    wf["signal"]  = (rssi == 0) ? 0 : min(max(2 * (rssi + 100), 0), 100);
    wf["channel"] = isSTAMode ? (int)WiFi.channel() : 0;
    wf["ap"]      = !isSTAMode;

    JsonObject fs = root.createNestedObject("fs");
    fs["u"] = 10; fs["t"] = 1000; fs["pmt"] = 0;

    root["time"] = "2026-01-01, 00:00:00";
}

static void addCORS() {
    server.sendHeader("Access-Control-Allow-Origin",  "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static void sendJson(int code, const String &body) {
    addCORS();
    server.send(code, "application/json", body);
}

// =============================================================================
// SECTION 9: HTML PAGE
// =============================================================================

const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ClockSpinner</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0d0d1a; color: #e0e0e0;
    font-family: system-ui, -apple-system, sans-serif;
    padding: 24px 16px 48px;
  }
  .container { max-width: 480px; margin: 0 auto; }
  h1 { font-size: 1.9rem; font-weight: 700; color: #7eb8f7; text-align: center;
       letter-spacing: 0.04em; margin-bottom: 22px; }
  .card { background: #141428; border: 1px solid #2a2a4a; border-radius: 14px;
          padding: 18px 20px; margin-bottom: 14px; }
  .card-title { font-size: 0.72rem; color: #555; text-transform: uppercase;
                letter-spacing: 0.1em; margin-bottom: 12px; }
  .info-row { display: flex; justify-content: space-between; align-items: center;
              margin-bottom: 6px; font-size: 0.88rem; }
  .info-key { color: #666; }
  .info-val { color: #aaa; font-weight: 500; }
  .info-val.good { color: #4cff88; }
  .info-val.warn { color: #f7c97e; }
  .badge { display: inline-block; padding: 4px 14px; border-radius: 20px;
           font-size: 0.78rem; font-weight: 700; letter-spacing: 0.1em; }
  .badge-running { background: #0d3320; color: #4cff88; border: 1px solid #1a6640; }
  .badge-stopped { background: #330d0d; color: #ff4c4c; border: 1px solid #661a1a; }
  .speed-big { font-size: 2.4rem; font-weight: 700; color: #7eb8f7;
               text-align: center; margin: 8px 0 2px; }
  .speed-unit { font-size: 0.85rem; color: #555; text-align: center; margin-bottom: 6px; }
  .dir-line { font-size: 0.88rem; color: #777; text-align: center; }
  .dir-arrow { color: #7eb8f7; }
  .slider-row { display: flex; align-items: center; gap: 12px; margin: 10px 0 8px; }
  input[type=range] {
    flex: 1; -webkit-appearance: none; appearance: none;
    height: 6px; border-radius: 3px; background: #2a2a4a; outline: none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none; width: 22px; height: 22px;
    border-radius: 50%; background: #7eb8f7; cursor: pointer; border: 2px solid #0d0d1a;
  }
  input[type=range]::-moz-range-thumb {
    width: 22px; height: 22px; border-radius: 50%;
    background: #7eb8f7; cursor: pointer; border: 2px solid #0d0d1a;
  }
  .slider-val { min-width: 52px; text-align: right; font-size: 1rem;
                color: #7eb8f7; font-weight: 600; }
  .btn-grid { display: grid; gap: 10px; margin-top: 8px; }
  .col2 { grid-template-columns: 1fr 1fr; }
  .col1 { grid-template-columns: 1fr; }
  button, input[type=submit] {
    padding: 13px 10px; border: none; border-radius: 10px;
    font-size: 0.88rem; font-weight: 600; cursor: pointer;
    letter-spacing: 0.03em; transition: opacity 0.15s, transform 0.1s;
    width: 100%;
  }
  button:active, input[type=submit]:active { opacity: 0.75; transform: scale(0.97); }
  .btn-accent { background: #7eb8f7; color: #0d0d1a; }
  .btn-start  { background: #1a6640; color: #4cff88; border: 1px solid #2a9960; }
  .btn-stop   { background: #661a1a; color: #ff6666; border: 1px solid #993030; }
  .btn-dir    { background: #1e1e38; color: #7eb8f7; border: 1px solid #3a3a6a; }
  .btn-save   { background: #1a3828; color: #7ef7a0; border: 1px solid #2a6040; }
  .btn-reboot { background: #2a1a1a; color: #f7a07e; border: 1px solid #6a3010; }
  label { display: block; font-size: 0.8rem; color: #888; margin-bottom: 5px; }
  input[type=text], input[type=password] {
    width: 100%; background: #0d0d1a; border: 1px solid #3a3a6a;
    border-radius: 8px; color: #e0e0e0; font-size: 0.95rem;
    padding: 10px 12px; outline: none; margin-bottom: 10px;
  }
  input[type=text]:focus, input[type=password]:focus { border-color: #7eb8f7; }
  .divider { border: none; border-top: 1px solid #1e1e38; margin: 12px 0; }
  .status-line { margin-top: 14px; font-size: 0.7rem; color: #3a3a5a;
                 text-align: center; min-height: 1.1em; word-break: break-all; }
</style>
</head>
<body>
<div class="container">
  <h1>Clock Spinner</h1>

  <!-- DEVICE INFO -->
  <div class="card">
    <div class="card-title">Device</div>
    <div class="info-row">
      <span class="info-key">Name</span>
      <span class="info-val" id="devName">—</span>
    </div>
    <div class="info-row">
      <span class="info-key">AP address</span>
      <span class="info-val good">192.168.4.1</span>
    </div>
    <div class="info-row">
      <span class="info-key">Network IP</span>
      <span class="info-val" id="devIP">—</span>
    </div>
    <div class="info-row">
      <span class="info-key">WiFi</span>
      <span class="info-val" id="devWifi">—</span>
    </div>
  </div>

  <!-- MOTOR STATUS -->
  <div class="card">
    <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:10px">
      <span class="card-title" style="margin-bottom:0">Motor</span>
      <span class="badge badge-stopped" id="statusBadge">STOPPED</span>
    </div>
    <div class="speed-big" id="speedDisplay">30.0</div>
    <div class="speed-unit">RPM</div>
    <div class="dir-line">
      Direction: <span class="dir-arrow" id="dirArrow">&#8594;</span>
      <span id="dirLabel">Forward</span>
    </div>
  </div>

  <!-- SPEED CONTROL -->
  <div class="card">
    <div class="card-title">Speed</div>
    <div class="slider-row">
      <input type="range" id="rpmSlider" min="0.5" max="120" step="0.5" value="30">
      <span class="slider-val" id="sliderVal">30.0</span>
    </div>
    <div class="btn-grid col1">
      <button class="btn-accent" onclick="sendSpeed()">Set Speed</button>
    </div>
  </div>

  <!-- CONTROL BUTTONS -->
  <div class="card">
    <div class="card-title">Control</div>
    <div class="btn-grid col2">
      <button class="btn-start" onclick="sendCmd('/start')">&#9654; Start</button>
      <button class="btn-stop"  onclick="sendCmd('/stop')">&#9646;&#9646; Stop</button>
    </div>
    <div class="btn-grid col1" style="margin-top:10px">
      <button class="btn-dir" onclick="sendCmd('/direction')">&#8646; Reverse Direction</button>
    </div>
  </div>

  <!-- WIFI SETTINGS -->
  <div class="card">
    <div class="card-title">WiFi Network</div>
    <form method="POST" action="/settings/wifi">
      <label for="ssid">Network SSID</label>
      <input type="text" id="ssid" name="ssid" placeholder="Your WiFi name" autocomplete="off">
      <label for="psk">Password</label>
      <input type="password" id="psk" name="psk" placeholder="WiFi password" autocomplete="off">
      <input type="submit" class="btn-save" value="&#10003; Save WiFi &amp; Reboot">
    </form>
  </div>

  <!-- DEVICE NAME -->
  <div class="card">
    <div class="card-title">Device Name</div>
    <label for="nameInput">Shown in Nebula app</label>
    <input type="text" id="nameInput" placeholder="MotorController" autocomplete="off">
    <button class="btn-save" onclick="saveName()">&#10003; Save Name</button>
  </div>

  <!-- REBOOT -->
  <div class="card">
    <button class="btn-reboot" onclick="if(confirm('Reboot device?')) sendCmd('/reset')">
      &#8635; Reboot Device
    </button>
  </div>

  <div class="status-line" id="statusLine">Connecting…</div>
</div>

<script>
  const slider = document.getElementById('rpmSlider');
  slider.addEventListener('input', () => {
    document.getElementById('sliderVal').textContent = parseFloat(slider.value).toFixed(1);
  });

  function setStatus(msg) { document.getElementById('statusLine').textContent = msg; }

  function updateMotorUI(data) {
    if (document.activeElement !== slider) {
      slider.value = data.rpm;
      document.getElementById('sliderVal').textContent = parseFloat(data.rpm).toFixed(1);
    }
    document.getElementById('speedDisplay').textContent = parseFloat(data.rpm).toFixed(1);
    const badge = document.getElementById('statusBadge');
    badge.textContent = data.running ? 'RUNNING' : 'STOPPED';
    badge.className   = data.running ? 'badge badge-running' : 'badge badge-stopped';
    const fwd = data.dir === 'fwd';
    document.getElementById('dirArrow').innerHTML = fwd ? '&#8594;' : '&#8592;';
    document.getElementById('dirLabel').textContent = fwd ? 'Forward' : 'Reverse';
  }

  function updateDeviceUI(info) {
    document.getElementById('devName').textContent = info.name || '—';
    document.getElementById('devIP').textContent   = info.ip || '—';
    const wf = info.wifi;
    if (wf && !wf.ap && wf.bssid) {
      document.getElementById('devWifi').textContent = '&#10003; ' + (wf.bssid || 'connected');
      document.getElementById('devWifi').classList.add('good');
    } else {
      document.getElementById('devWifi').textContent = 'AP only (not joined)';
      document.getElementById('devWifi').classList.add('warn');
    }
    const ni = document.getElementById('nameInput');
    if (document.activeElement !== ni) ni.value = info.name || '';
  }

  function fetchStatus() {
    fetch('/status')
      .then(r => r.json()).then(updateMotorUI)
      .catch(e => setStatus('Motor status error: ' + e));
  }

  function fetchInfo() {
    fetch('/json/info')
      .then(r => r.json()).then(updateDeviceUI)
      .catch(() => {});
  }

  function sendCmd(path) {
    fetch(path, { method: 'POST' })
      .then(r => r.json())
      .then(d => { setStatus(path + ' \u2192 ' + JSON.stringify(d)); fetchStatus(); })
      .catch(e => setStatus('Error: ' + e));
  }

  function sendSpeed() {
    fetch('/speed', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'rpm=' + slider.value
    }).then(r => r.json())
      .then(d => { setStatus('/speed \u2192 ' + JSON.stringify(d)); fetchStatus(); })
      .catch(e => setStatus('Error: ' + e));
  }

  function saveName() {
    const name = document.getElementById('nameInput').value.trim();
    if (!name) return;
    fetch('/settings/ui', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'name=' + encodeURIComponent(name)
    }).then(r => r.json())
      .then(() => { setStatus('Name saved'); fetchInfo(); })
      .catch(e => setStatus('Error: ' + e));
  }

  fetchStatus();
  fetchInfo();
  setInterval(fetchStatus, 3000);
  setInterval(fetchInfo, 10000);
</script>
</body>
</html>
)rawhtml";

// =============================================================================
// SECTION 10: WEB SERVER ROUTES
// =============================================================================

void setupRoutes() {

    // OPTIONS catch-all — CORS preflight for all paths
    server.onNotFound([]() {
        if (server.method() == HTTP_OPTIONS) {
            addCORS();
            server.send(200, "text/plain", "");
        } else {
            server.send(404, "text/plain", "Not found");
        }
    });

    // Root — config + motor control page
    server.on("/", HTTP_GET, []() {
        server.send_P(200, "text/html", INDEX_HTML);
    });

    // ---- Config endpoints ----------------------------------------

    // POST /settings/wifi — save credentials and reboot
    server.on("/settings/wifi", HTTP_POST, []() {
        if (server.hasArg("ssid")) wifiSSID = server.arg("ssid");
        if (server.hasArg("psk"))  wifiPass = server.arg("psk");
        saveConfig();
        server.send(200, "text/html",
            "<html><body style='background:#0d0d1a;color:#e0e0e0;"
            "font-family:sans-serif;text-align:center;padding:3rem'>"
            "<h2 style='color:#7eb8f7'>&#10003; Saved!</h2>"
            "<p>Rebooting now&hellip;</p></body></html>");
        delay(500);
        ESP.restart();
    });

    // POST /settings/ui — save device name (no reboot needed)
    server.on("/settings/ui", HTTP_POST, []() {
        if (server.hasArg("name")) {
            deviceName = server.arg("name");
            saveConfig();
        }
        sendJson(200, "{\"ok\":true}");
    });

    // POST /reset — reboot
    server.on("/reset", HTTP_POST, []() {
        server.send(200, "text/plain", "Rebooting...");
        delay(500);
        ESP.restart();
    });

    // ---- Motor control endpoints ---------------------------------

    server.on("/status", HTTP_GET, []() {
        sendJson(200, "{\"rpm\":"     + String(currentRpm, 1)
                    + ",\"running\":" + (motorRunning ? "true" : "false")
                    + ",\"dir\":\""   + (dirForward ? "fwd" : "rev") + "\"}");
    });

    server.on("/start", HTTP_POST, []() {
        startMotor();
        sendJson(200, "{\"ok\":true}");
    });

    server.on("/stop", HTTP_POST, []() {
        stopMotor();
        sendJson(200, "{\"ok\":true}");
    });

    server.on("/direction", HTTP_POST, []() {
        dirForward = !dirForward;
        if (motorRunning) applyDirection();
        sendJson(200, String("{\"ok\":true,\"dir\":\"") + (dirForward ? "fwd" : "rev") + "\"}");
    });

    server.on("/speed", HTTP_POST, []() {
        if (server.hasArg("rpm")) setSpeed(server.arg("rpm").toFloat());
        sendJson(200, "{\"ok\":true,\"rpm\":" + String(currentRpm, 1) + "}");
    });

    // ---- WLED JSON API -------------------------------------------

    // GET /json/info
    server.on("/json/info", HTTP_GET, []() {
        DynamicJsonDocument doc(2560);
        populateInfo(doc.to<JsonObject>());
        String out; serializeJson(doc, out);
        sendJson(200, out);
    });

    // GET /json/state
    server.on("/json/state", HTTP_GET, []() {
        DynamicJsonDocument doc(1536);
        populateState(doc.to<JsonObject>());
        String out; serializeJson(doc, out);
        sendJson(200, out);
    });

    // POST /json/state — raw JSON body via server.arg("plain")
    // `on` maps to motor start/stop; `bri` stored but not yet mapped
    server.on("/json/state", HTTP_POST, []() {
        String body = server.arg("plain");
        DynamicJsonDocument bodyDoc(256);
        if (body.length() > 0 && deserializeJson(bodyDoc, body) == DeserializationError::Ok) {
            if (bodyDoc.containsKey("on")) {
                bool on = bodyDoc["on"].as<bool>();
                if (on  && !motorRunning) startMotor();
                if (!on &&  motorRunning) stopMotor();
            }
            if (bodyDoc.containsKey("bri")) wledBri = bodyDoc["bri"].as<uint8_t>();
        }
        DynamicJsonDocument resp(1536);
        populateState(resp.to<JsonObject>());
        String out; serializeJson(resp, out);
        sendJson(200, out);
    });

    // GET /json — combined state + info + effects + palettes
    server.on("/json", HTTP_GET, []() {
        DynamicJsonDocument doc(5120);
        JsonObject root = doc.to<JsonObject>();
        populateState(root.createNestedObject("state"));
        populateInfo(root.createNestedObject("info"));
        root.createNestedArray("effects").add("Solid");
        root.createNestedArray("palettes").add("Default");
        String out; serializeJson(doc, out);
        sendJson(200, out);
    });

    // GET /json/effects
    server.on("/json/effects",  HTTP_GET, []() { sendJson(200, "[\"Solid\"]");   });
    // GET /json/palettes
    server.on("/json/palettes", HTTP_GET, []() { sendJson(200, "[\"Default\"]"); });

    // GET /win — WLED legacy XML API
    server.on("/win", HTTP_GET, []() {
        String name = deviceName;
        name.replace("&", "&amp;"); name.replace("<", "&lt;"); name.replace(">", "&gt;");
        String xml =
            "<?xml version=\"1.0\" ?><vs>"
            "<ac>" + String(wledBri) + "</ac>"
            "<cl>128</cl><cl>0</cl><cl>0</cl>"
            "<cs>0</cs><cs>0</cs><cs>0</cs>"
            "<ns>0</ns><nr>0</nr><nl>0</nl><nf>0</nf><nd>0</nd><nt>0</nt>"
            "<sq>0</sq><gn>0</gn><fx>0</fx><sx>128</sx><ix>128</ix>"
            "<f1>0</f1><f2>0</f2><f3>0</f3><fp>0</fp>"
            "<wv>-1</wv><ws>0</ws><ps>0</ps><cy>0</cy>"
            "<ds>" + name + "</ds>"
            "<ss>0</ss></vs>";
        server.send(200, "text/xml", xml);
    });

    server.begin();
    Serial.println("Web server started");
}

// =============================================================================
// SECTION 11: WIFI + mDNS SETUP
// =============================================================================

void setupWifi() {
    // Always run the AP so the device is directly accessible even without network
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("AP ready: %s — http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

    if (wifiSSID.length() == 0) {
        Serial.println("No WiFi credentials stored — AP only mode");
        Serial.println("Connect to ClockSpinner and open http://192.168.4.1 to configure");
        return;
    }

    Serial.printf("Connecting to '%s'", wifiSSID.c_str());
    WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());

    unsigned long deadline = millis() + 12000;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi failed — AP only mode");
        return;
    }

    isSTAMode = true;
    Serial.printf("\nConnected! STA IP: %s\n", WiFi.localIP().toString().c_str());

    // mDNS — hostname wled-XXXXXX (last 6 chars of MAC, lowercase).
    // Nebula app discovers devices by scanning for _wled._tcp on the network.
    String mac = WiFi.macAddress();
    mac.replace(":", ""); mac.toLowerCase();
    String mdnsName = "wled-" + mac.substring(6);

    if (MDNS.begin(mdnsName.c_str())) {
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("wled", "tcp", 80);                          // ← discovery hook
        MDNS.addServiceTxt("wled", "tcp", "mac", mac.c_str());
        MDNS.addServiceTxt("wled", "tcp", "ver", "0.15.0-b3");
        MDNS.addServiceTxt("wled", "tcp", "ip",  WiFi.localIP().toString().c_str());
        Serial.printf("mDNS: http://%s.local  (will appear in Nebula as '%s')\n",
                      mdnsName.c_str(), deviceName.c_str());
    } else {
        Serial.println("mDNS start failed");
    }
}

// =============================================================================
// SECTION 12: setup()
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\nClockSpinner booting...");

    // Init shift register GPIO pins
    pinMode(SR_BCK,  OUTPUT); pinMode(SR_WS,   OUTPUT); pinMode(SR_DATA, OUTPUT);
    digitalWrite(SR_BCK, LOW); digitalWrite(SR_WS, LOW); digitalWrite(SR_DATA, LOW);
    srState = 0;
    shiftOut32();
    srWrite(BIT_ENABLE, true);   // ENABLE HIGH = motors disabled at boot
    Serial.printf("SR init: srState=0x%X\n", srState);

    // Mount LittleFS (format on first boot if needed)
    if (!SPIFFS.begin(true)) {
        Serial.println("LittleFS mount/format failed");
    } else {
        Serial.println("LittleFS mounted");
        loadConfig();
    }

    // WiFi (STA if credentials saved, always AP)
    setupWifi();

    // HTTP routes
    setupRoutes();

    // Auto-start motor
    Serial.println("Auto-start: spinning up...");
    startMotor();

    Serial.println("Boot complete");
}

// =============================================================================
// SECTION 13: loop()
// =============================================================================

void loop() {
    server.handleClient();
    delay(1);
}
