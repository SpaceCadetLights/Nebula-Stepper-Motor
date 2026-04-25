/*
 * Water Wheel Control — MKS DLC32 stepper firmware with WLED-compatible network API
 *
 * Direct access (AP):  Connect to "ClockSpinner" / "clockwork" → http://192.168.4.1
 * Network access (STA): Connect to your WiFi via Settings, then the device
 *                        advertises itself via mDNS (_wled._tcp) for the Nebula app.
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
#include <Update.h>
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
#define SLIDER_MAX_RPM 70.0f

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

// Config (persisted in /cfg.json on SPIFFS)
String deviceName = "WaterWheel";
String wifiSSID   = "";
String wifiPass   = "";

// Runtime flags
bool isSTAMode = false;

// Web server
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
// SECTION 7: CONFIG PERSISTENCE (SPIFFS /cfg.json)
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
    deviceName  = doc["id"]["name"] | "WaterWheel";
    currentRpm  = doc["motor"]["rpm"] | DEFAULT_RPM;
    dirForward  = doc["motor"]["dir"] | 1;
    currentRpm  = constrain(currentRpm, MIN_RPM, MAX_RPM);
    if (doc["nw"]["ins"].size() > 0) {
        wifiSSID = doc["nw"]["ins"][0]["ssid"] | "";
        wifiPass = doc["nw"]["ins"][0]["psk"]  | "";
    }
    Serial.printf("Config: name=%s ssid=%s rpm=%.1f dir=%s\n",
                  deviceName.c_str(), wifiSSID.c_str(),
                  (float)currentRpm, dirForward ? "fwd" : "rev");
}

void saveConfig() {
    File f = SPIFFS.open("/cfg.json", "w");
    if (!f) { Serial.println("cfg.json write failed"); return; }
    DynamicJsonDocument doc(512);
    doc["id"]["name"]     = deviceName;
    doc["motor"]["rpm"]   = (float)currentRpm;
    doc["motor"]["dir"]   = dirForward ? 1 : 0;
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

void populateInfo(JsonObject root) {
    String mac = WiFi.macAddress();
    mac.replace(":", ""); mac.toLowerCase();

    root["ver"]          = "0.15.0-b3";
    root["vid"]          = 2409250;
    root["name"]         = deviceName;
    root["brand"]        = "WLED";
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
// SECTION 9: HTML — MAIN CONTROL PAGE
// =============================================================================

const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Water Wheel Control</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #080814; color: #e0e0e0;
    font-family: system-ui, -apple-system, sans-serif;
    min-height: 100vh;
  }
  .topbar {
    display: flex; align-items: center; justify-content: space-between;
    padding: 18px 20px 14px;
    border-bottom: 1px solid #1a1a30;
  }
  .topbar-title {
    font-size: 1.25rem; font-weight: 700; color: #7eb8f7;
    letter-spacing: 0.03em;
  }
  .topbar-sub { font-size: 0.7rem; color: #444; margin-top: 2px; letter-spacing: 0.06em; text-transform: uppercase; }
  .gear-link {
    display: flex; align-items: center; justify-content: center;
    width: 40px; height: 40px; border-radius: 10px;
    background: #141428; border: 1px solid #2a2a4a;
    color: #7eb8f7; font-size: 1.2rem; text-decoration: none;
    transition: background 0.15s;
  }
  .gear-link:active { background: #1e1e3a; }

  .main { padding: 20px 18px 48px; max-width: 440px; margin: 0 auto; }

  /* Status hero */
  .status-hero {
    background: #0f0f20; border: 1px solid #1e1e38;
    border-radius: 18px; padding: 24px 20px 20px;
    text-align: center; margin-bottom: 14px;
  }
  .badge {
    display: inline-block; padding: 5px 18px; border-radius: 20px;
    font-size: 0.72rem; font-weight: 700; letter-spacing: 0.12em;
    text-transform: uppercase; margin-bottom: 14px;
  }
  .badge-running { background: #0a2a18; color: #3dffa0; border: 1px solid #1a5535; }
  .badge-stopped { background: #200a0a; color: #ff5555; border: 1px solid #551515; }
  .rpm-big {
    font-size: 4rem; font-weight: 800; color: #7eb8f7;
    line-height: 1; letter-spacing: -0.02em;
  }
  .rpm-unit { font-size: 0.8rem; color: #444; letter-spacing: 0.12em; margin-top: 4px; margin-bottom: 4px; }
  .dir-line { font-size: 0.82rem; color: #555; margin-top: 6px; }
  .dir-arrow { color: #7eb8f7; font-size: 1rem; }

  /* Cards */
  .card {
    background: #0f0f20; border: 1px solid #1e1e38;
    border-radius: 14px; padding: 18px 18px; margin-bottom: 12px;
  }
  .card-label {
    font-size: 0.68rem; color: #444; text-transform: uppercase;
    letter-spacing: 0.12em; margin-bottom: 12px;
  }

  /* Slider */
  .slider-row { display: flex; align-items: center; gap: 14px; }
  input[type=range] {
    flex: 1; -webkit-appearance: none; appearance: none;
    height: 6px; border-radius: 3px; background: #1e1e38; outline: none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none; width: 24px; height: 24px;
    border-radius: 50%; background: #7eb8f7; cursor: pointer;
    border: 2px solid #080814; box-shadow: 0 0 8px #7eb8f740;
  }
  input[type=range]::-moz-range-thumb {
    width: 24px; height: 24px; border-radius: 50%;
    background: #7eb8f7; cursor: pointer; border: 2px solid #080814;
  }
  .slider-val {
    min-width: 52px; text-align: right;
    font-size: 1.1rem; color: #7eb8f7; font-weight: 700;
  }
  .set-btn {
    width: 100%; margin-top: 12px; padding: 12px;
    background: #1a2a3a; color: #7eb8f7; border: 1px solid #2a3a5a;
    border-radius: 10px; font-size: 0.88rem; font-weight: 600;
    cursor: pointer; letter-spacing: 0.04em;
    transition: background 0.12s, transform 0.1s;
  }
  .set-btn:active { background: #243550; transform: scale(0.97); }

  /* Power button */
  .power-btn {
    width: 100%; padding: 20px 10px; border: none; border-radius: 14px;
    font-size: 1.15rem; font-weight: 700; cursor: pointer;
    letter-spacing: 0.06em; transition: opacity 0.15s, transform 0.1s;
    margin-bottom: 12px;
  }
  .power-btn:active { opacity: 0.75; transform: scale(0.97); }
  .power-start { background: #0d3320; color: #3dffa0; border: 1px solid #1a6640; }
  .power-stop  { background: #330d0d; color: #ff6666; border: 1px solid #661a1a; }

  /* Direction */
  .dir-btn {
    width: 100%; padding: 15px 10px; border-radius: 12px;
    background: #131328; color: #7eb8f7; border: 1px solid #2a2a50;
    font-size: 0.92rem; font-weight: 600; cursor: pointer;
    letter-spacing: 0.04em; transition: background 0.12s, transform 0.1s;
  }
  .dir-btn:active { background: #1e1e3e; transform: scale(0.97); }

  .status-line {
    text-align: center; font-size: 0.68rem; color: #2a2a44;
    margin-top: 18px; min-height: 1em; word-break: break-all;
  }
</style>
</head>
<body>

<div class="topbar">
  <div>
    <div class="topbar-title">Water Wheel Control</div>
    <div class="topbar-sub" id="topSub">Connecting...</div>
  </div>
  <a href="/settings" class="gear-link" title="Settings">&#9881;</a>
</div>

<div class="main">

  <!-- STATUS HERO -->
  <div class="status-hero">
    <div><span class="badge badge-stopped" id="statusBadge">STOPPED</span></div>
    <div class="rpm-big" id="speedDisplay">30.0</div>
    <div class="rpm-unit">RPM</div>
    <div class="dir-line">
      <span class="dir-arrow" id="dirArrow">&#8594;</span>
      <span id="dirLabel">Forward</span>
    </div>
  </div>

  <!-- SPEED -->
  <div class="card">
    <div class="card-label">Speed</div>
    <div class="slider-row">
      <input type="range" id="rpmSlider" min="0.5" max="70" step="0.5" value="30">
      <span class="slider-val" id="sliderVal">30.0</span>
    </div>
    <button class="set-btn" onclick="sendSpeed()">Set Speed</button>
  </div>

  <!-- POWER + DIRECTION -->
  <button id="powerBtn" class="power-btn power-start" onclick="togglePower()">
    &#9654;&nbsp; Start
  </button>
  <button class="dir-btn" onclick="sendCmd('/direction')">
    &#8646;&nbsp; Reverse Direction
  </button>

  <div class="status-line" id="statusLine">&nbsp;</div>

</div>

<script>
  let isRunning = false;

  const slider = document.getElementById('rpmSlider');
  slider.addEventListener('input', () => {
    document.getElementById('sliderVal').textContent = parseFloat(slider.value).toFixed(1);
  });

  function setStatus(msg) {
    document.getElementById('statusLine').textContent = msg;
  }

  function updateMotorUI(data) {
    isRunning = data.running;
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
    const btn = document.getElementById('powerBtn');
    if (data.running) {
      btn.innerHTML = '&#9646;&#9646;&nbsp; Stop';
      btn.className = 'power-btn power-stop';
    } else {
      btn.innerHTML = '&#9654;&nbsp; Start';
      btn.className = 'power-btn power-start';
    }
    document.getElementById('topSub').textContent = data.running
      ? parseFloat(data.rpm).toFixed(1) + ' RPM — Running'
      : 'Stopped';
  }

  function togglePower() {
    sendCmd(isRunning ? '/stop' : '/start');
  }

  function fetchStatus() {
    fetch('/status')
      .then(r => r.json()).then(updateMotorUI)
      .catch(() => {});
  }

  function sendCmd(path) {
    fetch(path, { method: 'POST' })
      .then(r => r.json())
      .then(() => fetchStatus())
      .catch(e => setStatus('Error: ' + e));
  }

  function sendSpeed() {
    fetch('/speed', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'rpm=' + slider.value
    }).then(r => r.json())
      .then(() => { setStatus('Speed set to ' + slider.value + ' RPM'); fetchStatus(); })
      .catch(e => setStatus('Error: ' + e));
  }

  fetchStatus();
  setInterval(fetchStatus, 3000);
</script>
</body>
</html>
)rawhtml";

// =============================================================================
// SECTION 10: HTML — SETTINGS PAGE
// =============================================================================

const char SETTINGS_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Settings — Water Wheel</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #080814; color: #e0e0e0;
    font-family: system-ui, -apple-system, sans-serif;
    min-height: 100vh;
  }
  .topbar {
    display: flex; align-items: center; gap: 14px;
    padding: 18px 20px 14px;
    border-bottom: 1px solid #1a1a30;
  }
  .back-link {
    display: flex; align-items: center; justify-content: center;
    width: 38px; height: 38px; border-radius: 10px;
    background: #141428; border: 1px solid #2a2a4a;
    color: #7eb8f7; font-size: 1.1rem; text-decoration: none;
    flex-shrink: 0; transition: background 0.12s;
  }
  .back-link:active { background: #1e1e3a; }
  .topbar-title { font-size: 1.2rem; font-weight: 700; color: #e0e0e0; }

  .main { padding: 18px 18px 60px; max-width: 440px; margin: 0 auto; }

  .card {
    background: #0f0f20; border: 1px solid #1e1e38;
    border-radius: 14px; padding: 18px; margin-bottom: 12px;
  }
  .card-title {
    font-size: 0.68rem; color: #7eb8f7; text-transform: uppercase;
    letter-spacing: 0.12em; margin-bottom: 14px; font-weight: 600;
  }
  .info-row {
    display: flex; justify-content: space-between; align-items: center;
    margin-bottom: 8px; font-size: 0.87rem;
  }
  .info-key { color: #555; }
  .info-val { color: #999; font-weight: 500; }
  .info-val.good { color: #3dffa0; }
  .info-val.warn { color: #f7c97e; }

  label { display: block; font-size: 0.78rem; color: #666; margin-bottom: 5px; }
  input[type=text], input[type=password] {
    width: 100%; background: #080814; border: 1px solid #2a2a4a;
    border-radius: 8px; color: #e0e0e0; font-size: 0.95rem;
    padding: 11px 13px; outline: none; margin-bottom: 12px;
  }
  input[type=text]:focus, input[type=password]:focus { border-color: #7eb8f7; }
  input[type=file] {
    width: 100%; background: #080814; border: 1px solid #2a2a4a;
    border-radius: 8px; color: #999; font-size: 0.85rem;
    padding: 10px 12px; margin-bottom: 12px; cursor: pointer;
  }

  .btn {
    width: 100%; padding: 13px 10px; border: none; border-radius: 10px;
    font-size: 0.88rem; font-weight: 600; cursor: pointer;
    letter-spacing: 0.04em; transition: opacity 0.15s, transform 0.1s;
  }
  .btn:active { opacity: 0.78; transform: scale(0.97); }
  .btn-save   { background: #0d2d1e; color: #3dffa0; border: 1px solid #1a5535; }
  .btn-accent { background: #1a2a3a; color: #7eb8f7; border: 1px solid #2a3a5a; }
  .btn-reboot { background: #1e1010; color: #f7907e; border: 1px solid #4a2010; }
  .btn-update { background: #1a1a2a; color: #c0a8ff; border: 1px solid #3a2a6a; }

  .divider { border: none; border-top: 1px solid #1a1a2e; margin: 12px 0; }

  .ota-progress {
    display: none; background: #1a1a2a; border-radius: 8px;
    overflow: hidden; height: 8px; margin-bottom: 10px;
  }
  .ota-bar {
    height: 100%; background: #7eb8f7; width: 0%;
    transition: width 0.2s; border-radius: 8px;
  }
  .ota-status {
    font-size: 0.78rem; color: #666; text-align: center;
    min-height: 1.1em; margin-bottom: 10px;
  }
  .status-line {
    font-size: 0.72rem; color: #2a2a44; text-align: center;
    margin-top: 16px; min-height: 1em;
  }
</style>
</head>
<body>

<div class="topbar">
  <a href="/" class="back-link" title="Back">&#8592;</a>
  <div class="topbar-title">Settings</div>
</div>

<div class="main">

  <!-- DEVICE INFO -->
  <div class="card">
    <div class="card-title">Device Info</div>
    <div class="info-row">
      <span class="info-key">Name</span>
      <span class="info-val" id="devName">—</span>
    </div>
    <div class="info-row">
      <span class="info-key">AP (direct)</span>
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
    <div class="info-row">
      <span class="info-key">Free heap</span>
      <span class="info-val" id="devHeap">—</span>
    </div>
    <div class="info-row">
      <span class="info-key">Uptime</span>
      <span class="info-val" id="devUptime">—</span>
    </div>
  </div>

  <!-- WIFI -->
  <div class="card">
    <div class="card-title">WiFi Network</div>
    <form id="wifiForm">
      <label for="ssid">Network SSID</label>
      <input type="text" id="ssid" name="ssid" placeholder="WiFi network name" autocomplete="off">
      <label for="psk">Password</label>
      <input type="password" id="psk" name="psk" placeholder="WiFi password" autocomplete="off">
      <button type="button" class="btn btn-save" onclick="saveWifi()">
        &#10003; Save &amp; Reboot
      </button>
    </form>
  </div>

  <!-- DEVICE NAME -->
  <div class="card">
    <div class="card-title">Device Name</div>
    <label for="nameInput">Shown in Nebula app</label>
    <input type="text" id="nameInput" placeholder="WaterWheel" autocomplete="off">
    <button class="btn btn-save" onclick="saveName()">&#10003; Save Name</button>
  </div>

  <!-- FIRMWARE UPDATE -->
  <div class="card">
    <div class="card-title">Firmware Update (OTA)</div>
    <label for="fwFile">Select .bin file</label>
    <input type="file" id="fwFile" accept=".bin">
    <div class="ota-status" id="otaStatus">Choose a firmware .bin file to upload</div>
    <div class="ota-progress" id="otaProgressWrap">
      <div class="ota-bar" id="otaBar"></div>
    </div>
    <button class="btn btn-update" onclick="startOTA()">&#8593; Upload Firmware</button>
  </div>

  <!-- REBOOT -->
  <div class="card">
    <button class="btn btn-reboot" onclick="doReboot()">
      &#8635; Reboot Device
    </button>
  </div>

  <div class="status-line" id="statusLine">&nbsp;</div>

</div>

<script>
  function setStatus(msg) { document.getElementById('statusLine').textContent = msg; }

  function updateDeviceUI(info) {
    document.getElementById('devName').textContent = info.name || '—';
    document.getElementById('devIP').textContent   = info.ip  || '—';
    document.getElementById('devHeap').textContent =
      info.freeheap ? Math.round(info.freeheap / 1024) + ' KB' : '—';
    document.getElementById('devUptime').textContent =
      info.uptime ? Math.floor(info.uptime / 60) + 'm ' + (info.uptime % 60) + 's' : '—';
    const wf = info.wifi;
    const wifiEl = document.getElementById('devWifi');
    if (wf && !wf.ap && wf.bssid) {
      wifiEl.textContent = '\u2713 Connected (' + wf.signal + '%)';
      wifiEl.className = 'info-val good';
    } else {
      wifiEl.textContent = 'AP only';
      wifiEl.className = 'info-val warn';
    }
    const ni = document.getElementById('nameInput');
    if (document.activeElement !== ni) ni.value = info.name || '';
  }

  function saveWifi() {
    const ssid = document.getElementById('ssid').value.trim();
    const psk  = document.getElementById('psk').value;
    if (!ssid) { setStatus('Enter an SSID first'); return; }
    const body = 'ssid=' + encodeURIComponent(ssid) + '&psk=' + encodeURIComponent(psk);
    fetch('/settings/wifi', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body
    }).catch(() => {});
    setStatus('Saved — device is rebooting...');
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

  function doReboot() {
    if (!confirm('Reboot the device?')) return;
    fetch('/reset', { method: 'POST' }).catch(() => {});
    setStatus('Rebooting...');
  }

  function startOTA() {
    const file = document.getElementById('fwFile').files[0];
    if (!file) { setStatus('Please select a .bin file first'); return; }
    if (!file.name.endsWith('.bin')) { setStatus('File must be a .bin firmware image'); return; }

    const formData = new FormData();
    formData.append('firmware', file);

    const bar   = document.getElementById('otaBar');
    const wrap  = document.getElementById('otaProgressWrap');
    const status = document.getElementById('otaStatus');

    wrap.style.display = 'block';
    bar.style.width = '0%';
    status.textContent = 'Uploading...';
    document.querySelector('.btn-update').disabled = true;

    const xhr = new XMLHttpRequest();
    xhr.upload.addEventListener('progress', e => {
      if (e.lengthComputable) {
        const pct = Math.round(e.loaded / e.total * 100);
        bar.style.width = pct + '%';
        status.textContent = 'Uploading... ' + pct + '%';
      }
    });
    xhr.addEventListener('load', () => {
      bar.style.width = '100%';
      if (xhr.status === 200 && !xhr.responseText.includes('FAIL')) {
        status.textContent = '\u2713 ' + xhr.responseText;
        setStatus('Update complete — device is rebooting');
      } else {
        status.textContent = 'Update failed: ' + xhr.responseText;
        setStatus('OTA update failed');
        document.querySelector('.btn-update').disabled = false;
      }
    });
    xhr.addEventListener('error', () => {
      status.textContent = 'Upload error — check connection';
      document.querySelector('.btn-update').disabled = false;
    });
    xhr.open('POST', '/update');
    xhr.send(formData);
  }

  function fetchInfo() {
    fetch('/json/info')
      .then(r => r.json()).then(updateDeviceUI)
      .catch(() => {});
  }

  fetchInfo();
  setInterval(fetchInfo, 8000);
</script>
</body>
</html>
)rawhtml";

// =============================================================================
// SECTION 11: WEB SERVER ROUTES
// =============================================================================

void setupRoutes() {

    // OPTIONS catch-all — CORS preflight
    server.onNotFound([]() {
        if (server.method() == HTTP_OPTIONS) {
            addCORS();
            server.send(200, "text/plain", "");
        } else {
            server.send(404, "text/plain", "Not found");
        }
    });

    // GET / — main control page
    server.on("/", HTTP_GET, []() {
        server.send_P(200, "text/html", INDEX_HTML);
    });

    // GET /settings — settings page
    server.on("/settings", HTTP_GET, []() {
        server.send_P(200, "text/html", SETTINGS_HTML);
    });

    // ---- Config endpoints ----------------------------------------

    server.on("/settings/wifi", HTTP_POST, []() {
        if (server.hasArg("ssid")) wifiSSID = server.arg("ssid");
        if (server.hasArg("psk"))  wifiPass = server.arg("psk");
        saveConfig();
        server.send(200, "text/html",
            "<html><body style='background:#080814;color:#e0e0e0;"
            "font-family:sans-serif;text-align:center;padding:3rem'>"
            "<h2 style='color:#3dffa0'>&#10003; Saved!</h2>"
            "<p style='color:#666;margin-top:8px'>Rebooting now&hellip;</p></body></html>");
        delay(500);
        ESP.restart();
    });

    server.on("/settings/ui", HTTP_POST, []() {
        if (server.hasArg("name")) {
            deviceName = server.arg("name");
            saveConfig();
        }
        sendJson(200, "{\"ok\":true}");
    });

    server.on("/reset", HTTP_POST, []() {
        server.send(200, "text/plain", "Rebooting...");
        delay(500);
        ESP.restart();
    });

    // ---- OTA firmware update via HTTP upload --------------------

    server.on("/update", HTTP_POST,
        []() {
            server.sendHeader("Connection", "close");
            bool ok = !Update.hasError();
            server.send(200, "text/plain", ok ? "Update OK! Rebooting..." : "Update FAILED!");
            delay(1000);
            ESP.restart();
        },
        []() {
            HTTPUpload &upload = server.upload();
            if (upload.status == UPLOAD_FILE_START) {
                Serial.printf("OTA start: %s\n", upload.filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                }
            } else if (upload.status == UPLOAD_FILE_WRITE) {
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                }
            } else if (upload.status == UPLOAD_FILE_END) {
                if (Update.end(true)) {
                    Serial.printf("OTA success: %u bytes\n", upload.totalSize);
                } else {
                    Update.printError(Serial);
                }
            }
        }
    );

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
        saveConfig();
        sendJson(200, String("{\"ok\":true,\"dir\":\"") + (dirForward ? "fwd" : "rev") + "\"}");
    });

    server.on("/speed", HTTP_POST, []() {
        if (server.hasArg("rpm")) setSpeed(server.arg("rpm").toFloat());
        saveConfig();
        sendJson(200, "{\"ok\":true,\"rpm\":" + String(currentRpm, 1) + "}");
    });

    // ---- WLED JSON API -------------------------------------------

    server.on("/json/info", HTTP_GET, []() {
        DynamicJsonDocument doc(2560);
        populateInfo(doc.to<JsonObject>());
        String out; serializeJson(doc, out);
        sendJson(200, out);
    });

    server.on("/json/state", HTTP_GET, []() {
        DynamicJsonDocument doc(1536);
        populateState(doc.to<JsonObject>());
        String out; serializeJson(doc, out);
        sendJson(200, out);
    });

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

    server.on("/json/effects",  HTTP_GET, []() { sendJson(200, "[\"Solid\"]");   });
    server.on("/json/palettes", HTTP_GET, []() { sendJson(200, "[\"Default\"]"); });

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
// SECTION 12: WIFI + mDNS SETUP
// =============================================================================

void setupWifi() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("AP ready: %s — http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

    if (wifiSSID.length() == 0) {
        Serial.println("No WiFi credentials — AP only mode");
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

    String mac = WiFi.macAddress();
    mac.replace(":", ""); mac.toLowerCase();
    String mdnsName = "wled-" + mac.substring(6);

    if (MDNS.begin(mdnsName.c_str())) {
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("wled", "tcp", 80);
        MDNS.addServiceTxt("wled", "tcp", "mac", mac.c_str());
        MDNS.addServiceTxt("wled", "tcp", "ver", "0.15.0-b3");
        MDNS.addServiceTxt("wled", "tcp", "ip",  WiFi.localIP().toString().c_str());
        Serial.printf("mDNS: http://%s.local  (Nebula name: '%s')\n",
                      mdnsName.c_str(), deviceName.c_str());
    } else {
        Serial.println("mDNS start failed");
    }
}

// =============================================================================
// SECTION 13: setup()
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\nWater Wheel booting...");

    pinMode(SR_BCK,  OUTPUT); pinMode(SR_WS,   OUTPUT); pinMode(SR_DATA, OUTPUT);
    digitalWrite(SR_BCK, LOW); digitalWrite(SR_WS, LOW); digitalWrite(SR_DATA, LOW);
    srState = 0;
    shiftOut32();
    srWrite(BIT_ENABLE, true);
    Serial.printf("SR init: srState=0x%X\n", srState);

    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
    } else {
        Serial.println("SPIFFS mounted");
        loadConfig();
    }

    setupWifi();
    setupRoutes();

    Serial.println("Auto-start: spinning up...");
    startMotor();

    Serial.println("Boot complete");
}

// =============================================================================
// SECTION 14: loop()
// =============================================================================

void loop() {
    server.handleClient();
    delay(1);
}
