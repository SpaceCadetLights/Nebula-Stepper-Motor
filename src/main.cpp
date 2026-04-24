/*
 * ClockSpinner — MKS DLC32 constant-speed stepper firmware
 *
 * Wi-Fi: Connect to "ClockSpinner" (password: clockwork)
 * Web UI: http://192.168.4.1
 * Serial monitor: 115200 baud
 *
 * HARDWARE NOTE: STEP/DIR/ENABLE signals are NOT on direct GPIO pins.
 * They go through a 74HC595 shift register chain controlled by 3 GPIOs:
 *   GPIO 16 = SR_BCK  (bit clock)
 *   GPIO 17 = SR_WS   (latch)
 *   GPIO 21 = SR_DATA (data)
 * Bit assignments in the 32-bit shift word:
 *   Bit 0 = ENABLE (all axes, active LOW)
 *   Bit 1 = X STEP
 *   Bit 2 = X DIR
 *   Bit 5 = Y STEP
 *   Bit 6 = Y DIR
 *
 * If motors don't move:
 *   1. Check serial output — confirm "Web server started" message
 *   2. Verify you can reach http://192.168.4.1
 *   3. Try clicking Start in the web UI and watch Serial for any errors
 *   4. Check that 24V (or 12V) power is connected to the motor power input
 *   5. If motors buzz but don't move, try reversing direction in the UI
 */

// =============================================================================
// SECTION 1: INCLUDES AND DEFINES
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

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

// Motor hardware limits (motor RPM, never exposed to user)
#define STEPS_PER_REV   3200      // 200 full steps × 16x microstepping
#define MIN_MOTOR_RPM   0.5f
#define MAX_MOTOR_RPM   120.0f

// User-facing defaults (output RPM)
#define DEFAULT_GEAR_RATIO    1.0f    // 1 motor turn = 1 output turn
#define DEFAULT_MAX_OUT_RPM   120.0f  // max output RPM allowed by slider
#define DEFAULT_TARGET_RPM    30.0f   // default target output RPM on first boot
#define DEFAULT_ACCEL         5.0f    // output RPM per second
// Start Speed: motor launches at this output RPM before ramping to target.
// Default = target RPM (instant start, same as before acceleration was added).
// Lower it only if the wheel can tolerate a slow start without stalling.
#define DEFAULT_START_RPM     DEFAULT_TARGET_RPM

// Wi-Fi
#define WIFI_SSID  "ClockSpinner"
#define WIFI_PASS  "clockwork"

// =============================================================================
// SECTION 2: GLOBALS
// =============================================================================

static volatile uint32_t srState = 0;
static portMUX_TYPE      srMux   = portMUX_INITIALIZER_UNLOCKED;

// Internal motor speed (ramps toward targetMotorRpm, used by timer ISR)
volatile float currentMotorRpm = MIN_MOTOR_RPM;
volatile float targetMotorRpm  = DEFAULT_TARGET_RPM;

volatile bool  motorRunning = false;
volatile bool  dirForward   = true;

// User-facing settings (all in output RPM / output RPM per second)
float targetOutputRpm  = DEFAULT_TARGET_RPM;  // what user set
float startOutputRpm   = DEFAULT_START_RPM;   // output RPM the motor launches at (before ramp)
float accelRpmPerSec   = DEFAULT_ACCEL;        // output RPM/sec
float gearRatio        = DEFAULT_GEAR_RATIO;   // output_rpm = motor_rpm * gearRatio
float configMaxOutRpm  = DEFAULT_MAX_OUT_RPM;  // slider upper limit
bool  bootAutoStart    = true;

hw_timer_t* stepTimer = nullptr;
WebServer   server(80);
Preferences prefs;

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

uint32_t rpmToMicros(float motorRpm) {
    return (uint32_t)(1e6f / ((motorRpm / 60.0f) * STEPS_PER_REV));
}

// Convert output RPM → motor RPM, clamped to hardware limits
float outputToMotor(float outputRpm) {
    if (gearRatio <= 0.0f) return MIN_MOTOR_RPM;
    return constrain(outputRpm / gearRatio, MIN_MOTOR_RPM, MAX_MOTOR_RPM);
}

// Convert motor RPM → output RPM
float motorToOutput(float motorRpm) {
    return motorRpm * gearRatio;
}

void applyDirection() {
    srWrite(BIT_X_DIR, dirForward);
    srWrite(BIT_Y_DIR, dirForward);
    Serial.print("applyDirection: ");
    Serial.println(dirForward ? "FWD" : "REV");
}

void enableMotors() {
    srWrite(BIT_ENABLE, false);   // clear bit = output LOW = active-low ENABLE asserted
    Serial.print("enableMotors: srState=0x");
    Serial.println(srState, HEX);
}

void disableMotors() {
    srWrite(BIT_ENABLE, true);    // set bit = output HIGH = ENABLE de-asserted
    Serial.print("disableMotors: srState=0x");
    Serial.println(srState, HEX);
}

// =============================================================================
// SECTION 4.5: SETTINGS PERSISTENCE
// =============================================================================

void loadSettings() {
    prefs.begin("clockspinner", true);
    gearRatio       = prefs.getFloat("gearRatio",   DEFAULT_GEAR_RATIO);
    configMaxOutRpm = prefs.getFloat("maxOutRpm",   DEFAULT_MAX_OUT_RPM);
    targetOutputRpm = prefs.getFloat("targetOutRpm",DEFAULT_TARGET_RPM);
    startOutputRpm  = prefs.getFloat("startOutRpm", DEFAULT_START_RPM);
    accelRpmPerSec  = prefs.getFloat("accel",       DEFAULT_ACCEL);
    bootAutoStart   = prefs.getBool ("autostart",   true);
    dirForward      = prefs.getBool ("dir",         true);
    prefs.end();

    targetMotorRpm  = outputToMotor(targetOutputRpm);
    currentMotorRpm = MIN_MOTOR_RPM;

    Serial.printf("Loaded: outputTarget=%.2f startOut=%.2f motorTarget=%.2f ratio=%.4f maxOut=%.2f accel=%.2f autostart=%d dir=%d\n",
        targetOutputRpm, startOutputRpm, (float)targetMotorRpm, gearRatio, configMaxOutRpm,
        accelRpmPerSec, (int)bootAutoStart, (int)(bool)dirForward);
}

void saveSettings() {
    prefs.begin("clockspinner", false);
    prefs.putFloat("gearRatio",   gearRatio);
    prefs.putFloat("maxOutRpm",   configMaxOutRpm);
    prefs.putFloat("targetOutRpm",targetOutputRpm);
    prefs.putFloat("startOutRpm", startOutputRpm);
    prefs.putFloat("accel",       accelRpmPerSec);
    prefs.putBool ("autostart",   bootAutoStart);
    prefs.putBool ("dir",         dirForward);
    prefs.end();

    Serial.printf("Saved:  outputTarget=%.2f startOut=%.2f ratio=%.4f maxOut=%.2f accel=%.2f autostart=%d\n",
        targetOutputRpm, startOutputRpm, gearRatio, configMaxOutRpm, accelRpmPerSec, (int)bootAutoStart);
}

// =============================================================================
// SECTION 5: STEP ISR
// =============================================================================

void IRAM_ATTR onStep() {
    // DO NOT call srWrite() here — use portENTER_CRITICAL_ISR + direct bit ops
    portENTER_CRITICAL_ISR(&srMux);
    srState |=  (1UL << BIT_X_STEP) | (1UL << BIT_Y_STEP);   // both HIGH
    shiftOut32();
    delayMicroseconds(10);
    srState &= ~((1UL << BIT_X_STEP) | (1UL << BIT_Y_STEP));  // both LOW
    shiftOut32();
    portEXIT_CRITICAL_ISR(&srMux);
}
// shiftOut32() called exactly twice per step — ~20 µs ISR time.

// =============================================================================
// SECTION 6: START / STOP / SET SPEED / RAMP
// =============================================================================

void startMotor() {
    if (motorRunning) return;
    // Launch at startOutputRpm (converted to motor RPM), clamped to [MIN, target].
    // Default = target RPM → instant full-speed start (matches pre-acceleration behavior,
    // ensures enough torque to overcome static friction on heavy loads like water wheels).
    float launchMotorRpm = constrain(outputToMotor(startOutputRpm),
                                     MIN_MOTOR_RPM, (float)targetMotorRpm);
    currentMotorRpm = launchMotorRpm;
    Serial.printf("startMotor: launch=%.2f target=%.2f accel=%.2f\n",
        launchMotorRpm, (float)targetMotorRpm, accelRpmPerSec);
    applyDirection();
    enableMotors();
    stepTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(stepTimer, &onStep, true);
    timerAlarmWrite(stepTimer, rpmToMicros(currentMotorRpm), true);
    timerAlarmEnable(stepTimer);
    motorRunning = true;
    Serial.println("startMotor: running");
}

void stopMotor() {
    if (!motorRunning) return;
    timerAlarmDisable(stepTimer);
    timerDetachInterrupt(stepTimer);
    timerEnd(stepTimer);
    stepTimer = nullptr;
    disableMotors();
    motorRunning = false;
}

// setSpeed takes OUTPUT RPM (user-facing). Converts to motor RPM internally.
void setSpeed(float outputRpm) {
    outputRpm = constrain(outputRpm, 0.0f, configMaxOutRpm);
    targetOutputRpm = outputRpm;
    targetMotorRpm  = outputToMotor(outputRpm);
    if (!motorRunning) {
        currentMotorRpm = MIN_MOTOR_RPM;
    }
    saveSettings();
}

// Called every loop() — ramps currentMotorRpm toward targetMotorRpm.
// accelRpmPerSec is in output RPM/sec; we divide by gearRatio for motor RPM/sec.
void updateRamp() {
    if (!motorRunning || stepTimer == nullptr) return;
    if (currentMotorRpm == targetMotorRpm) return;

    static uint32_t lastRampMs = 0;
    uint32_t now = millis();
    float dt = (now - lastRampMs) / 1000.0f;
    lastRampMs = now;

    if (dt <= 0.0f || dt > 0.5f) {
        lastRampMs = now;
        return;
    }

    // Convert output accel → motor accel
    float motorAccel = (gearRatio > 0.0f) ? (accelRpmPerSec / gearRatio) : accelRpmPerSec;
    float maxStep    = motorAccel * dt;
    float diff       = targetMotorRpm - currentMotorRpm;

    if (fabsf(diff) <= maxStep) {
        currentMotorRpm = targetMotorRpm;
    } else {
        currentMotorRpm += (diff > 0.0f) ? maxStep : -maxStep;
    }
    currentMotorRpm = constrain(currentMotorRpm, MIN_MOTOR_RPM, MAX_MOTOR_RPM);
    timerAlarmWrite(stepTimer, rpmToMicros(currentMotorRpm), true);
}

// =============================================================================
// SECTION 7: HTML PAGE
// =============================================================================

const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Clock Spinner</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0d0d1a;
    color: #e0e0e0;
    font-family: system-ui, -apple-system, sans-serif;
    min-height: 100vh;
    display: flex;
    align-items: flex-start;
    justify-content: center;
    padding: 24px 16px 48px;
  }
  .container { width: 100%; max-width: 480px; }
  h1 {
    font-size: 1.9rem;
    font-weight: 700;
    color: #7eb8f7;
    text-align: center;
    letter-spacing: 0.04em;
    margin-bottom: 22px;
  }
  .card {
    background: #141428;
    border: 1px solid #2a2a4a;
    border-radius: 14px;
    padding: 18px 20px;
    margin-bottom: 14px;
  }
  .card-title {
    font-size: 0.72rem;
    color: #555;
    text-transform: uppercase;
    letter-spacing: 0.1em;
    margin-bottom: 12px;
  }
  .status-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 10px;
  }
  .badge {
    display: inline-block;
    padding: 4px 14px;
    border-radius: 20px;
    font-size: 0.78rem;
    font-weight: 700;
    letter-spacing: 0.1em;
  }
  .badge-running { background: #0d3320; color: #4cff88; border: 1px solid #1a6640; }
  .badge-stopped { background: #330d0d; color: #ff4c4c; border: 1px solid #661a1a; }
  .rpm-row {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 14px;
    margin: 8px 0 4px;
  }
  .rpm-block { text-align: center; }
  .rpm-big   { font-size: 2.3rem; font-weight: 700; color: #7eb8f7; line-height: 1; }
  .rpm-mid   { font-size: 1.5rem; font-weight: 600; color: #4a7aaa; line-height: 1; }
  .rpm-lbl   { font-size: 0.68rem; color: #555; text-transform: uppercase; letter-spacing: 0.08em; margin-top: 3px; }
  .rpm-arrow { font-size: 1rem; color: #2a2a4a; }
  .motor-info {
    font-size: 0.72rem;
    color: #444;
    text-align: center;
    margin-top: 5px;
  }
  .dir-display { font-size: 0.88rem; color: #777; text-align: center; margin-top: 5px; }
  .dir-arrow   { color: #7eb8f7; }
  .slider-row {
    display: flex;
    align-items: center;
    gap: 10px;
    margin: 10px 0 6px;
  }
  .s-lbl { font-size: 0.75rem; color: #555; min-width: 32px; }
  input[type=range] {
    flex: 1;
    -webkit-appearance: none;
    appearance: none;
    height: 6px;
    border-radius: 3px;
    background: #2a2a4a;
    outline: none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 22px; height: 22px;
    border-radius: 50%;
    background: #7eb8f7;
    cursor: pointer;
    border: 2px solid #0d0d1a;
  }
  input[type=range]::-moz-range-thumb {
    width: 22px; height: 22px;
    border-radius: 50%;
    background: #7eb8f7;
    cursor: pointer;
    border: 2px solid #0d0d1a;
  }
  .s-val {
    min-width: 56px;
    text-align: right;
    font-size: 0.95rem;
    color: #7eb8f7;
    font-weight: 600;
  }
  .accel-hint {
    font-size: 0.72rem;
    color: #444;
    text-align: center;
    margin-top: 2px;
    min-height: 1em;
  }
  .btn-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 8px; }
  .btn-1 { display: grid; grid-template-columns: 1fr; gap: 10px; margin-top: 8px; }
  button {
    padding: 13px 10px;
    border: none;
    border-radius: 10px;
    font-size: 0.88rem;
    font-weight: 600;
    cursor: pointer;
    letter-spacing: 0.03em;
    transition: opacity 0.15s, transform 0.1s;
  }
  button:active { opacity: 0.75; transform: scale(0.97); }
  .btn-accent { background: #7eb8f7; color: #0d0d1a; }
  .btn-start  { background: #1a6640; color: #4cff88; border: 1px solid #2a9960; }
  .btn-stop   { background: #661a1a; color: #ff6666; border: 1px solid #993030; }
  .btn-dir    { background: #1e1e38; color: #7eb8f7; border: 1px solid #3a3a6a; }
  .btn-save     { background: #1a3828; color: #7ef7a0; border: 1px solid #2a6040; }
  .btn-defaults { background: #1e1a10; color: #f7c97e; border: 1px solid #6a4a10; }
  /* Number inputs */
  .field-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin: 10px 0;
    gap: 10px;
  }
  .field-info { flex: 1; }
  .field-label { font-size: 0.88rem; color: #ccc; }
  .field-hint  { font-size: 0.7rem; color: #555; margin-top: 2px; }
  input[type=number] {
    width: 100px;
    background: #0d0d1a;
    border: 1px solid #3a3a6a;
    border-radius: 8px;
    color: #7eb8f7;
    font-size: 1rem;
    font-weight: 600;
    padding: 8px 10px;
    text-align: right;
    outline: none;
    flex-shrink: 0;
    -moz-appearance: textfield;
  }
  input[type=number]::-webkit-inner-spin-button,
  input[type=number]::-webkit-outer-spin-button { -webkit-appearance: none; margin: 0; }
  input[type=number]:focus { border-color: #7eb8f7; }
  /* Toggle */
  .toggle-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin: 12px 0 4px;
    gap: 10px;
  }
  .toggle-info .tgl-label { font-size: 0.88rem; color: #ccc; }
  .toggle-info .tgl-hint  { font-size: 0.7rem;  color: #555; margin-top: 2px; }
  .toggle { position: relative; width: 50px; height: 27px; flex-shrink: 0; }
  .toggle input { opacity: 0; width: 0; height: 0; }
  .tgl-slider {
    position: absolute; inset: 0;
    background: #2a2a4a;
    border-radius: 27px;
    cursor: pointer;
    transition: background 0.2s;
  }
  .tgl-slider:before {
    content: "";
    position: absolute;
    left: 3px; top: 3px;
    width: 21px; height: 21px;
    border-radius: 50%;
    background: #555;
    transition: transform 0.2s, background 0.2s;
  }
  .toggle input:checked + .tgl-slider { background: #1a4a30; }
  .toggle input:checked + .tgl-slider:before { transform: translateX(23px); background: #4cff88; }
  .divider { border: none; border-top: 1px solid #1e1e38; margin: 12px 0; }
  .status-line {
    margin-top: 14px;
    font-size: 0.7rem;
    color: #3a3a5a;
    text-align: center;
    min-height: 1.1em;
    word-break: break-all;
  }
</style>
</head>
<body>
<div class="container">
  <h1>Clock Spinner</h1>

  <!-- STATUS -->
  <div class="card">
    <div class="status-row">
      <span class="card-title" style="margin-bottom:0">Status</span>
      <span class="badge badge-stopped" id="statusBadge">STOPPED</span>
    </div>
    <div class="rpm-row">
      <div class="rpm-block">
        <div class="rpm-big" id="curOutDisplay">0.0</div>
        <div class="rpm-lbl">Current Output RPM</div>
      </div>
      <div class="rpm-arrow">&#8594;</div>
      <div class="rpm-block">
        <div class="rpm-mid" id="tgtOutDisplay">30.0</div>
        <div class="rpm-lbl">Target Output RPM</div>
      </div>
    </div>
    <div class="motor-info" id="motorInfo">Motor: — RPM</div>
    <div class="dir-display">
      Direction: <span class="dir-arrow" id="dirArrow">&#8594;</span>
      <span id="dirLabel">Forward</span>
    </div>
  </div>

  <!-- SPEED -->
  <div class="card">
    <div class="card-title">Target Speed (Output RPM)</div>
    <div class="slider-row">
      <span class="s-lbl" id="sMinLbl">0.1</span>
      <input type="range" id="rpmSlider" min="0.1" max="120" step="0.1" value="30">
      <span class="s-val" id="sliderVal">30.0</span>
    </div>
    <div class="btn-1">
      <button class="btn-accent" onclick="sendSpeed()">Set Speed</button>
    </div>
  </div>

  <!-- ACCELERATION -->
  <div class="card">
    <div class="card-title">Acceleration (Output RPM/sec)</div>
    <div class="slider-row">
      <span class="s-lbl">0.1</span>
      <input type="range" id="accelSlider" min="0.1" max="20" step="0.1" value="5">
      <span class="s-val" id="accelVal">5.0</span>
    </div>
    <div class="accel-hint" id="accelHint">Ramp time to target: —</div>
  </div>

  <!-- CONTROL -->
  <div class="card">
    <div class="card-title">Control</div>
    <div class="btn-2">
      <button class="btn-start" onclick="sendCmd('/start')">&#9654; Start</button>
      <button class="btn-stop"  onclick="sendCmd('/stop')">&#9646;&#9646; Stop</button>
    </div>
    <div class="btn-1" style="margin-top:10px">
      <button class="btn-dir" onclick="sendCmd('/direction')">&#8646; Reverse Direction</button>
    </div>
  </div>

  <!-- SETTINGS -->
  <div class="card">
    <div class="card-title">System Settings</div>

    <div class="field-row">
      <div class="field-info">
        <div class="field-label">Gear Ratio (output / motor)</div>
        <div class="field-hint">e.g. 0.1 &rarr; 1 motor turn = 0.1 output turns</div>
      </div>
      <input type="number" id="gearRatioInput" step="0.001" min="0.001" value="1.000">
    </div>

    <div class="field-row">
      <div class="field-info">
        <div class="field-label">Max Output RPM</div>
        <div class="field-hint">Speed slider upper limit</div>
      </div>
      <input type="number" id="maxRpmInput" step="0.1" min="0.1" value="120.0">
    </div>

    <div class="field-row">
      <div class="field-info">
        <div class="field-label">Start Speed (output RPM)</div>
        <div class="field-hint">Launch speed on Start — must be high enough to move the load. Set equal to target to match pre-acceleration behavior.</div>
      </div>
      <input type="number" id="startRpmInput" step="0.1" min="0.1" value="30.0">
    </div>

    <hr class="divider">

    <div class="toggle-row">
      <div class="toggle-info">
        <div class="tgl-label">Auto-Start on Power-Up</div>
        <div class="tgl-hint">Motor spins immediately when board powers on</div>
      </div>
      <label class="toggle">
        <input type="checkbox" id="autostartToggle" checked>
        <span class="tgl-slider"></span>
      </label>
    </div>

    <div class="btn-1" style="margin-top:14px">
      <button class="btn-save" onclick="saveSettings()">&#10003; Save All Settings</button>
    </div>
    <div class="btn-1" style="margin-top:8px">
      <button class="btn-defaults" onclick="restoreDefaults()">&#8635; Restore Factory Defaults</button>
    </div>
  </div>

  <div class="status-line" id="statusLine">Connecting…</div>
</div>

<script>
  const rpmSlider   = document.getElementById('rpmSlider');
  const accelSlider = document.getElementById('accelSlider');

  rpmSlider.addEventListener('input', () => {
    document.getElementById('sliderVal').textContent = parseFloat(rpmSlider.value).toFixed(1);
    updateAccelHint();
  });

  accelSlider.addEventListener('input', () => {
    document.getElementById('accelVal').textContent = parseFloat(accelSlider.value).toFixed(1);
    updateAccelHint();
  });

  function updateAccelHint() {
    const target = parseFloat(rpmSlider.value);
    const accel  = parseFloat(accelSlider.value);
    if (accel > 0) {
      const secs = ((target - 0.1) / accel).toFixed(1);
      document.getElementById('accelHint').textContent = 'Ramp time to target: ~' + secs + 's';
    }
  }

  function updateUI(data) {
    // Current and target output RPM
    const curOut = parseFloat(data.outputRpm  || 0).toFixed(1);
    const tgtOut = parseFloat(data.targetOutputRpm || 0).toFixed(1);
    document.getElementById('curOutDisplay').textContent = curOut;
    document.getElementById('tgtOutDisplay').textContent = tgtOut;

    // Motor RPM info line
    const motorRpm = parseFloat(data.motorRpm || 0).toFixed(1);
    document.getElementById('motorInfo').textContent = 'Motor: ' + motorRpm + ' RPM';

    // Update slider max and value — skip if user is actively dragging
    if (data.maxOutputRpm) {
      rpmSlider.max = data.maxOutputRpm;
    }
    if (document.activeElement !== rpmSlider) {
      rpmSlider.value = data.targetOutputRpm || tgtOut;
      document.getElementById('sliderVal').textContent = tgtOut;
    }

    // Accel slider — skip if user is actively dragging it
    if (data.accel !== undefined && document.activeElement !== accelSlider) {
      accelSlider.value = data.accel;
      document.getElementById('accelVal').textContent = parseFloat(data.accel).toFixed(1);
    }

    // Number inputs — skip update entirely if the field has focus (user is typing)
    const gearInput = document.getElementById('gearRatioInput');
    const maxInput  = document.getElementById('maxRpmInput');
    if (data.gearRatio !== undefined && document.activeElement !== gearInput) {
      gearInput.value = parseFloat(data.gearRatio).toFixed(4);
    }
    if (data.maxOutputRpm !== undefined && document.activeElement !== maxInput) {
      maxInput.value = parseFloat(data.maxOutputRpm).toFixed(1);
    }
    const startInput = document.getElementById('startRpmInput');
    if (data.startOutputRpm !== undefined && document.activeElement !== startInput) {
      startInput.value = parseFloat(data.startOutputRpm).toFixed(1);
    }

    // Auto-start toggle
    if (data.autostart !== undefined) {
      document.getElementById('autostartToggle').checked = data.autostart;
    }

    updateAccelHint();

    // Status badge
    const badge = document.getElementById('statusBadge');
    if (data.running) {
      badge.textContent = 'RUNNING';
      badge.className = 'badge badge-running';
    } else {
      badge.textContent = 'STOPPED';
      badge.className = 'badge badge-stopped';
    }

    // Direction
    const fwd = data.dir === 'fwd';
    document.getElementById('dirArrow').innerHTML = fwd ? '&#8594;' : '&#8592;';
    document.getElementById('dirLabel').textContent = fwd ? 'Forward' : 'Reverse';
  }

  function setStatus(msg) {
    document.getElementById('statusLine').textContent = msg;
  }

  function fetchStatus() {
    fetch('/status')
      .then(r => r.json())
      .then(data => { updateUI(data); setStatus('Updated ' + new Date().toLocaleTimeString()); })
      .catch(e => setStatus('Error: ' + e));
  }

  function sendCmd(path) {
    fetch(path, { method: 'POST' })
      .then(r => r.json())
      .then(data => { setStatus(path + ' \u2192 ' + JSON.stringify(data)); fetchStatus(); })
      .catch(e => setStatus('Error: ' + e));
  }

  function sendSpeed() {
    const rpm = rpmSlider.value;
    fetch('/speed', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'rpm=' + rpm
    })
      .then(r => r.json())
      .then(data => { setStatus('/speed \u2192 ' + JSON.stringify(data)); fetchStatus(); })
      .catch(e => setStatus('Error: ' + e));
  }

  function saveSettings() {
    const accel     = accelSlider.value;
    const autostart = document.getElementById('autostartToggle').checked ? '1' : '0';
    const ratio     = document.getElementById('gearRatioInput').value;
    const maxRpm    = document.getElementById('maxRpmInput').value;
    const startRpm  = document.getElementById('startRpmInput').value;
    const body = 'accel=' + accel
               + '&autostart=' + autostart
               + '&gearRatio=' + ratio
               + '&maxOutputRpm=' + maxRpm
               + '&startOutputRpm=' + startRpm;
    fetch('/settings', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: body
    })
      .then(r => r.json())
      .then(data => { setStatus('/settings \u2192 ' + JSON.stringify(data)); fetchStatus(); })
      .catch(e => setStatus('Error: ' + e));
  }

  function restoreDefaults() {
    if (!confirm('Reset all settings to factory defaults?')) return;
    fetch('/defaults', { method: 'POST' })
      .then(r => r.json())
      .then(data => { setStatus('Defaults restored \u2192 ' + JSON.stringify(data)); fetchStatus(); })
      .catch(e => setStatus('Error: ' + e));
  }

  fetchStatus();
  setInterval(fetchStatus, 3000);
</script>
</body>
</html>
)rawhtml";

// =============================================================================
// SECTION 8: WEB HANDLERS
// =============================================================================

void handleRoot() {
    server.send(200, "text/html", INDEX_HTML);
}

void handleSpeed() {
    if (server.hasArg("rpm")) {
        float outputRpm = server.arg("rpm").toFloat();
        setSpeed(outputRpm);
        server.send(200, "application/json",
            "{\"ok\":true,\"targetOutputRpm\":" + String(targetOutputRpm, 2) +
            ",\"targetMotorRpm\":"              + String((float)targetMotorRpm, 2) + "}");
    } else {
        server.send(400, "application/json", "{\"ok\":false}");
    }
}

void handleStart() {
    startMotor();
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
    stopMotor();
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleDirection() {
    dirForward = !dirForward;
    if (motorRunning) applyDirection();
    saveSettings();
    server.send(200, "application/json",
        String("{\"ok\":true,\"dir\":\"") + (dirForward ? "fwd" : "rev") + "\"}");
}

void handleSettings() {
    bool changed = false;

    if (server.hasArg("gearRatio")) {
        float r = server.arg("gearRatio").toFloat();
        if (r > 0.0f) {
            gearRatio = r;
            // Recalculate motor target with new ratio
            targetMotorRpm = outputToMotor(targetOutputRpm);
            changed = true;
        }
    }
    if (server.hasArg("maxOutputRpm")) {
        float m = server.arg("maxOutputRpm").toFloat();
        if (m > 0.0f) {
            configMaxOutRpm = m;
            // Clamp current target if it now exceeds the new max
            if (targetOutputRpm > configMaxOutRpm) {
                setSpeed(configMaxOutRpm);
            }
            changed = true;
        }
    }
    if (server.hasArg("accel")) {
        float a = server.arg("accel").toFloat();
        if (a > 0.0f) {
            accelRpmPerSec = constrain(a, 0.1f, 100.0f);
            changed = true;
        }
    }
    if (server.hasArg("startOutputRpm")) {
        float s = server.arg("startOutputRpm").toFloat();
        if (s > 0.0f) {
            startOutputRpm = constrain(s, 0.0f, configMaxOutRpm);
            changed = true;
        }
    }
    if (server.hasArg("autostart")) {
        bootAutoStart = (server.arg("autostart") == "1" || server.arg("autostart") == "true");
        changed = true;
    }

    if (changed) saveSettings();

    server.send(200, "application/json",
        "{\"ok\":true"
        ",\"gearRatio\":"      + String(gearRatio, 4) +
        ",\"maxOutputRpm\":"   + String(configMaxOutRpm, 1) +
        ",\"startOutputRpm\":" + String(startOutputRpm, 1) +
        ",\"accel\":"          + String(accelRpmPerSec, 1) +
        ",\"autostart\":"      + (bootAutoStart ? "true" : "false") + "}");
}

void handleDefaults() {
    // Reset every setting to the compile-time defaults — same state as before
    // acceleration was added. startOutputRpm == targetOutputRpm means the motor
    // launches at full speed instantly, giving maximum torque to overcome inertia.
    targetOutputRpm = DEFAULT_TARGET_RPM;
    startOutputRpm  = DEFAULT_START_RPM;   // = DEFAULT_TARGET_RPM = 30 RPM
    accelRpmPerSec  = DEFAULT_ACCEL;
    gearRatio       = DEFAULT_GEAR_RATIO;
    configMaxOutRpm = DEFAULT_MAX_OUT_RPM;
    bootAutoStart   = true;
    dirForward      = true;
    targetMotorRpm  = outputToMotor(targetOutputRpm);
    saveSettings();
    Serial.println("handleDefaults: all settings reset to factory defaults");
    server.send(200, "application/json",
        "{\"ok\":true"
        ",\"targetOutputRpm\":" + String(targetOutputRpm, 1) +
        ",\"startOutputRpm\":"  + String(startOutputRpm, 1) +
        ",\"accel\":"           + String(accelRpmPerSec, 1) +
        ",\"gearRatio\":"       + String(gearRatio, 4) +
        ",\"maxOutputRpm\":"    + String(configMaxOutRpm, 1) +
        ",\"autostart\":true,\"dir\":\"fwd\"}");
}

void handleStatus() {
    float curOut = motorToOutput(currentMotorRpm);
    String json =
        "{\"outputRpm\":"       + String(curOut, 2) +
        ",\"targetOutputRpm\":" + String(targetOutputRpm, 2) +
        ",\"motorRpm\":"        + String((float)currentMotorRpm, 2) +
        ",\"running\":"         + (motorRunning ? "true" : "false") +
        ",\"dir\":\""           + (dirForward ? "fwd" : "rev") + "\"" +
        ",\"accel\":"           + String(accelRpmPerSec, 1) +
        ",\"autostart\":"       + (bootAutoStart ? "true" : "false") +
        ",\"gearRatio\":"       + String(gearRatio, 4) +
        ",\"maxOutputRpm\":"    + String(configMaxOutRpm, 1) +
        ",\"startOutputRpm\":"  + String(startOutputRpm, 1) +
        "}";
    server.send(200, "application/json", json);
}

// =============================================================================
// SECTION 9: setup()
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("ClockSpinner booting...");

    // Init shift register GPIO pins
    pinMode(SR_BCK,  OUTPUT);
    pinMode(SR_WS,   OUTPUT);
    pinMode(SR_DATA, OUTPUT);
    digitalWrite(SR_BCK,  LOW);
    digitalWrite(SR_WS,   LOW);
    digitalWrite(SR_DATA, LOW);

    // Start with all outputs LOW, then explicitly disable motors
    srState = 0;
    shiftOut32();
    srWrite(BIT_ENABLE, true);
    Serial.print("Shift register init: srState=0x");
    Serial.println(srState, HEX);

    // Load persisted settings
    loadSettings();

    // Start Wi-Fi AP
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    // Register routes and start server
    server.on("/",          HTTP_GET,  handleRoot);
    server.on("/speed",     HTTP_POST, handleSpeed);
    server.on("/start",     HTTP_POST, handleStart);
    server.on("/stop",      HTTP_POST, handleStop);
    server.on("/direction", HTTP_POST, handleDirection);
    server.on("/settings",  HTTP_POST, handleSettings);
    server.on("/defaults",  HTTP_POST, handleDefaults);
    server.on("/status",    HTTP_GET,  handleStatus);
    server.begin();
    Serial.println("Web server started at http://192.168.4.1");

    // Auto-start if configured
    if (bootAutoStart) {
        Serial.println("Auto-start: spinning up...");
        startMotor();
    }
}

// =============================================================================
// SECTION 10: loop()
// =============================================================================

void loop() {
    updateRamp();
    server.handleClient();
    delay(1);
}
