// Main application for Pico W RV control
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <cmath>          // logf/isnan
#include "build_config.h"

// ===================== User-adjustable hardware polarity =====================
static const bool RELAY_ACTIVE_HIGH = true;   // set false if relay inputs are active-low
static const bool LIGHT_ACTIVE_HIGH = true;   // set false if your light driver is active-low
static const bool TEST_ACTIVE_HIGH  = true;   // set false if your +12V test driver enable is active-low
static const bool FURNACE_ACTIVE_HIGH = true; // set false if furnace relay channel is active-low

// ===================== GPIO MAP (per our latest plan) =====================
// Tank level inputs (active-low from PC817 outputs)
static const uint8_t IN_BLACK_1_3   = 2;
static const uint8_t IN_BLACK_2_3   = 3;
static const uint8_t IN_BLACK_FULL  = 4;
static const uint8_t IN_GREY_1_3    = 5;
static const uint8_t IN_GREY_2_3    = 6;
static const uint8_t IN_GREY_FULL   = 7;
static const uint8_t IN_FRESH_1_3   = 8;
static const uint8_t IN_FRESH_2_3   = 9;
static const uint8_t IN_FRESH_FULL  = 10;

// Water heater DSI fault input (active-low from PC817 output)
static const uint8_t IN_HEATER_DSI_FAULT = 11;

// Reserved opto inputs: 12-14, 16-17 (unused here)

// Indoor DS18B20 temperature sensor
static const uint8_t PIN_INDOOR_TEMP_ONEWIRE = 15;

// Awning piggyback relay drives
static const uint8_t OUT_AWNING_EXT = 18;
static const uint8_t OUT_AWNING_RET = 19;

// Furnace piggyback relay drive (spare relay channel)
static const uint8_t OUT_FURNACE_CALL = 20;

// Water heater / pump relays
static const uint8_t OUT_HEATER = 21;
static const uint8_t OUT_PUMP   = 22;

// Tank TEST outputs (+12V injection enables)
static const uint8_t OUT_TEST_BLACK = 23;
static const uint8_t OUT_TEST_GREY  = 24;
static const uint8_t OUT_TEST_FRESH = 25;

// Lights
static const uint8_t OUT_INDOOR_LIGHTS  = 26;
static const uint8_t OUT_OUTDOOR_LIGHTS = 27;

// Thermistor ADC
static const uint8_t PIN_THERM_ADC = 28; // ADC2

// Heater DSI fault retry/lockout
static const uint8_t HEATER_RETRY_MAX = 3;
static const uint32_t HEATER_RETRY_WINDOW_MS = 5UL * 60UL * 1000UL; // 5 minutes
static const uint32_t HEATER_RETRY_OFF_MS = 5000UL; // off time before retry

// GP0/GP1 reserved for UART/I2C later

// Indoor DS18B20 temperature sensor
static const uint8_t INDOOR_TEMP_RESOLUTION_BITS = 12;
static const uint32_t INDOOR_TEMP_READ_INTERVAL_MS = 5000UL;
static const uint32_t INDOOR_TEMP_CONVERSION_MS = 800UL;
OneWire indoorTempWire(PIN_INDOOR_TEMP_ONEWIRE);
DallasTemperature indoorTempSensors(&indoorTempWire);
static uint8_t indoorTempSensorCount = 0;
static bool indoorTempConversionPending = false;
static uint32_t indoorTempRequestMs = 0;
static uint32_t indoorTempLastReadMs = 0;
static float indoorTempF = NAN;

// ===================== Helpers =====================
static inline void writeOut(uint8_t pin, bool on, bool activeHigh) {
  digitalWrite(pin, (on == activeHigh) ? HIGH : LOW);
}
static inline bool readActiveLow(uint8_t pin) {
  return (digitalRead(pin) == LOW);
}

// ===================== Web Server =====================
WebServer server(80);

// ===================== MQTT =====================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
static const uint16_t MQTT_PORT = 1883;

static String topic(const char* suffix) {
  String t = String(MQTT_BASE_TOPIC);
  if (!t.endsWith("/")) t += "/";
  t += suffix;
  return t;
}

static void mqttPublish(const String& t, const String& payload, bool retain=true) {
  if (!mqtt.connected()) return;
  mqtt.publish(t.c_str(), payload.c_str(), retain);
}

static void publishIndoorTemp() {
  if (isnan(indoorTempF)) mqttPublish(topic("indoor/temp_f"), "nan");
  else mqttPublish(topic("indoor/temp_f"), String(indoorTempF, 1));
  mqttPublish(topic("indoor/temp_status"), indoorTempSensorCount > 0 ? "online" : "missing");
}

// ===================== System State =====================
struct State {
  bool heater = false;
  bool pump = false;
  bool indoor_lights = false;
  bool outdoor_lights = false;
  bool heater_dsi_fault = false;

  enum class AwningMode { STOP, EXTEND, RETRACT } awning = AwningMode::STOP;
  uint32_t awningHoldDeadlineMs = 0; // deadman deadline
} st;

static const uint32_t AWNING_DEADMAN_MS = 700;
static const uint16_t AWNING_DEADTIME_MS = 250;

// Furnace thermostat
enum class FurnaceMode { AUTO, MANUAL_OFF, MANUAL_ON };
static FurnaceMode furnaceMode = FurnaceMode::AUTO;

static float furnaceSetpointF = 68.0f;
static float furnaceHysteresisF = 1.0f;
static bool furnaceCall = false;
static uint32_t furnaceLastChangeMs = 0;
static const uint32_t FURNACE_MIN_ON_MS  = 60UL * 1000UL;
static const uint32_t FURNACE_MIN_OFF_MS = 60UL * 1000UL;
static float lastTempF = NAN;

// Heater DSI fault tracking
struct HeaterFault {
  bool lockout = false;
  uint8_t count = 0;
  uint32_t windowStartMs = 0;
  bool retryActive = false;
  uint32_t retryNextMs = 0;
} heaterFault;
static String heaterStatusMsg = "";

// ===================== Apply Outputs =====================
static const char* awningModeStr(State::AwningMode m) {
  switch (m) {
    case State::AwningMode::STOP: return "stop";
    case State::AwningMode::EXTEND: return "extend";
    case State::AwningMode::RETRACT: return "retract";
  }
  return "stop";
}
static const char* furnaceModeStr(FurnaceMode m) {
  switch (m) {
    case FurnaceMode::AUTO: return "AUTO";
    case FurnaceMode::MANUAL_OFF: return "OFF";
    case FurnaceMode::MANUAL_ON: return "ON";
  }
  return "AUTO";
}

static void applyOutputs() {
  writeOut(OUT_HEATER, st.heater, RELAY_ACTIVE_HIGH);
  writeOut(OUT_PUMP, st.pump, RELAY_ACTIVE_HIGH);
  writeOut(OUT_INDOOR_LIGHTS, st.indoor_lights, LIGHT_ACTIVE_HIGH);
  writeOut(OUT_OUTDOOR_LIGHTS, st.outdoor_lights, LIGHT_ACTIVE_HIGH);

  // Tank tests are momentary; default off
  writeOut(OUT_TEST_BLACK, false, TEST_ACTIVE_HIGH);
  writeOut(OUT_TEST_GREY,  false, TEST_ACTIVE_HIGH);
  writeOut(OUT_TEST_FRESH, false, TEST_ACTIVE_HIGH);

  // Awning interlock
  bool ex = (st.awning == State::AwningMode::EXTEND);
  bool rt = (st.awning == State::AwningMode::RETRACT);
  writeOut(OUT_AWNING_EXT, ex && !rt, RELAY_ACTIVE_HIGH);
  writeOut(OUT_AWNING_RET, rt && !ex, RELAY_ACTIVE_HIGH);

  // Furnace call (relay in parallel with thermostat)
  writeOut(OUT_FURNACE_CALL, furnaceCall, FURNACE_ACTIVE_HIGH);
}

// ===================== Awning controls =====================
static void setAwning(State::AwningMode mode) {
  if (mode == State::AwningMode::STOP) {
    st.awning = State::AwningMode::STOP;
    st.awningHoldDeadlineMs = 0;
    applyOutputs();
    return;
  }

  // If reversing direction, stop briefly first to avoid overlap on physical relays
  if (st.awning != State::AwningMode::STOP && st.awning != mode) {
    st.awning = State::AwningMode::STOP;
    applyOutputs();
    delay(AWNING_DEADTIME_MS);
  }

  st.awning = mode;
  st.awningHoldDeadlineMs = millis() + AWNING_DEADMAN_MS;
  applyOutputs();
}

static void refreshAwningHold() {
  if (st.awning != State::AwningMode::STOP) {
    st.awningHoldDeadlineMs = millis() + AWNING_DEADMAN_MS;
  }
}

// ===================== Thermistor (100k NTC) =====================
static float readThermistorF() {
  // Divider: 3.3V -> R_FIXED -> ADC -> NTC -> GND
  const float R_FIXED = 100000.0f;
  const float BETA    = 3950.0f;    // adjust if your NTC differs
  const float T0      = 298.15f;    // 25C
  const float R0      = 100000.0f;  // 100k @ 25C

  int adc = analogRead(PIN_THERM_ADC); // 0..4095
  if (adc <= 0 || adc >= 4095) return NAN;

  float vRatio = (float)adc / 4095.0f;
  float rNTC = (R_FIXED * vRatio) / (1.0f - vRatio);
  if (rNTC < 100.0f || rNTC > 10000000.0f) return NAN;

  float invT = (1.0f / T0) + (1.0f / BETA) * log(rNTC / R0);
  float tempK = 1.0f / invT;
  float tempC = tempK - 273.15f;
  return tempC * 9.0f / 5.0f + 32.0f;
}

static bool canChangeFurnace(bool turningOn) {
  uint32_t now = millis();
  uint32_t dt = now - furnaceLastChangeMs;
  if (turningOn) return dt >= FURNACE_MIN_OFF_MS;
  return dt >= FURNACE_MIN_ON_MS;
}

static void applyFurnace(bool callHeat) {
  if (callHeat != furnaceCall) {
    if (!canChangeFurnace(callHeat)) return;
    furnaceCall = callHeat;
    furnaceLastChangeMs = millis();
    mqttPublish(topic("furnace/state"), furnaceCall ? "ON" : "OFF");
  }
  writeOut(OUT_FURNACE_CALL, furnaceCall, FURNACE_ACTIVE_HIGH);
}

static void furnaceLoop() {
  // Manual override
  if (furnaceMode == FurnaceMode::MANUAL_ON) {
    applyFurnace(true);
    return;
  }
  if (furnaceMode == FurnaceMode::MANUAL_OFF) {
    applyFurnace(false);
    return;
  }

  // AUTO
  float tF = readThermistorF();
  lastTempF = tF;
  if (isnan(tF)) {
    // fail-safe off
    applyFurnace(false);
    return;
  }

  float onThresh  = furnaceSetpointF - furnaceHysteresisF;
  float offThresh = furnaceSetpointF + furnaceHysteresisF;

  if (!furnaceCall && tF <= onThresh) applyFurnace(true);
  else if (furnaceCall && tF >= offThresh) applyFurnace(false);
  else applyFurnace(furnaceCall);
}

// ===================== Tank helper / MQTT publish =====================
static void publishTankStates() {
  mqttPublish(topic("tank/black/one_third"),  readActiveLow(IN_BLACK_1_3) ? "1" : "0");
  mqttPublish(topic("tank/black/two_third"),  readActiveLow(IN_BLACK_2_3) ? "1" : "0");
  mqttPublish(topic("tank/black/full"),       readActiveLow(IN_BLACK_FULL) ? "1" : "0");
  mqttPublish(topic("tank/grey/one_third"),   readActiveLow(IN_GREY_1_3) ? "1" : "0");
  mqttPublish(topic("tank/grey/two_third"),   readActiveLow(IN_GREY_2_3) ? "1" : "0");
  mqttPublish(topic("tank/grey/full"),        readActiveLow(IN_GREY_FULL) ? "1" : "0");
  mqttPublish(topic("tank/fresh/one_third"),  readActiveLow(IN_FRESH_1_3) ? "1" : "0");
  mqttPublish(topic("tank/fresh/two_third"),  readActiveLow(IN_FRESH_2_3) ? "1" : "0");
  mqttPublish(topic("tank/fresh/full"),       readActiveLow(IN_FRESH_FULL) ? "1" : "0");
}

static void publishHeaterDsiFault() {
  mqttPublish(topic("heater/dsi_fault"), readActiveLow(IN_HEATER_DSI_FAULT) ? "1" : "0");
}

static void publishHeaterMsg() {
  mqttPublish(topic("heater/message"), heaterStatusMsg);
}

// ===================== Indoor DS18B20 temperature =====================
static void setupIndoorTempSensor() {
  indoorTempSensors.begin();
  indoorTempSensors.setWaitForConversion(false);
  indoorTempSensorCount = indoorTempSensors.getDeviceCount();

  Serial.print("Indoor DS18B20 sensors found: ");
  Serial.println(indoorTempSensorCount);

  if (indoorTempSensorCount > 0) {
    indoorTempSensors.setResolution(0, INDOOR_TEMP_RESOLUTION_BITS);
    indoorTempSensors.requestTemperatures();
    indoorTempConversionPending = true;
    indoorTempRequestMs = millis();
  }
}

static void indoorTempLoop() {
  uint32_t now = millis();

  if (indoorTempSensorCount == 0) {
    if (now - indoorTempLastReadMs >= 30000UL) {
      indoorTempLastReadMs = now;
      indoorTempSensors.begin();
      indoorTempSensorCount = indoorTempSensors.getDeviceCount();
      if (indoorTempSensorCount > 0) {
        indoorTempSensors.setResolution(0, INDOOR_TEMP_RESOLUTION_BITS);
      }
    }
    indoorTempF = NAN;
    return;
  }

  if (!indoorTempConversionPending && now - indoorTempLastReadMs >= INDOOR_TEMP_READ_INTERVAL_MS) {
    indoorTempSensors.requestTemperatures();
    indoorTempConversionPending = true;
    indoorTempRequestMs = now;
    return;
  }

  if (indoorTempConversionPending && now - indoorTempRequestMs >= INDOOR_TEMP_CONVERSION_MS) {
    float c = indoorTempSensors.getTempCByIndex(0);
    indoorTempF = (c == DEVICE_DISCONNECTED_C) ? NAN : DallasTemperature::toFahrenheit(c);
    indoorTempLastReadMs = now;
    indoorTempConversionPending = false;
    publishIndoorTemp();
  }
}

// ===================== Web UI HTML (embedded) =====================
static const char HOME_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RV Home Control</title>
<style>
:root{--bg:#0b0f14;--card:#101824;--text:#e7eef8;--muted:#9cb2c9;--good:#2bd576;--bad:#ff4d4d;--warn:#ffd24d;--line:#1d2a3a;--btn:#1b2a3d;--btn2:#213449;--shadow:0 10px 30px rgba(0,0,0,.35);--r:18px;--cold:#3aa0ff;--hot:#ff5b5b;}
*{box-sizing:border-box} body{margin:0;font-family:ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,Arial;background:radial-gradient(1200px 800px at 20% 0%,#101a2b 0%,var(--bg) 55%);color:var(--text);}
header{padding:18px 16px 8px;max-width:1100px;margin:0 auto;display:flex;align-items:center;justify-content:space-between;gap:12px;}
.title{display:flex;flex-direction:column;gap:2px;} h1{font-size:18px;margin:0;letter-spacing:.2px;} .sub{margin:0;color:var(--muted);font-size:12px;}
.pill{display:inline-flex;align-items:center;gap:8px;padding:10px 12px;border:1px solid var(--line);background:rgba(16,24,36,.6);border-radius:999px;box-shadow:var(--shadow);user-select:none;}
.dot{width:10px;height:10px;border-radius:50%;background:var(--warn);box-shadow:0 0 0 3px rgba(255,210,77,.15);}
main{max-width:1100px;margin:0 auto;padding:8px 16px 28px;display:grid;grid-template-columns:repeat(12,1fr);gap:12px;}
.card{grid-column:span 6;background:linear-gradient(180deg,rgba(16,24,36,.9),rgba(12,18,28,.9));border:1px solid var(--line);border-radius:var(--r);box-shadow:var(--shadow);padding:14px;min-height:110px;}
.card h2{margin:0 0 10px 0;font-size:14px;color:var(--text);letter-spacing:.2px;display:flex;align-items:center;justify-content:space-between;gap:10px;}
.row{display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap;}
.meta{color:var(--muted);font-size:12px;line-height:1.3;}
.btn{border:1px solid var(--line);background:linear-gradient(180deg,var(--btn2),var(--btn));color:var(--text);padding:10px 12px;border-radius:14px;cursor:pointer;font-weight:600;letter-spacing:.15px;min-width:110px;transition:transform .05s ease,filter .2s ease;}
.btn:active{transform:translateY(1px)} .btn:hover{filter:brightness(1.06)}
.btn.good{border-color:rgba(43,213,118,.35)} .btn.bad{border-color:rgba(255,77,77,.35)} .btn.warn{border-color:rgba(255,210,77,.35)}
.toggle{display:flex;align-items:center;gap:10px;} .switch{width:54px;height:30px;border-radius:999px;background:rgba(156,178,201,.18);border:1px solid var(--line);position:relative;cursor:pointer;flex:0 0 auto;}
.knob{position:absolute;top:3px;left:3px;width:24px;height:24px;border-radius:50%;background:rgba(231,238,248,.9);transition:left .18s ease,background .18s ease;box-shadow:0 8px 16px rgba(0,0,0,.35);}
.switch.on{background:rgba(43,213,118,.18);border-color:rgba(43,213,118,.35)} .switch.on .knob{left:27px;background:rgba(43,213,118,.95)}
.awning{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;width:100%;}
.wide{grid-column:span 12;} .card.awningCard{grid-column:span 12;}
.pillBadge{display:inline-flex;align-items:center;gap:6px;padding:8px 10px;border-radius:12px;border:1px solid var(--line);background:rgba(16,24,36,.6);font-weight:700;font-size:12px;}
.pillBadge.good{border-color:rgba(43,213,118,.35);color:var(--good);}
.pillBadge.bad{border-color:rgba(255,77,77,.35);color:var(--bad);}
.modal{position:fixed;inset:0;display:none;align-items:center;justify-content:center;background:rgba(0,0,0,.6);z-index:20;}
.modal .card{max-width:360px;width:90%;grid-column:span 12;}
.modal h3{margin:0 0 8px 0;}
pre{margin:10px 0 0 0;padding:12px;border-radius:14px;border:1px solid var(--line);background:rgba(9,13,20,.55);color:rgba(231,238,248,.92);overflow:auto;font-size:12px;}
.thermo{display:grid;grid-template-columns:repeat(12,1fr);gap:12px;align-items:flex-start;}
.thermo .tempCol{grid-column:span 4;}
.thermo .setCol{grid-column:span 4;}
.thermo .modeCol{grid-column:span 4;}
.tempLabel{font-size:12px;color:var(--muted);margin-bottom:4px;}
.tempValue{font-size:28px;font-weight:800;letter-spacing:.2px;}
.tempBar{position:relative;height:14px;border-radius:999px;background:linear-gradient(90deg,var(--cold),#73c6ff,#ffd24d,var(--hot));box-shadow:inset 0 0 0 1px rgba(255,255,255,.06);}
.tempMarker{position:absolute;top:-4px;width:4px;height:22px;background:rgba(231,238,248,.9);border-radius:4px;box-shadow:0 6px 16px rgba(0,0,0,.35);}
.setControls{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px;}
.setValue{font-size:22px;font-weight:700;}
.modeButtons{display:flex;gap:8px;flex-wrap:wrap;}
.modeButtons .btn{min-width:70px;}
/* live hold visuals */
.btn.holding{filter:brightness(1.10);box-shadow:0 0 0 3px rgba(43,213,118,.12),0 16px 40px rgba(0,0,0,.35);}
.btn.holding.bad{box-shadow:0 0 0 3px rgba(255,77,77,.12),0 16px 40px rgba(0,0,0,.35);}
.awningStatus{margin-top:10px;padding:10px 12px;border-radius:14px;border:1px solid var(--line);background:rgba(9,13,20,.35);color:var(--muted);display:flex;align-items:center;justify-content:space-between;gap:10px;}
.awningBadge{padding:6px 10px;border-radius:999px;border:1px solid var(--line);background:rgba(16,24,36,.55);color:var(--text);font-weight:700;font-size:12px;}
.awningBadge.good{border-color:rgba(43,213,118,.35)} .awningBadge.bad{border-color:rgba(255,77,77,.35)} .awningBadge.warn{border-color:rgba(255,210,77,.35)}
@media (max-width:900px){.card,.card.awningCard{grid-column:span 12;}}
</style></head><body>
<header><div class="title"><h1>RV Home Control</h1><p class="sub">Pico W • Local</p></div>
<div class="pill"><span class="dot" id="connDot"></span><span id="connText">Connecting…</span>
<button class="btn" style="min-width:auto;padding:8px 10px" onclick="refreshState()">Refresh</button></div></header>

<main>
<section class="card"><h2>Water Heater <span class="meta" id="heaterMeta">—</span></h2>
<div class="row"><div class="toggle"><div class="switch" id="heaterSw" onclick="toggle('heater')"><div class="knob"></div></div>
<div><div style="font-weight:700" id="heaterLabel">OFF</div><div class="meta">Piggyback relay</div></div></div>
<div class="pillBadge bad" id="dsiBadge">DSI FAULT</div>
<button class="btn warn" onclick="toggle('heater')">Toggle</button></div>
<div class="meta" id="heaterMsg">—</div>
</section>

<section class="card"><h2>Water Pump <span class="meta" id="pumpMeta">—</span></h2>
<div class="row"><div class="toggle"><div class="switch" id="pumpSw" onclick="toggle('pump')"><div class="knob"></div></div>
<div><div style="font-weight:700" id="pumpLabel">OFF</div><div class="meta">Piggyback relay</div></div></div>
<button class="btn warn" onclick="toggle('pump')">Toggle</button></div></section>

<section class="card"><h2>Indoor Lights <span class="meta" id="indoorMeta">—</span></h2>
<div class="row"><div class="toggle"><div class="switch" id="indoorSw" onclick="toggle('indoor_lights')"><div class="knob"></div></div>
<div><div style="font-weight:700" id="indoorLabel">OFF</div><div class="meta">Dedicated output</div></div></div>
<button class="btn warn" onclick="toggle('indoor_lights')">Toggle</button></div></section>

<section class="card"><h2>Outdoor Lights <span class="meta" id="outdoorMeta">—</span></h2>
<div class="row"><div class="toggle"><div class="switch" id="outdoorSw" onclick="toggle('outdoor_lights')"><div class="knob"></div></div>
<div><div style="font-weight:700" id="outdoorLabel">OFF</div><div class="meta">Dedicated output</div></div></div>
<button class="btn warn" onclick="toggle('outdoor_lights')">Toggle</button></div></section>

<section class="card"><h2>Indoor Temperature <span class="meta" id="indoorTempMeta">DS18B20</span></h2>
<div class="row"><div><div class="tempValue" id="indoorTempValue">--&deg;F</div><div class="meta">OneWire on GP15</div></div>
<div class="pillBadge warn" id="indoorTempBadge">UNKNOWN</div></div></section>

<section class="card thermoCard">
<h2>Furnace / Thermostat <span class="meta" id="furnaceMeta">--</span></h2>
<div class="thermo">
  <div class="tempCol">
    <div class="tempLabel">Current</div>
    <div class="tempValue" id="tempValue">--°F</div>
    <div class="tempBar">
      <div class="tempMarker" id="tempMarker" style="left:0%"></div>
    </div>
  </div>
  <div class="setCol">
    <div class="tempLabel">Setpoint</div>
    <div class="setValue" id="setValue">--°F</div>
    <div class="setControls">
      <button class="btn" onclick="changeSet(-1)">-1°F</button>
      <button class="btn" onclick="changeSet(-0.5)">-0.5°F</button>
      <button class="btn good" onclick="changeSet(0.5)">+0.5°F</button>
      <button class="btn good" onclick="changeSet(1)">+1°F</button>
    </div>
  </div>
  <div class="modeCol">
    <div class="tempLabel">Mode</div>
    <div class="modeButtons">
      <button class="btn warn" id="modeAuto" onclick="setMode('auto')">Auto</button>
      <button class="btn good" id="modeOn" onclick="setMode('on')">On</button>
      <button class="btn bad" id="modeOff" onclick="setMode('off')">Off</button>
    </div>
    <div class="meta" id="furnaceStatus">Status: --</div>
    <div class="meta" id="furnaceWarn">—</div>
  </div>
</div>
</section>

<section class="card awningCard">
<h2>Awning Control <span class="meta" id="awningMeta">Hold-to-run</span></h2>
<div class="awning">
  <button class="btn good" id="btnExtend">Extend</button>
  <button class="btn warn" id="btnStop" onclick="awningStop()">Stop</button>
  <button class="btn bad" id="btnRetract">Retract</button>
</div>
<div class="awningStatus">
  <div><div style="font-weight:700;color:var(--text)">Awning</div><div class="meta" id="awningHint">Press and hold Extend or Retract</div></div>
  <div class="awningBadge warn" id="awningBadge">STOPPED</div>
</div>
</section>

<section class="card wide">
<h2>System Status <span class="meta">/api/state</span></h2>
<div class="row"><div class="meta">States + temp + MQTT connection.</div><button class="btn" onclick="refreshState()">Refresh Now</button></div>
<pre id="statusBox">{}</pre></section>
</main>

<div class="modal" id="furnaceModal">
  <section class="card">
    <h3>Check LP level</h3>
    <p class="meta" id="furnaceModalText">Please confirm LP levels before starting furnace.</p>
    <div style="display:flex;gap:10px;flex-wrap:wrap;justify-content:flex-end;margin-top:12px;">
      <button class="btn bad" onclick="closeFurnaceModal()">Return</button>
      <button class="btn good" onclick="confirmFurnaceOverride()">Start Furnace Anyway</button>
    </div>
  </section>
</div>

<script>
let holding=null, holdTimer=null;
const ui={connDot:document.getElementById("connDot"),connText:document.getElementById("connText"),statusBox:document.getElementById("statusBox"),
items:{heater:{sw:"heaterSw",label:"heaterLabel"},pump:{sw:"pumpSw",label:"pumpLabel"},indoor_lights:{sw:"indoorSw",label:"indoorLabel"},outdoor_lights:{sw:"outdoorSw",label:"outdoorLabel"}}};

const btnExtend=document.getElementById("btnExtend");
const btnRetract=document.getElementById("btnRetract");
const awningBadge=document.getElementById("awningBadge");
const awningHint=document.getElementById("awningHint");
const tempMarker=document.getElementById("tempMarker");
const tempValue=document.getElementById("tempValue");
const indoorTempValue=document.getElementById("indoorTempValue");
const indoorTempBadge=document.getElementById("indoorTempBadge");
const indoorTempMeta=document.getElementById("indoorTempMeta");
const setValue=document.getElementById("setValue");
const furnaceMeta=document.getElementById("furnaceMeta");
const furnaceStatus=document.getElementById("furnaceStatus");
const modeButtons={auto:document.getElementById("modeAuto"),on:document.getElementById("modeOn"),off:document.getElementById("modeOff")};
const dsiBadge=document.getElementById("dsiBadge");
const heaterMsg=document.getElementById("heaterMsg");
const furnaceWarn=document.getElementById("furnaceWarn");
const furnaceModal=document.getElementById("furnaceModal");
const furnaceModalText=document.getElementById("furnaceModalText");
let heaterLockout=false, heaterDsi=false, pendingMode=null;
let lastSetpoint=null;

function setConn(ok,text){
  ui.connDot.style.background=ok?"var(--good)":"var(--bad)";
  ui.connDot.style.boxShadow=ok?"0 0 0 3px rgba(43,213,118,.15)":"0 0 0 3px rgba(255,77,77,.15)";
  ui.connText.textContent=text;
}
function setSwitch(id,on){const el=document.getElementById(id); if(el) el.classList.toggle("on",!!on);}
function setLabel(id,on){const el=document.getElementById(id); if(el) el.textContent=on?"ON":"OFF";}
function setDsiFault(on){
  if(!dsiBadge) return;
  dsiBadge.classList.toggle("bad",!!on);
  dsiBadge.classList.toggle("good",!on);
  dsiBadge.textContent = on ? "DSI FAULT" : "DSI OK";
}
function setFurnaceWarn(msg){
  if(furnaceWarn) furnaceWarn.textContent = msg || "—";
}
function updateIndoorTemp(temp,ok,count){
  if(indoorTempValue) indoorTempValue.textContent = isFinite(temp) ? `${temp.toFixed(1)}°F` : "--°F";
  if(indoorTempMeta) indoorTempMeta.textContent = `${count || 0} sensor${count===1 ? "" : "s"}`;
  if(indoorTempBadge){
    indoorTempBadge.classList.toggle("good",!!ok);
    indoorTempBadge.classList.toggle("warn",!ok);
    indoorTempBadge.textContent = ok ? "ONLINE" : "MISSING";
  }
}

function setAwningUI(mode){
  btnExtend.classList.remove("holding"); btnRetract.classList.remove("holding","bad");
  btnExtend.textContent="Extend"; btnRetract.textContent="Retract";
  if(mode==="extend"){
    btnExtend.classList.add("holding"); btnExtend.textContent="HOLDING…";
    awningBadge.className="awningBadge good"; awningBadge.textContent="EXTENDING";
    awningHint.textContent="Release to stop";
  }else if(mode==="retract"){
    btnRetract.classList.add("holding","bad"); btnRetract.textContent="HOLDING…";
    awningBadge.className="awningBadge bad"; awningBadge.textContent="RETRACTING";
    awningHint.textContent="Release to stop";
  }else if(mode==="offline"){
    awningBadge.className="awningBadge warn"; awningBadge.textContent="OFFLINE";
    awningHint.textContent="Connection lost (deadman should stop)";
  }else{
    awningBadge.className="awningBadge warn"; awningBadge.textContent="STOPPED";
    awningHint.textContent="Press and hold Extend or Retract";
  }
}

function clamp(v,min,max){ return Math.min(max, Math.max(min,v)); }
function tempToPercent(f){
  if(isNaN(f)) return 0;
  return clamp((f-40)/50*100, 0, 100);
}
function tempToColor(f){
  const pct = tempToPercent(f)/100;
  const lerp=(a,b,t)=>Math.round(a+(b-a)*t);
  const c1=[0x3a,0xa0,0xff], c2=[0xff,0x5b,0x5b];
  const t=clamp(pct,0,1);
  const r=lerp(c1[0],c2[0],t), g=lerp(c1[1],c2[1],t), b=lerp(c1[2],c2[2],t);
  return `rgb(${r},${g},${b})`;
}
function updateThermo(temp,setpoint,mode,calling){
  if(isFinite(temp)){
    tempValue.textContent=`${temp.toFixed(1)}°F`;
    tempMarker.style.left=`${tempToPercent(temp)}%`;
    tempMarker.style.background=tempToColor(temp);
  }else{
    tempValue.textContent="--°F";
  }
  if(isFinite(setpoint)){
    lastSetpoint=setpoint;
    setValue.textContent=`${setpoint.toFixed(1)}°F`;
  }else{
    setValue.textContent="--°F";
  }
  const m=String(mode||"").toUpperCase();
  furnaceMeta.textContent=m||"--";
  furnaceStatus.textContent=`Status: ${calling?"ON":"OFF"}`;
  for(const key of Object.keys(modeButtons)){
    const btn=modeButtons[key];
    if(!btn) continue;
    const active = (key==="auto" && m==="AUTO") || (key==="on" && m==="ON") || (key==="off" && m==="OFF");
    btn.style.filter=active?"brightness(1.12)":"";
    btn.style.boxShadow=active?"0 0 0 2px rgba(255,255,255,.08)":"";
  }
}

async function post(path){
  const r=await fetch(path,{method:"POST"});
  if(!r.ok) throw new Error("HTTP "+r.status);
}
async function setMode(mode){
  if((heaterLockout || heaterDsi) && (mode==="on" || mode==="auto")){
    pendingMode=mode;
    furnaceModal.style.display="flex";
    furnaceModalText.textContent="Please confirm LP levels before starting furnace.";
    return;
  }
  await post(`/api/furnace/mode?mode=${encodeURIComponent(mode)}`);
  await refreshState();
}
async function changeSet(delta){
  const base = isFinite(lastSetpoint) ? lastSetpoint : 68;
  const next = clamp(base + delta, 45, 85);
  await post(`/api/furnace/setpoint?f=${encodeURIComponent(next.toFixed(1))}`);
  lastSetpoint = next;
  await refreshState();
}
function closeFurnaceModal(){
  furnaceModal.style.display="none";
  pendingMode=null;
}
async function confirmFurnaceOverride(){
  if(!pendingMode) { closeFurnaceModal(); return; }
  const mode=pendingMode;
  closeFurnaceModal();
  await post(`/api/furnace/mode?mode=${encodeURIComponent(mode)}`);
  await refreshState();
}
async function refreshState(){
  try{
    const r=await fetch("/api/state",{cache:"no-store"});
    if(!r.ok) throw new Error("HTTP "+r.status);
    const data=await r.json();
    setConn(true,"Connected");
    ui.statusBox.textContent=JSON.stringify(data,null,2);
    for(const key of Object.keys(ui.items)){
      const map=ui.items[key];
      setSwitch(map.sw, !!data[key]);
    setLabel(map.label, !!data[key]);
  }
  if(!holding && data.awning) setAwningUI(data.awning);
  setDsiFault(!!data.heater_dsi_fault);
  heaterMsg.textContent = data.heater_msg || "—";
  heaterLockout = !!data.heater_lockout;
  heaterDsi = !!data.heater_dsi_fault;
  setFurnaceWarn(heaterLockout ? (data.heater_msg || "Check LP level!") : "—");
  updateIndoorTemp(parseFloat(data.indoor_temp_f), !!data.indoor_temp_ok, data.indoor_temp_sensor_count);
  updateThermo(parseFloat(data.temp_f), parseFloat(data.setpoint_f), data.furnace_mode, !!data.furnace_call);
}catch(e){
  setConn(false,"Offline");
  ui.statusBox.textContent="Failed to fetch /api/state\n"+e;
  if(holding) setAwningUI("offline");
  }
}
async function toggle(dev){ await post("/api/toggle/"+encodeURIComponent(dev)); await refreshState(); }

async function awningHold(action){ await post("/api/awning/"+encodeURIComponent(action)); }
async function awningStop(){
  holding=null;
  if(holdTimer){clearInterval(holdTimer); holdTimer=null;}
  try{ await post("/api/awning/stop"); }catch(e){}
  setAwningUI("stop"); refreshState();
}
function startHold(action){
  if(holding===action) return;
  holding=action; setAwningUI(action);
  awningHold(action).catch(()=>setAwningUI("offline"));
  if(holdTimer) clearInterval(holdTimer);
  holdTimer=setInterval(()=>{awningHold(action).catch(()=>setAwningUI("offline"));},250);
}
function endHold(){ awningStop(); }
function bindHold(btn,action){
  btn.addEventListener("mousedown",e=>{e.preventDefault();startHold(action);});
  btn.addEventListener("mouseup",e=>{e.preventDefault();endHold();});
  btn.addEventListener("mouseleave",e=>{e.preventDefault();endHold();});
  btn.addEventListener("touchstart",e=>{e.preventDefault();startHold(action);},{passive:false});
  btn.addEventListener("touchend",e=>{e.preventDefault();endHold();},{passive:false});
  btn.addEventListener("touchcancel",e=>{e.preventDefault();endHold();},{passive:false});
}
bindHold(btnExtend,"extend"); bindHold(btnRetract,"retract"); setAwningUI("stop");
refreshState(); setInterval(refreshState, 4000);
</script></body></html>
)HTML";

// ===================== HTTP handlers =====================
static void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", FPSTR(HOME_HTML));
}

static void handleState() {
  IPAddress ip = WiFi.localIP();
  String json = "{";
  json += "\"heater\":" + String(st.heater ? "true" : "false") + ",";
  json += "\"pump\":" + String(st.pump ? "true" : "false") + ",";
  json += "\"indoor_lights\":" + String(st.indoor_lights ? "true" : "false") + ",";
  json += "\"outdoor_lights\":" + String(st.outdoor_lights ? "true" : "false") + ",";
  json += "\"heater_dsi_fault\":" + String(readActiveLow(IN_HEATER_DSI_FAULT) ? "true" : "false") + ",";
  json += "\"heater_lockout\":" + String(heaterFault.lockout ? "true" : "false") + ",";
  json += "\"heater_msg\":\"" + heaterStatusMsg + "\",";
  json += "\"awning\":\"" + String(awningModeStr(st.awning)) + "\",";
  json += "\"indoor_temp_ok\":" + String(!isnan(indoorTempF) ? "true" : "false") + ",";
  if (isnan(indoorTempF)) json += "\"indoor_temp_f\":null,";
  else json += "\"indoor_temp_f\":" + String(indoorTempF, 1) + ",";
  json += "\"indoor_temp_sensor_count\":" + String(indoorTempSensorCount) + ",";
  json += "\"furnace_call\":" + String(furnaceCall ? "true" : "false") + ",";
  json += "\"furnace_mode\":\"" + String(furnaceModeStr(furnaceMode)) + "\",";
  if (isnan(lastTempF)) json += "\"temp_f\":null,";
  else json += "\"temp_f\":" + String(lastTempF, 1) + ",";
  json += "\"setpoint_f\":" + String(furnaceSetpointF, 1) + ",";
  json += "\"mqtt_connected\":" + String(mqtt.connected() ? "true" : "false") + ",";
  json += "\"ip\":\"" + ip.toString() + "\"";
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

static void handleToggle(const String& which) {
  if (which == "heater") st.heater = !st.heater;
  else if (which == "pump") st.pump = !st.pump;
  else if (which == "indoor_lights") st.indoor_lights = !st.indoor_lights;
  else if (which == "outdoor_lights") st.outdoor_lights = !st.outdoor_lights;
  else { server.send(404, "text/plain", "Unknown device"); return; }

  if (which == "heater" && !st.heater) {
    heaterFault.lockout = false;
    heaterFault.count = 0;
    heaterFault.retryActive = false;
    heaterStatusMsg = "";
  }
  applyOutputs();

  // publish state
  mqttPublish(topic((which + "/state").c_str()), (which == "heater" ? (st.heater?"ON":"OFF") :
                                                 which == "pump" ? (st.pump?"ON":"OFF") :
                                                 which == "indoor_lights" ? (st.indoor_lights?"ON":"OFF") :
                                                 (st.outdoor_lights?"ON":"OFF")));
  server.send(204);
}

static void handleAwning(const String& cmd) {
  if (cmd == "extend") { setAwning(State::AwningMode::EXTEND); refreshAwningHold(); }
  else if (cmd == "retract") { setAwning(State::AwningMode::RETRACT); refreshAwningHold(); }
  else if (cmd == "stop") { setAwning(State::AwningMode::STOP); }
  else { server.send(404, "text/plain", "Unknown awning cmd"); return; }

  mqttPublish(topic("awning/state"), awningModeStr(st.awning));
  server.send(204);
}

static void handleFurnaceModeHttp() {
  String m = server.arg("mode");
  m.toUpperCase();
  if (m == "ON") furnaceMode = FurnaceMode::MANUAL_ON;
  else if (m == "OFF") furnaceMode = FurnaceMode::MANUAL_OFF;
  else furnaceMode = FurnaceMode::AUTO;
  mqttPublish(topic("furnace/mode/state"), furnaceModeStr(furnaceMode));
  server.send(204);
}

static void handleFurnaceSetpointHttp() {
  String s = server.arg("f");
  if (!s.length()) { server.send(400, "text/plain", "missing setpoint"); return; }
  float v = s.toFloat();
  if (v < 45.0f || v > 85.0f) { server.send(400, "text/plain", "setpoint out of range"); return; }
  furnaceSetpointF = v;
  mqttPublish(topic("furnace/setpoint_f"), String(furnaceSetpointF, 1));
  server.send(204);
}

// ===================== Heater DSI retry/lockout =====================
static bool heaterDsiActive() {
  return readActiveLow(IN_HEATER_DSI_FAULT);
}

static void heaterLockout(const char* msg) {
  heaterFault.lockout = true;
  heaterFault.retryActive = false;
  st.heater = false;
  heaterStatusMsg = msg;
  applyOutputs();
  publishHeaterMsg();
}

static void heaterHandleFault() {
  uint32_t now = millis();
  if (heaterFault.windowStartMs == 0 || (now - heaterFault.windowStartMs) > HEATER_RETRY_WINDOW_MS) {
    heaterFault.windowStartMs = now;
    heaterFault.count = 0;
  }
  heaterFault.count++;

  if (heaterFault.count >= HEATER_RETRY_MAX) {
    heaterLockout("Check LP level!");
    return;
  }

  // schedule retry: turn off now, re-enable after delay
  st.heater = false;
  applyOutputs();
  heaterFault.retryActive = true;
  heaterFault.retryNextMs = now + HEATER_RETRY_OFF_MS;
  heaterStatusMsg = String("Retry ") + String(heaterFault.count) + "/" + String(HEATER_RETRY_MAX);
  publishHeaterMsg();
}

static void heaterRetryLoop() {
  if (!heaterFault.retryActive) return;
  if ((int32_t)(millis() - heaterFault.retryNextMs) >= 0) {
    st.heater = true;
    applyOutputs();
    heaterFault.retryActive = false;
    heaterStatusMsg = "";
    publishHeaterMsg();
  }
}

// ===================== MQTT handling =====================
static void publishCoreStates() {
  mqttPublish(topic("status"), "online");
  mqttPublish(topic("heater/state"), st.heater ? "ON" : "OFF");
  mqttPublish(topic("pump/state"), st.pump ? "ON" : "OFF");
  mqttPublish(topic("indoor_lights/state"), st.indoor_lights ? "ON" : "OFF");
  mqttPublish(topic("outdoor_lights/state"), st.outdoor_lights ? "ON" : "OFF");
  mqttPublish(topic("awning/state"), awningModeStr(st.awning));
  publishHeaterDsiFault();
  publishHeaterMsg();
  mqttPublish(topic("furnace/state"), furnaceCall ? "ON" : "OFF");
  mqttPublish(topic("furnace/mode/state"), furnaceModeStr(furnaceMode));
  mqttPublish(topic("furnace/setpoint_f"), String(furnaceSetpointF, 1));
  mqttPublish(topic("furnace/hyst_f"), String(furnaceHysteresisF, 1));
  publishIndoorTemp();
}

static void mqttCallback(char* tpc, byte* payload, unsigned int len) {
  String t = String(tpc);
  String p;
  p.reserve(len+1);
  for (unsigned int i=0;i<len;i++) p += (char)payload[i];
  p.trim();

  auto ends = [&](const char* s){ return t.endsWith(s); };

  if (ends("/heater/set")) {
    st.heater = (p.equalsIgnoreCase("ON") || p=="1" || p.equalsIgnoreCase("TRUE"));
    if (!st.heater) {
      heaterFault.lockout = false;
      heaterFault.count = 0;
      heaterFault.retryActive = false;
      heaterStatusMsg = "";
    }
    applyOutputs();
    mqttPublish(topic("heater/state"), st.heater ? "ON" : "OFF");
  } else if (ends("/pump/set")) {
    st.pump = (p.equalsIgnoreCase("ON") || p=="1" || p.equalsIgnoreCase("TRUE"));
    applyOutputs();
    mqttPublish(topic("pump/state"), st.pump ? "ON" : "OFF");
  } else if (ends("/indoor_lights/set")) {
    st.indoor_lights = (p.equalsIgnoreCase("ON") || p=="1" || p.equalsIgnoreCase("TRUE"));
    applyOutputs();
    mqttPublish(topic("indoor_lights/state"), st.indoor_lights ? "ON" : "OFF");
  } else if (ends("/outdoor_lights/set")) {
    st.outdoor_lights = (p.equalsIgnoreCase("ON") || p=="1" || p.equalsIgnoreCase("TRUE"));
    applyOutputs();
    mqttPublish(topic("outdoor_lights/state"), st.outdoor_lights ? "ON" : "OFF");
  } else if (ends("/awning/set")) {
    if (p.equalsIgnoreCase("EXTEND")) { setAwning(State::AwningMode::EXTEND); refreshAwningHold(); }
    else if (p.equalsIgnoreCase("RETRACT")) { setAwning(State::AwningMode::RETRACT); refreshAwningHold(); }
    else { setAwning(State::AwningMode::STOP); }
    mqttPublish(topic("awning/state"), awningModeStr(st.awning));
  } else if (ends("/furnace/mode/set")) {
    if (p.equalsIgnoreCase("ON")) furnaceMode = FurnaceMode::MANUAL_ON;
    else if (p.equalsIgnoreCase("OFF")) furnaceMode = FurnaceMode::MANUAL_OFF;
    else furnaceMode = FurnaceMode::AUTO;
    mqttPublish(topic("furnace/mode/state"), furnaceModeStr(furnaceMode));
  } else if (ends("/furnace/setpoint_f/set")) {
    float v = p.toFloat();
    if (v >= 40.0f && v <= 90.0f) furnaceSetpointF = v;
    mqttPublish(topic("furnace/setpoint_f"), String(furnaceSetpointF, 1));
  } else if (ends("/furnace/hyst_f/set")) {
    float v = p.toFloat();
    if (v >= 0.5f && v <= 5.0f) furnaceHysteresisF = v;
    mqttPublish(topic("furnace/hyst_f"), String(furnaceHysteresisF, 1));
  }
}

static void mqttReconnectIfNeeded() {
  if (!MQTT_BROKER[0]) return; // disabled if empty
  if (mqtt.connected()) return;

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);

  String clientId = String("picoW-") + String((uint32_t)rp2040.getChipID(), HEX);

  // LWT
  String willTopic = topic("status");
  bool ok;
  if (String(MQTT_USER).length() > 0) {
    ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, willTopic.c_str(), 0, true, "offline");
  } else {
    ok = mqtt.connect(clientId.c_str(), willTopic.c_str(), 0, true, "offline");
  }

  if (ok) {
    mqttPublish(topic("status"), "online");

    // Subscribe to command topics
    mqtt.subscribe(topic("heater/set").c_str());
    mqtt.subscribe(topic("pump/set").c_str());
    mqtt.subscribe(topic("indoor_lights/set").c_str());
    mqtt.subscribe(topic("outdoor_lights/set").c_str());
    mqtt.subscribe(topic("awning/set").c_str());
    mqtt.subscribe(topic("furnace/mode/set").c_str());
    mqtt.subscribe(topic("furnace/setpoint_f/set").c_str());
    mqtt.subscribe(topic("furnace/hyst_f/set").c_str());

    publishCoreStates();
    publishTankStates();
  }
}

// ===================== setup/loop =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Inputs
  pinMode(IN_BLACK_1_3, INPUT);
  pinMode(IN_BLACK_2_3, INPUT);
  pinMode(IN_BLACK_FULL, INPUT);
  pinMode(IN_GREY_1_3, INPUT);
  pinMode(IN_GREY_2_3, INPUT);
  pinMode(IN_GREY_FULL, INPUT);
  pinMode(IN_FRESH_1_3, INPUT);
  pinMode(IN_FRESH_2_3, INPUT);
  pinMode(IN_FRESH_FULL, INPUT);
  pinMode(IN_HEATER_DSI_FAULT, INPUT);

  // Outputs
  pinMode(OUT_AWNING_EXT, OUTPUT);
  pinMode(OUT_AWNING_RET, OUTPUT);
  pinMode(OUT_FURNACE_CALL, OUTPUT);
  pinMode(OUT_HEATER, OUTPUT);
  pinMode(OUT_PUMP, OUTPUT);
  pinMode(OUT_TEST_BLACK, OUTPUT);
  pinMode(OUT_TEST_GREY, OUTPUT);
  pinMode(OUT_TEST_FRESH, OUTPUT);
  pinMode(OUT_INDOOR_LIGHTS, OUTPUT);
  pinMode(OUT_OUTDOOR_LIGHTS, OUTPUT);

  // ADC
  analogReadResolution(12);
  setupIndoorTempSensor();

  applyOutputs();

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
  WiFi.mode(WIFI_STA); // Pico W is STA-only; ESP cores need explicit mode
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - start > 20000) break;
  }
  Serial.println();
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "NOT CONNECTED");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // HTTP routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/toggle/heater", HTTP_POST, [](){ handleToggle("heater"); });
  server.on("/api/toggle/pump", HTTP_POST, [](){ handleToggle("pump"); });
  server.on("/api/toggle/indoor_lights", HTTP_POST, [](){ handleToggle("indoor_lights"); });
  server.on("/api/toggle/outdoor_lights", HTTP_POST, [](){ handleToggle("outdoor_lights"); });

  server.on("/api/awning/extend", HTTP_POST, [](){ handleAwning("extend"); });
  server.on("/api/awning/retract", HTTP_POST, [](){ handleAwning("retract"); });
  server.on("/api/awning/stop", HTTP_POST, [](){ handleAwning("stop"); });
  server.on("/api/furnace/mode", HTTP_POST, handleFurnaceModeHttp);
  server.on("/api/furnace/setpoint", HTTP_POST, handleFurnaceSetpointHttp);

  server.onNotFound([](){ server.send(404, "text/plain", "Not Found"); });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  mqttReconnectIfNeeded();
  mqtt.loop();
  indoorTempLoop();

  // Heater DSI monitoring/retry
  if (st.heater && heaterDsiActive()) {
    if (!heaterFault.lockout) {
      heaterHandleFault();
    } else {
      st.heater = false;
      applyOutputs();
      heaterStatusMsg = "Check LP level!";
    }
  }
  heaterRetryLoop();

  // Deadman awning stop if hold refresh stops
  if (st.awning != State::AwningMode::STOP && st.awningHoldDeadlineMs != 0) {
    if ((int32_t)(millis() - st.awningHoldDeadlineMs) > 0) {
      setAwning(State::AwningMode::STOP);
      mqttPublish(topic("awning/state"), "stop");
    }
  }

  // Furnace thermostat loop
  static uint32_t lastFurnMs = 0;
  if (millis() - lastFurnMs > 1000) {
    lastFurnMs = millis();
    furnaceLoop();

    if (mqtt.connected()) {
      if (!isnan(lastTempF)) mqttPublish(topic("furnace/temp_f"), String(lastTempF, 1));
      mqttPublish(topic("furnace/state"), furnaceCall ? "ON" : "OFF");
    }
  }

  // Periodic tank publish
  static uint32_t lastTankMs = 0;
  if (millis() - lastTankMs > 5000) {
    lastTankMs = millis();
    publishTankStates();
    publishHeaterDsiFault();
    publishIndoorTemp();
  }
}
