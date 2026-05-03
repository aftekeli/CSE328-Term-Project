#include "arduino_secrets.h"
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>

#include <Motor_Vibration_Monitoring_inferencing.h>
#include "thingProperties.h"

// =====================================================================
// ESP32 + ADXL345 + OLED + RGB LED + RELAY + MLX90614 + EDGE IMPULSE
// Arduino Cloud — Final Version
//
// Cloud variables:
//   motorCommand        READWRITE  -> motor ON/OFF control
//   resetFaultCommand   READWRITE  -> FAULT reset command
//   motorRunning        READ       -> actual motor state
//   faultLatchedCloud   READ       -> fault lock status
//   systemState         READ       -> STOP / NORMAL / WARNING / FAULT / SENSOR_ERROR
//   systemStateCode     READ       -> 0=STOP / 1=NORMAL / 2=WARNING / 3=FAULT / 4=SENSOR_ERROR
//   faultReasonCloud    READ       -> NONE / VIB / TEMP / SENS / EI
//   objectTempCloud     READ       -> motor surface temperature
//   anomalyCloud        READ       -> Edge Impulse anomaly score
//   confidenceCloud     READ       -> model confidence percentage (0-100)
//
// Serial commands:
//   1 -> motor ON
//   0 -> motor OFF
//   r -> FAULT reset
//
// Main rules:
//   - When motor is OFF, model output does not change the state; stays STOP.
//   - Vibration FAULT cannot happen when motor is OFF.
//   - After motor starts, FAULT check is disabled for the first 3 seconds.
//   - Vibration FAULT needs anomaly above threshold for 2 consecutive inferences.
//   - Temperature FAULT triggers when objectTempC >= 32.0 C.
//   - objectTempCloud is only written inside readTemperature().
//   - anomalyCloud is only written inside runInference().
//   - confidenceCloud is only written inside runInference().
// =====================================================================


// =====================
// PINS
// =====================
#define SDA_PIN 21
#define SCL_PIN 22

#define RELAY_PIN 32

#define LED_R 25
#define LED_G 26
#define LED_B 27

const bool RELAY_ACTIVE_LOW = true;
const bool COMMON_ANODE = false;


// =====================
// OLED
// =====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledOK = false;

// OLED page swap: Page A (Wi-Fi/cloud/RSSI) and Page B (IP address)
// Switches every 3 seconds
bool oledPageB = false;
unsigned long lastOledPageSwapMs = 0;
const unsigned long OLED_PAGE_SWAP_MS = 3000;


// =====================
// MLX90614 / GY-906
// =====================
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
bool mlxOK = false;

float ambientTempC = 0.0f;
float objectTempC = 0.0f;

// Temperature fault threshold for demo.
// Can be increased to 34.0f if motor heats too fast.
const float OBJECT_TEMP_FAULT_C = 32.0f;

const unsigned long TEMP_READ_INTERVAL_MS = 1000;
unsigned long lastTempReadMs = 0;


// =====================
// ADXL345
// =====================
#define ADXL345_ADDR 0x53

#define REG_DEVID       0x00
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_BW_RATE     0x2C
#define REG_DATAX0      0x32

const uint32_t I2C_CLOCK = 50000;
const unsigned long SAMPLE_INTERVAL_US = 20000; // 50 Hz


// =====================
// EDGE IMPULSE
// =====================
#define AXES 3

const uint32_t EI_INPUT_SIZE = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
const uint32_t WINDOW_SAMPLES = EI_INPUT_SIZE / AXES;

const uint32_t STRIDE_SAMPLES = 10;
const uint32_t STRIDE_VALUES = STRIDE_SAMPLES * AXES;

// Anomaly threshold set to 2.5 for stable demo.
// Reduces false FAULT risk during normal operation.
// Unbalanced propeller will still trigger FAULT.
const float ANOMALY_THRESHOLD = 2.5f;

// FAULT check is disabled for 3 seconds after motor starts.
const unsigned long MOTOR_FAULT_GRACE_MS = 3000UL;

// State transition needs 2 consecutive confirmations.
const uint8_t STATE_CONFIRM_COUNT = 2;

// Vibration fault needs anomaly above threshold for 2 consecutive inferences.
const uint8_t VIB_FAULT_CONFIRM_COUNT = 2;

const uint8_t SENSOR_ERROR_CONFIRM_COUNT = 10;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
uint32_t featureCount = 0;


// =====================
// STATE
// =====================
enum SystemState {
  STATE_STOP,
  STATE_NORMAL,
  STATE_WARNING,
  STATE_FAULT,
  STATE_SENSOR_ERROR
};

enum FaultReason {
  FAULT_REASON_NONE,
  FAULT_REASON_VIBRATION,
  FAULT_REASON_TEMPERATURE,
  FAULT_REASON_SENSOR,
  FAULT_REASON_INFERENCE
};

SystemState currentState = STATE_STOP;
SystemState candidateState = STATE_STOP;
uint8_t candidateCount = 0;

FaultReason faultReason = FAULT_REASON_NONE;

float lastConfidence = 0.0f;
float lastAnomaly = 0.0f;
char lastLabel[24] = "BOOT";

bool motorOn = false;
bool faultLatched = false;

unsigned long motorStartMs = 0;

uint32_t inferenceNo = 0;
uint32_t i2cFailTotal = 0;
uint8_t i2cFailStreak = 0;
uint8_t vibrationFaultCount = 0;

unsigned long lastSampleUs = 0;
unsigned long lastOledUpdateMs = 0;
unsigned long lastCloudSyncMs = 0;


// =====================================================================
// TEXT HELPERS
// =====================================================================
const char* stateName(SystemState state) {
  switch (state) {
    case STATE_STOP:         return "STOP";
    case STATE_NORMAL:       return "NORMAL";
    case STATE_WARNING:      return "WARNING";
    case STATE_FAULT:        return "FAULT";
    case STATE_SENSOR_ERROR: return "SENSOR_ERROR";
    default:                 return "UNKNOWN";
  }
}

const char* shortStateName(SystemState state) {
  switch (state) {
    case STATE_STOP:         return "STOP";
    case STATE_NORMAL:       return "NORMAL";
    case STATE_WARNING:      return "WARNING";
    case STATE_FAULT:        return "FAULT";
    case STATE_SENSOR_ERROR: return "SENSOR";
    default:                 return "UNKNOWN";
  }
}

const char* faultReasonName(FaultReason reason) {
  switch (reason) {
    case FAULT_REASON_VIBRATION:   return "VIB";
    case FAULT_REASON_TEMPERATURE: return "TEMP";
    case FAULT_REASON_SENSOR:      return "SENS";
    case FAULT_REASON_INFERENCE:   return "EI";
    case FAULT_REASON_NONE:
    default:                       return "NONE";
  }
}

int stateToCode(SystemState state) {
  switch (state) {
    case STATE_STOP:         return 0;
    case STATE_NORMAL:       return 1;
    case STATE_WARNING:      return 2;
    case STATE_FAULT:        return 3;
    case STATE_SENSOR_ERROR: return 4;
    default:                 return -1;
  }
}


// =====================================================================
// CLOUD STATUS HELPER
// =====================================================================
bool cloudConnected() {
  return ArduinoCloud.connected();
}


// =====================================================================
// CLOUD SYNC
// objectTempCloud -> updated in readTemperature()
// anomalyCloud    -> updated in runInference()
// confidenceCloud -> updated in runInference()
// =====================================================================
void syncCloudNow() {
  motorRunning = motorOn;
  faultLatchedCloud = faultLatched;
  systemState = stateName(currentState);
  faultReasonCloud = faultReasonName(faultReason);
  systemStateCode = stateToCode(currentState);
  confidenceCloud = lastConfidence * 100.0f;
}

void syncCloudPeriodic() {
  unsigned long nowMs = millis();

  if (nowMs - lastCloudSyncMs >= 1000) {
    lastCloudSyncMs = nowMs;
    syncCloudNow();
  }
}


// =====================================================================
// MOTOR / RELAY
// =====================================================================
void setRelayRaw(bool on) {
  motorOn = on;

  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  }

  motorRunning = motorOn;
}

bool motorInGracePeriod() {
  if (!motorOn) return false;
  return (millis() - motorStartMs) < MOTOR_FAULT_GRACE_MS;
}

unsigned long remainingGraceMs() {
  if (!motorInGracePeriod()) return 0;

  unsigned long elapsed = millis() - motorStartMs;
  if (elapsed >= MOTOR_FAULT_GRACE_MS) return 0;

  return MOTOR_FAULT_GRACE_MS - elapsed;
}

void motorStart() {
  if (faultLatched) {
    setRelayRaw(false);
    motorCommand = false;
    syncCloudNow();
    return;
  }

  if (currentState == STATE_SENSOR_ERROR) {
    setRelayRaw(false);
    motorCommand = false;
    syncCloudNow();
    return;
  }

  vibrationFaultCount = 0;
  motorStartMs = millis();
  setRelayRaw(true);

  syncCloudNow();
}

void motorStop() {
  setRelayRaw(false);
  motorCommand = false;

  // When motor is OFF, set state to STOP (unless fault is latched).
  if (!faultLatched) {
    currentState = STATE_STOP;
    candidateState = STATE_STOP;
    candidateCount = STATE_CONFIRM_COUNT;
    vibrationFaultCount = 0;
  }

  syncCloudNow();
}

void latchFault(FaultReason reason) {
  faultLatched = true;
  faultReason = reason;

  setRelayRaw(false);
  motorCommand = false;

  currentState = STATE_FAULT;
  candidateState = STATE_FAULT;
  candidateCount = STATE_CONFIRM_COUNT;
  vibrationFaultCount = 0;

  syncCloudNow();
}

void resetFault() {
  faultLatched = false;
  faultReason = FAULT_REASON_NONE;

  setRelayRaw(false);
  motorCommand = false;

  currentState = STATE_STOP;
  candidateState = STATE_STOP;
  candidateCount = STATE_CONFIRM_COUNT;
  vibrationFaultCount = 0;

  strcpy(lastLabel, "RESET");
  lastConfidence = 0.0f;
  lastAnomaly = 0.0f;

  syncCloudNow();
}


// =====================================================================
// RGB LED
// =====================================================================
void writeLedPin(uint8_t pin, bool on) {
  if (COMMON_ANODE) {
    digitalWrite(pin, on ? LOW : HIGH);
  } else {
    digitalWrite(pin, on ? HIGH : LOW);
  }
}

void setColor(bool r, bool g, bool b) {
  writeLedPin(LED_R, r);
  writeLedPin(LED_G, g);
  writeLedPin(LED_B, b);
}

void applyColor() {
  if (faultLatched || currentState == STATE_FAULT) {
    setColor(true, false, false);     // FAULT -> red
    return;
  }

  switch (currentState) {
    case STATE_STOP:
      setColor(false, true, false);   // STOP -> green
      break;

    case STATE_NORMAL:
      setColor(false, false, true);   // NORMAL -> blue
      break;

    case STATE_WARNING:
      setColor(true, false, true);    // WARNING -> magenta
      break;

    case STATE_SENSOR_ERROR:
      setColor(true, true, true);     // SENSOR_ERROR -> white
      break;

    default:
      setColor(true, false, false);
      break;
  }
}


// =====================================================================
// I2C / ADXL345
// =====================================================================
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)1) != 1) return false;

  value = Wire.read();
  return true;
}

bool readXYZ(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(REG_DATAX0);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)6) != 6) return false;

  uint8_t x0 = Wire.read();
  uint8_t x1 = Wire.read();
  uint8_t y0 = Wire.read();
  uint8_t y1 = Wire.read();
  uint8_t z0 = Wire.read();
  uint8_t z1 = Wire.read();

  x = (int16_t)((x1 << 8) | x0);
  y = (int16_t)((y1 << 8) | y0);
  z = (int16_t)((z1 << 8) | z0);

  return true;
}

void i2cRecover() {
  Wire.end();
  delay(20);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(20);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeOut(50);

  delay(20);
}

bool initADXL345() {
  uint8_t devid = 0;

  if (!readRegister(REG_DEVID, devid)) return false;
  if (devid != 0xE5) return false;

  if (!writeRegister(REG_POWER_CTL, 0x00)) return false;  // Standby
  delay(10);

  if (!writeRegister(REG_DATA_FORMAT, 0x0B)) return false; // FULL_RES + +/-16g
  delay(10);

  if (!writeRegister(REG_BW_RATE, 0x0A)) return false;    // 100 Hz data rate
  delay(10);

  if (!writeRegister(REG_POWER_CTL, 0x08)) return false;  // Measure mode
  delay(10);

  return true;
}


// =====================================================================
// MLX90614
// =====================================================================
bool initMLX90614() {
  return mlx.begin();
}

void checkTemperatureFault() {
  if (!mlxOK) return;
  if (!motorOn) return;
  if (faultLatched) return;
  if (motorInGracePeriod()) return;

  if (objectTempC >= OBJECT_TEMP_FAULT_C) {
    latchFault(FAULT_REASON_TEMPERATURE);
    applyColor();
    updateOLED();
  }
}

// Simple temperature reading without filtering.
// objectTempCloud is only updated here.
void readTemperature() {
  unsigned long nowMs = millis();

  if (nowMs - lastTempReadMs < TEMP_READ_INTERVAL_MS) {
    return;
  }

  lastTempReadMs = nowMs;

  if (!mlxOK) {
    return;
  }

  ambientTempC = mlx.readAmbientTempC();
  objectTempC = mlx.readObjectTempC();

  objectTempCloud = objectTempC;

  checkTemperatureFault();
}


// =====================================================================
// OLED — Dual Page Display
//
// Both pages share lines 1-3:
//   Line 1 (size 2): State name (STOP / NORMAL / WARNING)
//   Line 2 (size 1): Temp + Anomaly score
//   Line 3 (size 1): Confidence + Motor state
//
// Line 4 swaps every 3 seconds:
//   Page A: Wi:OK Cl:OK RSSI:-29
//   Page B: IP:192.168.1.171
//
// FAULT screen: full-screen warning, no page swap.
// =====================================================================

// Helper: draw the common top 3 lines (used by both pages)
void drawOledCommonLines() {
  // Line 1: state name in large text
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(shortStateName(currentState));

  // Line 2: temperature + anomaly
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Temp:");
  display.print(objectTempC, 1);
  display.print("C");

  display.setCursor(74, 20);
  display.print("Anom:");
  display.print(lastAnomaly, 2);

  // Line 3: confidence + motor state
  display.setCursor(0, 34);
  display.print("Conf:");
  display.print((int)(lastConfidence * 100.0f));
  display.print("%");

  display.setCursor(74, 34);
  display.print("Motor:");
  display.print(motorOn ? "ON" : "OFF");
}

void updateOLED() {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // ---------------------------------
  // FAULT / SENSOR ERROR — full screen
  // ---------------------------------
  if (faultLatched || currentState == STATE_FAULT || currentState == STATE_SENSOR_ERROR) {

    display.setTextSize(2);
    display.setCursor(0, 0);

    if (currentState == STATE_SENSOR_ERROR) {
      display.print("SENSOR!!");
    } else {
      display.print("!!");
      display.setCursor(24, 0);
      display.print("MOTOR");
    }

    display.setCursor(0, 18);

    if (currentState == STATE_SENSOR_ERROR) {
      display.setTextSize(1);
      display.print("I2C / SENSOR ERROR");
    } else {
      display.print("STOPPED!");
    }

    display.setTextSize(1);
    display.setCursor(0, 38);
    display.print("Reason:");
    display.print(faultReasonName(faultReason));

    display.setCursor(70, 38);
    display.print("T:");
    display.print(objectTempC, 1);
    display.print("C");

    display.setCursor(0, 52);
    display.print("Anom:");
    display.print(lastAnomaly, 1);

    display.setCursor(70, 52);
    display.print("Reset:'r'");

    display.display();
    return;
  }

  // ---------------------------------
  // NORMAL / WARNING / STOP — dual page
  // ---------------------------------

  // Handle page swap timer
  unsigned long nowMs = millis();
  if (nowMs - lastOledPageSwapMs >= OLED_PAGE_SWAP_MS) {
    lastOledPageSwapMs = nowMs;
    oledPageB = !oledPageB;
  }

  // Draw common lines 1-3
  drawOledCommonLines();

  // Line 4: depends on current page
  display.setCursor(0, 48);

  if (!oledPageB) {
    // PAGE A: WiFi + Cloud + RSSI
    display.print("Wi:");
    if (WiFi.status() == WL_CONNECTED) {
      display.print("OK ");
    } else {
      display.print("-- ");
    }

    display.print("Cl:");
    if (cloudConnected()) {
      display.print("OK ");
    } else {
      display.print("-- ");
    }

    if (WiFi.status() == WL_CONNECTED) {
      display.print("RSSI:");
      display.print(WiFi.RSSI());
    }

  } else {
    // PAGE B: Full IP address
    if (WiFi.status() == WL_CONNECTED) {
      display.print("IP:");
      display.print(WiFi.localIP());
    } else {
      display.print("IP:Not connected");
    }
  }

  display.display();
}


// =====================================================================
// SERIAL COMMANDS
// =====================================================================
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r' || c == ' ' || c == '\t') {
      continue;
    }

    if (c == '1') {
      motorCommand = true;
      motorStart();
      updateOLED();
    }
    else if (c == '0') {
      motorStop();
      updateOLED();
    }
    else if (c == 'r' || c == 'R') {
      resetFault();
      resetFaultCommand = false;
      applyColor();
      updateOLED();
    }
  }
}


// =====================================================================
// EDGE IMPULSE CALLBACK
// =====================================================================
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

SystemState labelToState(const char* label) {
  if (strcmp(label, "STOP") == 0)    return STATE_STOP;
  if (strcmp(label, "NORMAL") == 0)  return STATE_NORMAL;
  if (strcmp(label, "WARNING") == 0) return STATE_WARNING;
  return STATE_STOP;
}

void updateStableState(SystemState detectedState) {
  if (faultLatched) {
    currentState = STATE_FAULT;
    setRelayRaw(false);
    applyColor();
    syncCloudNow();
    return;
  }

  // When motor is OFF, do not update state from model output.
  if (!motorOn) {
    currentState = STATE_STOP;
    candidateState = STATE_STOP;
    candidateCount = STATE_CONFIRM_COUNT;
    applyColor();
    syncCloudNow();
    return;
  }

  if (detectedState == candidateState) {
    if (candidateCount < 255) {
      candidateCount++;
    }
  } else {
    candidateState = detectedState;
    candidateCount = 1;
  }

  if (candidateCount >= STATE_CONFIRM_COUNT && currentState != candidateState) {
    currentState = candidateState;

    if (currentState == STATE_FAULT) {
      latchFault(FAULT_REASON_VIBRATION);
    } else if (currentState == STATE_SENSOR_ERROR) {
      latchFault(FAULT_REASON_SENSOR);
    }

    applyColor();
    syncCloudNow();
    updateOLED();
  }
}

void runInference() {
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &raw_feature_get_data;

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  inferenceNo++;

  if (res != EI_IMPULSE_OK) {
    strcpy(lastLabel, "EI_ERROR");
    lastConfidence = 0.0f;
    lastAnomaly = 0.0f;

    latchFault(FAULT_REASON_INFERENCE);
    applyColor();
    updateOLED();
    return;
  }

  float bestValue = 0.0f;
  const char* bestLabel = "UNKNOWN";

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    float value = result.classification[ix].value;

    if (value > bestValue) {
      bestValue = value;
      bestLabel = result.classification[ix].label;
    }
  }

  strncpy(lastLabel, bestLabel, sizeof(lastLabel) - 1);
  lastLabel[sizeof(lastLabel) - 1] = '\0';
  lastConfidence = bestValue;

#if EI_CLASSIFIER_HAS_ANOMALY != EI_ANOMALY_TYPE_UNKNOWN
  lastAnomaly = result.anomaly;
#else
  lastAnomaly = 0.0f;
#endif

  // anomalyCloud and confidenceCloud are only updated here.
  anomalyCloud = lastAnomaly;
  confidenceCloud = lastConfidence * 100.0f;

  SystemState detectedState = labelToState(bestLabel);

  // When motor is OFF, ignore model output.
  if (!motorOn) {
    detectedState = STATE_STOP;
    vibrationFaultCount = 0;
  }

  // Vibration FAULT conditions:
  // - Motor is ON
  // - Grace period is over
  // - Anomaly is above threshold
  // - Confirmed for 2 consecutive inferences
#if EI_CLASSIFIER_HAS_ANOMALY != EI_ANOMALY_TYPE_UNKNOWN
  if (motorOn && !motorInGracePeriod() && result.anomaly > ANOMALY_THRESHOLD) {
    if (vibrationFaultCount < 255) {
      vibrationFaultCount++;
    }

    if (vibrationFaultCount >= VIB_FAULT_CONFIRM_COUNT) {
      detectedState = STATE_FAULT;
    }
  } else {
    vibrationFaultCount = 0;
  }
#endif

  updateStableState(detectedState);

  if (faultLatched) {
    setRelayRaw(false);
  }

  unsigned long nowMs = millis();
  if (nowMs - lastOledUpdateMs >= 500) {
    lastOledUpdateMs = nowMs;
    updateOLED();
  }
}


// =====================================================================
// FEATURE BUFFER
// =====================================================================
void addSampleToFeatureBuffer(int16_t x, int16_t y, int16_t z) {
  if (featureCount + AXES > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    return;
  }

  features[featureCount++] = (float)x;
  features[featureCount++] = (float)y;
  features[featureCount++] = (float)z;
}

void slideWindowAfterInference() {
  if (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE <= STRIDE_VALUES) {
    featureCount = 0;
    return;
  }

  uint32_t remaining = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - STRIDE_VALUES;

  memmove(features, features + STRIDE_VALUES, remaining * sizeof(float));
  featureCount = remaining;
}


// =====================================================================
// ARDUINO CLOUD CALLBACKS
// =====================================================================
void onMotorCommandChange() {
  if (motorCommand) {
    motorStart();
  } else {
    motorStop();
  }

  applyColor();
  updateOLED();
  syncCloudNow();
}

void onResetFaultCommandChange() {
  if (resetFaultCommand) {
    resetFault();

    // Reset the flag if it stays true (push button behavior).
    resetFaultCommand = false;

    applyColor();
    updateOLED();
    syncCloudNow();
  }
}


// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(9600);
  delay(1500);

  pinMode(RELAY_PIN, OUTPUT);
  setRelayRaw(false);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  faultLatched = false;
  faultReason = FAULT_REASON_NONE;

  currentState = STATE_STOP;
  candidateState = STATE_STOP;
  candidateCount = STATE_CONFIRM_COUNT;

  applyColor();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeOut(50);

  delay(100);

  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  mlxOK = initMLX90614();

  if (!initADXL345()) {
    currentState = STATE_SENSOR_ERROR;
    faultLatched = true;
    faultReason = FAULT_REASON_SENSOR;

    setRelayRaw(false);

    strcpy(lastLabel, "ADXL_ERROR");
    lastConfidence = 0.0f;
    lastAnomaly = 0.0f;

    applyColor();
    updateOLED();
  } else {
    currentState = STATE_STOP;
    candidateState = STATE_STOP;
    candidateCount = STATE_CONFIRM_COUNT;

    faultLatched = false;
    faultReason = FAULT_REASON_NONE;

    setRelayRaw(false);

    strcpy(lastLabel, "BOOT");
    lastConfidence = 0.0f;
    lastAnomaly = 0.0f;

    applyColor();
    updateOLED();
  }

  featureCount = 0;
  lastSampleUs = micros();

  // Arduino Cloud
  initProperties();

  motorCommand = false;
  resetFaultCommand = false;
  motorRunning = false;
  faultLatchedCloud = faultLatched;
  systemState = stateName(currentState);
  faultReasonCloud = faultReasonName(faultReason);

  // Write initial local values to Cloud.
  systemStateCode = stateToCode(currentState);
  confidenceCloud = 0.0f;
  objectTempCloud = objectTempC;
  anomalyCloud = lastAnomaly;

  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  syncCloudNow();
  updateOLED();
}


// =====================================================================
// LOOP
// =====================================================================
void loop() {
  ArduinoCloud.update();

  handleSerialCommands();

  readTemperature();

  handleInferenceSampling();

  syncCloudPeriodic();
}


// =====================================================================
// MAIN SAMPLING LOOP
// =====================================================================
void handleInferenceSampling() {
  unsigned long nowUs = micros();

  if ((unsigned long)(nowUs - lastSampleUs) < SAMPLE_INTERVAL_US) {
    return;
  }

  lastSampleUs += SAMPLE_INTERVAL_US;

  int16_t x, y, z;

  if (!readXYZ(x, y, z)) {
    i2cFailTotal++;

    if (i2cFailStreak < 255) {
      i2cFailStreak++;
    }

    if (i2cFailStreak >= SENSOR_ERROR_CONFIRM_COUNT) {
      strcpy(lastLabel, "I2C_ERROR");
      lastConfidence = 0.0f;
      lastAnomaly = 0.0f;

      currentState = STATE_SENSOR_ERROR;
      latchFault(FAULT_REASON_SENSOR);

      applyColor();
      updateOLED();
    }

    if (i2cFailTotal % 5 == 0) {
      i2cRecover();
      initADXL345();
      mlxOK = initMLX90614();
    }

    return;
  }

  i2cFailStreak = 0;

  addSampleToFeatureBuffer(x, y, z);

  if (featureCount >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    runInference();
    slideWindowAfterInference();
  }
}
