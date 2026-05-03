#include "arduino_secrets.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>

#include <Motor_Vibration_Monitoring_inferencing.h>
#include "thingProperties.h"

// =====================================================================
// ESP32 + ADXL345 + OLED + RGB LED + RELAY + MLX90614 + EDGE IMPULSE
// Arduino Cloud final demo sürümü
//
// Cloud variables:
//   motorCommand        READWRITE  -> motor ON/OFF
//   resetFaultCommand   READWRITE  -> FAULT reset
//   motorRunning        READ       -> motor gerçek durumu
//   faultLatchedCloud   READ       -> fault kilidi
//   systemState         READ       -> STOP / NORMAL / WARNING / FAULT / SENSOR_ERROR
//   systemStateCode     READ       -> 0=STOP / 1=NORMAL / 2=WARNING / 3=FAULT / 4=SENSOR_ERROR (chart için)
//   faultReasonCloud    READ       -> NONE / VIB / TEMP / SENS / EI
//   objectTempCloud     READ       -> motor gövde sıcaklığı
//   anomalyCloud        READ       -> Edge Impulse anomaly skoru
//   confidenceCloud     READ       -> Model confidence yüzdesi (0-100)
//
// Lokal serial komutlar:
//   1 -> motor ON
//   0 -> motor OFF
//   r -> FAULT reset
//
// Kritik mantık:
//   - Motor OFF iken model sonucu state'i değiştirmez; state STOP kalır.
//   - Motor OFF iken vibration FAULT oluşmaz.
//   - Motor ON olduktan sonra ilk 3 saniye FAULT kontrolü yapılmaz.
//   - Vibration FAULT için anomaly üst üste 2 inference eşiği geçmelidir.
//   - Sıcaklık FAULT için objectTempC >= 32.0 C yeterlidir.
//   - objectTempCloud sadece readTemperature() içinde yazılır.
//   - anomalyCloud sadece runInference() içinde yazılır.
//   - confidenceCloud sadece runInference() içinde yazılır.
// =====================================================================


// =====================
// PINLER
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


// =====================
// MLX90614 / GY-906
// =====================
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
bool mlxOK = false;

float ambientTempC = 0.0f;
float objectTempC = 0.0f;

// Demo eşiği.
// Çok erken kapanırsa 34.0f yapılabilir.
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

// Kararlı demo için 2.5.
// Normal çalışmada gereksiz FAULT riskini azaltır.
// Aşırı dengesiz pervanede yine FAULT'a düşer.
const float ANOMALY_THRESHOLD = 2.5f;

// Motor çalıştıktan sonra ilk 3 saniye FAULT kontrolü kapalı.
const unsigned long MOTOR_FAULT_GRACE_MS = 3000UL;

// State geçiş stabilizasyonu.
const uint8_t STATE_CONFIRM_COUNT = 2;

// Vibration fault için üst üste kaç inference anomaly eşiğini geçmeli.
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
    case STATE_STOP:
      return "STOP";
    case STATE_NORMAL:
      return "NORMAL";
    case STATE_WARNING:
      return "WARNING";
    case STATE_FAULT:
      return "FAULT";
    case STATE_SENSOR_ERROR:
      return "SENSOR_ERROR";
    default:
      return "UNKNOWN";
  }
}

const char* shortStateName(SystemState state) {
  switch (state) {
    case STATE_STOP:
      return "STOP";
    case STATE_NORMAL:
      return "NORMAL";
    case STATE_WARNING:
      return "WARNING";
    case STATE_FAULT:
      return "FAULT";
    case STATE_SENSOR_ERROR:
      return "SENSOR";
    default:
      return "UNKNOWN";
  }
}

const char* faultReasonName(FaultReason reason) {
  switch (reason) {
    case FAULT_REASON_VIBRATION:
      return "VIB";
    case FAULT_REASON_TEMPERATURE:
      return "TEMP";
    case FAULT_REASON_SENSOR:
      return "SENS";
    case FAULT_REASON_INFERENCE:
      return "EI";
    case FAULT_REASON_NONE:
    default:
      return "NONE";
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
// Bu fonksiyon yalnızca durum/komut değişkenlerini yazar.
// objectTempCloud -> readTemperature()
// anomalyCloud    -> runInference()
// confidenceCloud -> runInference()
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

  // Motor OFF iken state'i bilinçli olarak STOP'a çekiyoruz.
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
    setColor(true, false, false);     // FAULT -> kırmızı
    return;
  }

  switch (currentState) {
    case STATE_STOP:
      setColor(false, true, false);   // STOP -> yeşil
      break;

    case STATE_NORMAL:
      setColor(false, false, true);   // NORMAL -> mavi
      break;

    case STATE_WARNING:
      setColor(true, false, true);    // WARNING -> mor / magenta
      break;

    case STATE_SENSOR_ERROR:
      setColor(true, true, true);     // SENSOR_ERROR -> beyaz
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

  // Standby
  if (!writeRegister(REG_POWER_CTL, 0x00)) return false;
  delay(10);

  // FULL_RES + +/-16g
  if (!writeRegister(REG_DATA_FORMAT, 0x0B)) return false;
  delay(10);

  // 100 Hz internal data rate
  if (!writeRegister(REG_BW_RATE, 0x0A)) return false;
  delay(10);

  // Measure mode
  if (!writeRegister(REG_POWER_CTL, 0x08)) return false;
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

// Gece düzgün çalışan sade sıcaklık mantığı.
// Filtreleme yok.
// objectTempCloud sadece burada güncellenir.
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
// OLED  —  Proposal Layout
// Line 1: Inference result (state) + confidence score
// Line 2: K-Means anomaly score
// Line 3: Motor surface temperature + bar indicator
// Line 4: Relay (motor) state + WiFi/EI + lock/grace
// FAULT : Full-screen "!! MOTOR STOPPED !!" uyarısı
// =====================================================================
void updateOLED() {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // -----------------------------
  // FAULT / SENSOR ERROR SCREEN
  // Proposal: full-screen "!! MOTOR STOPPED !!"
  // -----------------------------
  if (faultLatched || currentState == STATE_FAULT || currentState == STATE_SENSOR_ERROR) {

    // Satır 1 — büyük başlık
    display.setTextSize(2);
    display.setCursor(0, 0);

    if (currentState == STATE_SENSOR_ERROR) {
      display.print("SENSOR!!");
    } else {
      display.print("!!");
      display.setCursor(24, 0);
      display.print("MOTOR");
    }

    // Satır 2 — alt başlık
    display.setCursor(0, 18);

    if (currentState == STATE_SENSOR_ERROR) {
      display.setTextSize(1);
      display.print("I2C / SENSOR ERROR");
    } else {
      display.print("STOPPED!");
    }

    // Satır 3 — sebep
    display.setTextSize(1);
    display.setCursor(0, 38);
    display.print("Reason:");
    display.print(faultReasonName(faultReason));

    display.setCursor(70, 38);
    display.print("T:");
    display.print(objectTempC, 1);
    display.print("C");

    // Satır 4 — anomaly + reset ipucu
    display.setCursor(0, 52);
    display.print("Anom:");
    display.print(lastAnomaly, 1);

    display.setCursor(70, 52);
    display.print("Reset:'r'");

    display.display();
    return;
  }

  // -----------------------------
  // NORMAL / WARNING / STOP SCREEN
  // -----------------------------
  display.setTextSize(1);

  // Line 1: inference result + confidence
  display.setCursor(0, 0);

  if (currentState == STATE_WARNING) {
    display.print("! ");
  } else if (currentState == STATE_NORMAL) {
    display.print("> ");
  } else if (currentState == STATE_STOP) {
    display.print("S ");
  } else {
    display.print("* ");
  }

  display.print(shortStateName(currentState));

  display.setCursor(74, 0);
  display.print("Conf:");
  display.print((int)(lastConfidence * 100.0f));
  display.print("%");

  // Line 2: K-Means anomaly score
  display.setCursor(0, 16);
  display.print("Anomaly:");
  display.print(lastAnomaly, 2);

  // Line 3: motor surface temperature + bar indicator
  display.setCursor(0, 32);
  display.print("Temp:");
  display.print(objectTempC, 1);
  display.print("C");

  int barX = 78;
  int barY = 33;
  int barW = 48;
  int barH = 7;

  display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);

  float tempForBar = objectTempC;
  if (tempForBar < 25.0f) tempForBar = 25.0f;
  if (tempForBar > 45.0f) tempForBar = 45.0f;

  int fillW = (int)((tempForBar - 25.0f) * (barW - 2) / 20.0f);
  if (fillW < 0) fillW = 0;
  if (fillW > barW - 2) fillW = barW - 2;

  display.fillRect(barX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);

  // Line 4: relay state + WiFi/EI + lock/grace
  display.setCursor(0, 48);
  display.print("Motor:");
  display.print(motorOn ? "ON " : "OFF");

  display.setCursor(52, 48);
  display.print("Wi:");

  if (cloudConnected()) {
    display.print("OK");
  } else {
    display.print("--");
  }

  display.setCursor(92, 48);
  if (motorInGracePeriod()) {
    display.print("G:");
    display.print(remainingGraceMs() / 1000UL);
    display.print("s");
  } else {
    display.print("L:");
    display.print(faultLatched ? "Y" : "N");
  }

  display.display();
}


// =====================================================================
// LOCAL SERIAL KOMUTLAR
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
  if (strcmp(label, "STOP") == 0) {
    return STATE_STOP;
  }

  if (strcmp(label, "NORMAL") == 0) {
    return STATE_NORMAL;
  }

  if (strcmp(label, "WARNING") == 0) {
    return STATE_WARNING;
  }

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

  // Motor OFF iken state'i modelden güncellemiyoruz.
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

  // anomalyCloud ve confidenceCloud sadece burada yazılır.
  anomalyCloud = lastAnomaly;
  confidenceCloud = lastConfidence * 100.0f;

  SystemState detectedState = labelToState(bestLabel);

  // Motor OFF iken model sonucu dikkate alınmaz.
  if (!motorOn) {
    detectedState = STATE_STOP;
    vibrationFaultCount = 0;
  }

  // Vibration FAULT:
  // - Motor ON
  // - Grace bitmiş
  // - Anomaly eşik üstünde
  // - Üst üste 2 inference doğrulanmış
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

    // Push button / switch true kaldıysa tekrar false'a çek.
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

  // Başlangıçta mevcut lokal değerleri Cloud'a yaz.
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
