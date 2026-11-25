#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>

// Stack: UNO R4 -> mA shield (DAC buffer, 200 ohm) -> uA shield (relay, 10 k, ADS1115).
// mA shield: A0 DAC -> op amp buffer -> source node (A1, J3-1). DUT -> sense node (A2, D2, J3-2).
//            200 ohm: sense node -> J3-3.
// uA shield: relay COM = GND. One contact -> J3-3 (200 ohm path), the other -> 10 k -> sense node.
//            D4 -> 1 k -> NPN base drives the relay coil (HIGH = coil energized).
//            ADS1115 (ADDR = GND, 0x48): AIN0 = buffered source, AIN1/AIN2 = buffered sense, AIN3 = GND.

namespace {
constexpr uint32_t kBaud = 115200;
constexpr float kAdcReferenceV = 5.0f;  // Nominal until checked against a meter.
constexpr uint16_t kAdcMax = 16383;
constexpr float kVoltsPerCount = kAdcReferenceV / kAdcMax;

enum class Range : uint8_t { ua, ma };
constexpr float kMaSenseOhms = 200.0f;
constexpr float kUaSenseOhms = 10000.0f;
constexpr uint8_t kRelayPin = 4;
// true: D4 HIGH (coil energized) grounds the 10 k path. Check once with a meter before trusting it.
constexpr bool kUaRelayHigh = true;

constexpr float kSweepCurrentLimitA = 0.008f;  // End the sweep at this measured current boundary.
constexpr float kCurrentCutoffA = 0.012f;      // Software fault cutoff on the 200 ohm range.
// The cutoff is applied to the sense-node voltage, so it holds even if the relay is in the wrong state.
constexpr float kSenseCutoffV = kCurrentCutoffA * kMaSenseOhms;  // 2.4 V
constexpr float kUaUpperA = 200e-6f;  // uA range hands off to mA range at this averaged current.

constexpr uint16_t kLastDacCode = 880;  // Nominal 4.297 V; source voltage, not DUT voltage.
constexpr uint16_t kDacStep = 10;       // Nominal 48.8 mV per step.
constexpr float kStepV = kAdcReferenceV * kDacStep / 1024.0f;
constexpr float kNominalSourceMaxV = kAdcReferenceV * kLastDacCode / 1024.0f;
constexpr float kOutputToleranceV = 0.25f;
static_assert(kNominalSourceMaxV <= 4.3f, "Review DUT ratings before increasing the sweep voltage");
static_assert(kLastDacCode % kDacStep == 0, "The sweep must reach its source voltage endpoint");
static_assert(kSweepCurrentLimitA + kStepV / kMaSenseOhms < kCurrentCutoffA,
              "The normal current boundary must precede the fault cutoff");
static_assert((kUaUpperA + kStepV / kUaSenseOhms) * kUaSenseOhms < kSenseCutoffV,
              "A normal uA-range step must stay below the sense-node cutoff");
static_assert(kNominalSourceMaxV / kUaSenseOhms > kUaUpperA, "The uA range must be able to reach its handoff current");

constexpr uint32_t kSettleMs = 220;  // Also covers relay operate and bounce time (< 10 ms).
constexpr uint32_t kSampleSpacingMs = 20;
constexpr uint8_t kSamples = 10;
constexpr uint32_t kHeartbeatTimeoutMs = 1200;
constexpr uint32_t kCurrentCheckMs = 5;

// Minimal ADS1115 driver: single-shot AIN2-AIN3 with automatic gain.
namespace ads {
constexpr uint8_t kAddress = 0x48;
constexpr uint8_t kConversionReg = 0x00;
constexpr uint8_t kConfigReg = 0x01;
constexpr float kFullScaleV[] = {6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f};  // PGA codes 0..5
constexpr uint32_t kTimeoutMs = 12;  // 250 SPS -> 4 ms per conversion.

bool present() {
  Wire.beginTransmission(kAddress);
  return Wire.endTransmission() == 0;
}

bool writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value >> 8));
  Wire.write(static_cast<uint8_t>(value & 0xFF));
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  if (Wire.endTransmission() != 0 || Wire.requestFrom(kAddress, static_cast<uint8_t>(2)) != 2) return false;
  value = static_cast<uint16_t>(Wire.read()) << 8;
  value |= static_cast<uint16_t>(Wire.read());
  return true;
}

bool convert(uint8_t pga, int16_t& raw) {
  const uint16_t config = 0x8000                // OS: start a single conversion
                          | (0b011 << 12)       // MUX: AIN2 - AIN3 (AIN3 is GND on the board)
                          | (pga << 9)          // PGA full scale
                          | 0x0100              // single-shot mode
                          | (0b101 << 5)        // 250 SPS
                          | 0x0003;             // comparator off
  if (!writeRegister(kConfigReg, config)) return false;
  const uint32_t started = millis();
  uint16_t status = 0;
  do {
    delayMicroseconds(500);
    if (!readRegister(kConfigReg, status)) return false;
  } while (!(status & 0x8000) && millis() - started < kTimeoutMs);
  uint16_t value;
  if (!(status & 0x8000) || !readRegister(kConversionReg, value)) return false;
  raw = static_cast<int16_t>(value);
  return true;
}

// hintV comes from the UNO A2 reading and picks the narrowest safe range.
bool readSense(float hintV, float& volts) {
  uint8_t pga = 5;
  while (pga > 0 && kFullScaleV[pga] < fabsf(hintV) * 1.25f + 0.02f) --pga;
  for (;;) {
    int16_t raw;
    if (!convert(pga, raw)) return false;
    const bool clipped = raw == INT16_MAX || raw == INT16_MIN;
    if (clipped && pga > 0) {
      --pga;  // Widen the range and convert again.
      continue;
    }
    volts = raw * kFullScaleV[pga] / 32768.0f;
    return !clipped;
  }
}
}  // namespace ads

char rx[256];
size_t rxSize = 0;
bool discardLine = false;
char runId[33] = {};
bool measuring = false;
uint32_t startedAt;
uint32_t lastHeartbeat;
uint32_t nextSampleAt;
uint32_t nextCurrentCheckAt;
uint16_t dacCode;
uint16_t seq;
uint32_t sumA1;
uint32_t sumA2;
float sumAdsV;
uint8_t sampleCount;
Range range = Range::ma;
bool autoRange = false;
bool checkHandoff = false;  // Verify the first mA point after a uA -> mA handoff.
float handoffCurrentA = 0;

const char* rangeName(Range value) { return value == Range::ua ? "ua" : "ma"; }
float senseOhms(Range value) { return value == Range::ua ? kUaSenseOhms : kMaSenseOhms; }

void setRelay(Range value) {
  digitalWrite(kRelayPin, (value == Range::ua) == kUaRelayHigh ? HIGH : LOW);
}

bool validId(const char* id) {
  return id && strlen(id) == 32 && strspn(id, "0123456789abcdef") == 32;
}

bool due(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

void sendEvent(const char* type, const char* id, JsonDocument& event) {
  event["proto"] = 1;
  event["type"] = type;
  event["id"] = id;
  char line[320];
  const size_t size = serializeJson(event, line, sizeof(line) - 1);
  line[size] = '\n';
  Serial1.write(reinterpret_cast<const uint8_t*>(line), size + 1);
}

void acknowledge(const char* id, const char* command, bool ok, const char* reason = nullptr,
                 const char* rangeMode = nullptr, bool uaAdc = false) {
  JsonDocument event;
  char requestId[40];
  snprintf(requestId, sizeof(requestId), "%s:%s", id, command);
  event["command"] = command;
  event["requestId"] = requestId;
  event["ok"] = ok;
  if (reason) event["reason"] = reason;
  if (ok && strcmp(command, "start") == 0) {
    JsonObject conditions = event["conditions"].to<JsonObject>();
    conditions["profile"] = "auto-v3-4v3-8ma";
    conditions["direction"] = "up";
    conditions["settleMs"] = kSettleMs;
    conditions["rangeMode"] = rangeMode;
    conditions["startRange"] = rangeName(range);
    conditions["uaAdc"] = uaAdc;
  }
  sendEvent("ack", id, event);
}

void sendRange(const char* reason = nullptr) {
  JsonDocument event;
  event["phase"] = "active";
  event["range"] = rangeName(range);
  event["senseOhms"] = senseOhms(range);
  if (reason) event["reason"] = reason;
  sendEvent("range", runId, event);
}

void zeroOutput() {
  analogWrite(A0, 0);
  setRelay(Range::ma);  // Idle on the 200 ohm path; `range` keeps the last reported value.
  measuring = false;
}

void fail(const char* code, const char* message) {
  zeroOutput();
  JsonDocument event;
  event["code"] = code;
  event["message"] = message;
  sendEvent("error", runId, event);
}

void finishSweep(const char* reason) {
  zeroOutput();
  // Completion means the configured voltage OR current boundary was reached.
  sendRange(reason);
  JsonDocument done;
  done["count"] = seq;
  sendEvent("done", runId, done);
}

void setSweepPoint(uint32_t now) {
  analogWrite(A0, dacCode);
  sumA1 = 0;
  sumA2 = 0;
  sumAdsV = 0;
  sampleCount = 0;
  nextSampleAt = now + kSettleMs;
  nextCurrentCheckAt = now + kCurrentCheckMs;
}

// Switching 10 k -> 200 ohm at the same source voltage would multiply the current.
// Lower the source first so the DUT keeps roughly the same operating point:
//   source' = V_dut + I * 200 ohm (rounded down to the DAC step grid).
void enterMaRange(float dutV, float currentA) {
  const float targetV = fmaxf(dutV + currentA * kMaSenseOhms, 0.0f);
  uint16_t code = static_cast<uint16_t>(targetV * 1024.0f / kAdcReferenceV);
  code -= code % kDacStep;
  if (code < dacCode) dacCode = code;  // Never raise the source during a handoff.
  analogWrite(A0, dacCode);  // The 10 k path still limits the current here.
  range = Range::ma;
  setRelay(range);
  checkHandoff = true;
  handoffCurrentA = currentA;
  sendRange("전류 200µA 도달 · mA 레인지(200Ω) 전환");
  setSweepPoint(millis());
}

void handleCommand(const char* line) {
  JsonDocument cmd;
  if (deserializeJson(cmd, line, DeserializationOption::NestingLimit(3)) || cmd["proto"] != 1) return;
  const char* id = cmd["id"] | "";
  if (!validId(id)) return;
  const char* type = cmd["type"] | "";
  const bool sameRun = strcmp(id, runId) == 0;
  if (strcmp(type, "keepalive") == 0) {
    if (measuring && sameRun) lastHeartbeat = millis();
    return;
  }

  const bool start = strcmp(type, "start") == 0;
  const bool stop = strcmp(type, "stop") == 0;
  if (!start && !stop) return;
  char expectedRequest[40];
  snprintf(expectedRequest, sizeof(expectedRequest), "%s:%s", id, type);
  if (strcmp(cmd["requestId"] | "", expectedRequest) != 0) return;

  if (stop) {
    if (sameRun) zeroOutput();
    acknowledge(id, "stop", sameRun, sameRun ? nullptr : "Measurement ID not active on Arduino");
    return;
  }
  if (measuring || sameRun) {
    if (measuring && sameRun) return;
    acknowledge(id, "start", false, measuring ? "Arduino busy" : "Use a new measurement ID");
    return;
  }

  // Optional "range": "auto" (default) | "ua" | "ma". Fixed "ua" ends the sweep at 200 uA.
  const char* mode = cmd["range"] | "auto";
  const bool modeAuto = strcmp(mode, "auto") == 0;
  const bool modeUa = strcmp(mode, "ua") == 0;
  const bool modeMa = strcmp(mode, "ma") == 0;
  if (!modeAuto && !modeUa && !modeMa) {
    acknowledge(id, "start", false, "Unknown range mode");
    return;
  }
  const bool adsReady = ads::present();
  if (modeUa && !adsReady) {
    acknowledge(id, "start", false, "ADS1115 not found. Check the uA board and I2C wiring.");
    return;
  }

  memcpy(runId, id, sizeof(runId));
  autoRange = modeAuto && adsReady;  // Without the ADS1115, auto falls back to mA only.
  range = modeUa || autoRange ? Range::ua : Range::ma;
  checkHandoff = false;
  setRelay(range);
  startedAt = millis();
  lastHeartbeat = startedAt;
  dacCode = 0;
  seq = 0;
  measuring = true;
  setSweepPoint(startedAt);  // The settle time also covers the relay.
  acknowledge(runId, "start", true, nullptr, mode, adsReady);
  sendRange();
}

void readCommands() {
  // Bound work per iteration so serial input cannot starve the watchdog.
  for (size_t read = 0; read < sizeof(rx) && Serial1.available(); ++read) {
    const char byte = static_cast<char>(Serial1.read());
    if (byte == '\n') {
      rx[rxSize] = '\0';
      if (!discardLine && rxSize) handleCommand(rx);
      rxSize = 0;
      discardLine = false;
    } else if (byte != '\r' && !discardLine) {
      if (byte == '\0' || rxSize == sizeof(rx) - 1) discardLine = true;
      else rx[rxSize++] = byte;
    }
  }
}

void measure(uint32_t now) {
  if (now - lastHeartbeat >= kHeartbeatTimeoutMs) {
    fail("LINK_TIMEOUT", "ESP32 heartbeat lost. DAC set to zero.");
    return;
  }
  if (!due(now, nextCurrentCheckAt)) return;
  nextCurrentCheckAt = now + kCurrentCheckMs;
  const uint16_t rawA2 = analogRead(A2);
  const float senseFastV = rawA2 * kVoltsPerCount;
  if (senseFastV >= kSenseCutoffV) {
    fail("CURRENT_LIMIT", "Sense node reached 2.4 V (12 mA on 200 ohm). Output set to zero. Check wiring and relay.");
    return;
  }
  const float fastCurrent = senseFastV / senseOhms(range);
  if (range == Range::ma && fastCurrent >= kSweepCurrentLimitA) {
    if (!seq) {
      fail("OUTPUT_NOT_ZERO", "Current detected before a valid first point. Check output and wiring.");
    } else {
      // Do not append an unsettled or over-boundary point. Keep the last completed averages.
      finishSweep("전류 상한 8mA 도달 · 마지막 안정 측정점까지 완료 · 최대 전압 미도달");
    }
    return;
  }
  if (range == Range::ua && !seq && fastCurrent >= kUaUpperA) {
    fail("OUTPUT_NOT_ZERO", "Current detected before a valid first point. Check output and wiring.");
    return;
  }
  if (!due(now, nextSampleAt)) return;
  nextSampleAt = now + kSampleSpacingMs;
  const uint16_t rawA1 = analogRead(A1);
  if (rawA1 == kAdcMax) {
    fail("ADC_SATURATION", "A1 saturated. Check the buffer and voltage reference.");
    return;
  }
  sumA1 += rawA1;
  if (range == Range::ma) {
    sumA2 += rawA2;
  } else {
    float adsV;
    if (!ads::readSense(senseFastV, adsV)) {
      fail("ADS_READ", "ADS1115 read failed. Check the uA board, I2C wiring and 5 V.");
      return;
    }
    sumAdsV += adsV;
  }
  if (++sampleCount != kSamples) return;

  const float sourceV = sumA1 * kVoltsPerCount / kSamples;
  const float senseV = range == Range::ua ? sumAdsV / kSamples : sumA2 * kVoltsPerCount / kSamples;
  const float currentA = senseV / senseOhms(range);
  const float targetV = kAdcReferenceV * dacCode / 1024.0f;
  if (fabsf(sourceV - targetV) > kOutputToleranceV) {
    fail("OUTPUT_TRACKING", "Buffer output does not follow DAC. Check A0, A1, op amp and ground.");
    return;
  }
  // After a handoff the source was lowered, so the current cannot grow much.
  // A large jump means the relay grounds the wrong resistor (NC/NO swapped).
  if (checkHandoff) {
    checkHandoff = false;
    if (currentA > 2.0f * handoffCurrentA + 20e-6f) {
      fail("RANGE_MISMATCH", "Current jumped after uA->mA switch. Check relay NC/NO and kUaRelayHigh.");
      return;
    }
  }
  const bool lastPoint = dacCode == kLastDacCode;
  if (lastPoint) zeroOutput();
  JsonDocument point;
  point["seq"] = seq++;
  point["sweep"] = 0;
  point["range"] = rangeName(range);
  point["v"] = sourceV - senseV;
  point["a"] = currentA;
  point["elapsedMs"] = now - startedAt;
  sendEvent("point", runId, point);
  if (lastPoint) {
    finishSweep("출력 전압 상한 도달 · 명목 4.297V");
  } else if (range == Range::ua && currentA >= kUaUpperA) {
    if (autoRange) enterMaRange(sourceV - senseV, currentA);
    else finishSweep("uA 레인지 상한 200µA 도달 · 고정 레인지 측정 종료");
  } else {
    dacCode += kDacStep;
    setSweepPoint(millis());
  }
}
}  // namespace

void setup() {
  pinMode(2, INPUT);  // D2 shares the sense node on the mA shield.
  pinMode(kRelayPin, OUTPUT);
  analogWriteResolution(10);
  analogReadResolution(14);
  zeroOutput();
  Wire.begin();
  Wire.setClock(400000);
  Serial1.begin(kBaud);  // D0 RX / D1 TX, independent of USB Serial.
}

void loop() {
  if (measuring && millis() - lastHeartbeat >= kHeartbeatTimeoutMs) measure(millis());
  readCommands();
  if (measuring) measure(millis());
}
