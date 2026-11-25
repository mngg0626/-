#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>

// Auto-ranging sweep for the stacked mA + uA shields.
//   mA range: relay released  -> sense node -> 200 ohm -> relay NC -> GND
//   uA range: relay energized -> sense node -> 10 kohm -> relay NO -> GND (200 ohm path left open)
// The sweep starts in the uA range and moves to the mA range once the sense voltage
// passes kUaToMaSenseV. Before the relay moves, the DAC is lowered so the DUT keeps
// roughly the same operating point instead of seeing the full source voltage at once.
//
// Optional start field "range": "auto" (default) | "ua" | "ma".
//   auto without the ADS1115 falls back to the mA range only.

namespace {
constexpr uint32_t kBaud = 115200;
constexpr float kAdcReferenceV = 5.0f;  // Nominal. Corrected at run time from the ADS1115 (see refScale).
constexpr float kMaSenseOhms = 200.0f;
constexpr float kUaSenseOhms = 10000.0f;
constexpr float kSweepCurrentLimitA = 0.008f;  // End the sweep at this measured current boundary.
constexpr float kCurrentCutoffA = 0.012f;  // Software fault cutoff; the 200 ohm series resistor stays fitted.
constexpr uint16_t kAdcMax = 16383;
constexpr float kUnoVoltsPerCount = kAdcReferenceV / kAdcMax;
constexpr float kDacVoltsPerCode = kAdcReferenceV / 1024.0f;
constexpr uint16_t kLastDacCode = 880;  // Nominal 4.297 V; source voltage, not DUT voltage.
constexpr uint16_t kDacStep = 10;  // Nominal 48.8 mV; bounds the next-step current increment through 200 ohms.
constexpr float kNominalSourceMaxV = kDacVoltsPerCode * kLastDacCode;
constexpr float kOutputToleranceV = 0.25f;
static_assert(kNominalSourceMaxV <= 4.3f, "Review DUT ratings before increasing the sweep voltage");
static_assert(kSweepCurrentLimitA + kDacVoltsPerCode * kDacStep / kMaSenseOhms < kCurrentCutoffA,
              "The normal current boundary must precede the fault cutoff");
constexpr uint32_t kSettleMs = 220;  // Also covers relay contact bounce after a range change.
constexpr uint32_t kSampleSpacingMs = 20;
constexpr uint8_t kSamples = 10;
constexpr uint32_t kHeartbeatTimeoutMs = 1200;
constexpr uint32_t kCurrentCheckMs = 5;

// Range relay on the uA shield: D4 -> 1 k -> Q1 base. With D4 LOW and no DUT, the 200 ohm
// end on J3 should read 0 ohm to GND. If it is the other way round, set this to false.
constexpr uint8_t kRangeRelayPin = 4;
constexpr bool kUaWhenRelayHigh = true;
constexpr float kUaToMaSenseV = 1.3f;  // 130 uA through 10 kohm.
// In the uA range the sense node stays below kUaToMaSenseV + one step. A higher reading
// means the 200 ohm path is still closed, so it is treated as a relay fault before 8 mA.
constexpr float kUaFaultSenseV = kSweepCurrentLimitA * kMaSenseOhms;  // 1.6 V
static_assert(kUaToMaSenseV + kDacVoltsPerCode * kDacStep < kUaFaultSenseV * 0.9f,
              "Leave margin between the uA switch point and the relay fault check");

// ADS1115 on the uA shield. ADDR is tied to GND.
constexpr uint8_t kAdsAddress = 0x48;
constexpr uint8_t kAdsSource = 0;  // Buffered source node.
constexpr uint8_t kAdsSense = 1;   // Buffered sense node (A2 carries the same node).
constexpr float kAdsFullScaleV[] = {6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f};
constexpr uint8_t kAdsWidestPga = 0;
constexpr uint8_t kAdsNarrowestPga = 5;
constexpr uint16_t kAdsRate250 = 0x5 << 5;
constexpr uint32_t kAdsTimeoutUs = 20000;

// The UNO ADC uses the USB 5 V as its reference; the ADS1115 has a precise internal one.
// In the uA range both read the source node, so their ratio corrects the UNO readings
// in the mA range. Without this the curve shows a small step at the range change.
constexpr float kCalMinSourceV = 0.3f;  // Skip readings too small to compare.
constexpr float kCalMaxSourceV = 3.0f;  // Stay below where a 5 V op amp buffer may clip.
constexpr float kRefScaleMin = 0.94f;   // USB 5 V +-6 %; anything outside is a wiring fault.
constexpr float kRefScaleMax = 1.06f;

enum class Range : uint8_t { ua, ma };

char rx[256];
size_t rxSize = 0;
bool discardLine = false;
char runId[33] = {};
bool measuring = false;
bool autoRange = false;
Range range = Range::ma;
uint32_t startedAt;
uint32_t lastHeartbeat;
uint32_t nextSampleAt;
uint32_t nextCurrentCheckAt;
uint16_t dacCode;
uint16_t seq;
float sumSourceV;
float sumSenseV;
uint32_t sumTrackCounts;  // UNO A1 counts, compared ratiometrically with the DAC code.
uint8_t sampleCount;
uint8_t sensePga;
float switchCurrentA = -1;  // uA-range current at the range change, checked on the first mA point.
float refScale = 1.0f;      // Actual UNO reference / 5 V, learned in the uA range.
float calAdsV;
float calUnoV;

const char* rangeName(Range value) { return value == Range::ua ? "ua" : "ma"; }
float senseOhms(Range value) { return value == Range::ua ? kUaSenseOhms : kMaSenseOhms; }
float unoVolts(uint16_t raw) { return raw * kUnoVoltsPerCount * refScale; }

bool validId(const char* id) {
  return id && strlen(id) == 32 && strspn(id, "0123456789abcdef") == 32;
}

bool due(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

void setRange(Range value) {
  range = value;
  digitalWrite(kRangeRelayPin, (value == Range::ua) == kUaWhenRelayHigh ? HIGH : LOW);
}

// ---------- ADS1115 ----------
bool adsWrite(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(kAdsAddress);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value >> 8));
  Wire.write(static_cast<uint8_t>(value & 0xFF));
  return Wire.endTransmission() == 0;
}

bool adsRead(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(kAdsAddress);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(kAdsAddress, static_cast<size_t>(2)) != 2) return false;
  const uint8_t high = Wire.read();
  const uint8_t low = Wire.read();
  value = static_cast<uint16_t>(high) << 8 | low;
  return true;
}

bool adsPresent() {
  Wire.beginTransmission(kAdsAddress);
  return Wire.endTransmission() == 0;
}

// Single-shot, single-ended conversion. Returns false on an I2C fault or timeout.
bool adsVolts(uint8_t channel, uint8_t pga, float& volts, bool& clipped) {
  const uint16_t config = 0x8000                            // Start one conversion.
                          | (0x4 | channel) << 12          // AINx against GND.
                          | static_cast<uint16_t>(pga) << 9
                          | 0x0100                          // Single-shot mode.
                          | kAdsRate250
                          | 0x0003;                         // Comparator off.
  if (!adsWrite(1, config)) return false;
  const uint32_t begun = micros();
  delayMicroseconds(3500);  // Nominal 4 ms at 250 SPS.
  uint16_t status = 0;
  do {
    if (!adsRead(1, status) || micros() - begun > kAdsTimeoutUs) return false;
  } while (!(status & 0x8000));
  uint16_t raw;
  if (!adsRead(0, raw)) return false;
  const int16_t code = static_cast<int16_t>(raw);
  clipped = code == 32767;
  volts = code * kAdsFullScaleV[pga] / 32768.0f;
  return true;
}

uint8_t pickPga(float coarseV) {
  const float needed = fabsf(coarseV) * 1.25f + 0.005f;
  uint8_t pga = kAdsWidestPga;
  while (pga < kAdsNarrowestPga && kAdsFullScaleV[pga + 1] > needed) ++pga;
  return pga;
}

// ---------- Events ----------
void sendEvent(const char* type, const char* id, JsonDocument& event) {
  event["proto"] = 1;
  event["type"] = type;
  event["id"] = id;
  char line[320];
  const size_t size = serializeJson(event, line, sizeof(line) - 1);
  line[size] = '\n';
  Serial1.write(reinterpret_cast<const uint8_t*>(line), size + 1);
}

void sendRange(Range value, const char* reason = nullptr) {
  JsonDocument event;
  event["phase"] = "active";
  event["range"] = rangeName(value);
  if (value == Range::ma) event["refScale"] = refScale;
  if (reason) event["reason"] = reason;
  sendEvent("range", runId, event);
}

void acknowledge(const char* id, const char* command, bool ok, const char* reason = nullptr,
                 const char* mode = nullptr, bool adsReady = false) {
  JsonDocument event;
  char requestId[40];
  snprintf(requestId, sizeof(requestId), "%s:%s", id, command);
  event["command"] = command;
  event["requestId"] = requestId;
  event["ok"] = ok;
  if (reason) event["reason"] = reason;
  if (ok && strcmp(command, "start") == 0) {
    JsonObject conditions = event["conditions"].to<JsonObject>();
    conditions["profile"] = "auto-ua-ma-v1-4v3-8ma";
    conditions["direction"] = "up";
    conditions["settleMs"] = kSettleMs;
    conditions["rangeMode"] = mode;
    conditions["startRange"] = rangeName(range);
    conditions["uaAdc"] = adsReady;
    conditions["uaToMaA"] = kUaToMaSenseV / kUaSenseOhms;
  }
  sendEvent("ack", id, event);
}

void zeroOutput() {
  analogWrite(A0, 0);
  measuring = false;
  setRange(Range::ma);  // Release the relay coil while idle.
}

void fail(const char* code, const char* message) {
  zeroOutput();
  JsonDocument event;
  event["code"] = code;
  event["message"] = message;
  sendEvent("error", runId, event);
}

void finishSweep(Range lastRange, const char* reason) {
  zeroOutput();
  // Completion means the configured voltage OR current boundary was reached.
  sendRange(lastRange, reason);
  JsonDocument done;
  done["count"] = seq;
  sendEvent("done", runId, done);
}

void setSweepPoint(uint32_t now) {
  analogWrite(A0, dacCode);
  sumSourceV = 0;
  sumSenseV = 0;
  sumTrackCounts = 0;
  sampleCount = 0;
  nextSampleAt = now + kSettleMs;
  nextCurrentCheckAt = now + kCurrentCheckMs;
}

// Fix the UNO reference scale from the uA-range points collected so far.
void applyReferenceScale() {
  if (calUnoV <= 0) return;  // No usable uA points: keep the nominal 5 V.
  const float scale = calAdsV / calUnoV;
  if (scale >= kRefScaleMin && scale <= kRefScaleMax) refScale = scale;
}

// Lower the source first so the DUT keeps its last operating point, then release the relay.
void switchToMa(float dutV, float currentA) {
  switchCurrentA = currentA;
  applyReferenceScale();
  const float targetV = dutV + currentA * kMaSenseOhms;
  long code = lroundf(ceilf(targetV / kDacVoltsPerCode)) + kDacStep;
  if (code > kLastDacCode) code = kLastDacCode;
  if (code < 0) code = 0;
  dacCode = static_cast<uint16_t>(code);
  analogWrite(A0, dacCode);  // The 10 k path still limits the current here.
  delay(2);
  setRange(Range::ma);
  sendRange(Range::ma, "전류 130µA 도달 · mA 레인지(200Ω) 전환");
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

  const char* mode = cmd["range"] | "auto";
  const bool modeAuto = strcmp(mode, "auto") == 0;
  const bool modeUa = strcmp(mode, "ua") == 0;
  if (!modeAuto && !modeUa && strcmp(mode, "ma") != 0) {
    acknowledge(id, "start", false, "Unknown range mode. Use auto, ua or ma.");
    return;
  }
  const bool adsReady = adsPresent();
  if (modeUa && !adsReady) {
    acknowledge(id, "start", false, "ADS1115 not found at 0x48. Check the uA shield and I2C wiring.");
    return;
  }

  memcpy(runId, id, sizeof(runId));
  autoRange = modeAuto && adsReady;
  startedAt = millis();
  lastHeartbeat = startedAt;
  dacCode = 0;
  seq = 0;
  switchCurrentA = -1;
  refScale = 1.0f;
  calAdsV = 0;
  calUnoV = 0;
  measuring = true;
  setRange(modeUa || autoRange ? Range::ua : Range::ma);
  setSweepPoint(startedAt);
  acknowledge(runId, "start", true, nullptr, mode, adsReady);
  sendRange(range);
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

// One uA-range sample through the ADS1115. The sense gain is chosen once per point.
bool readUaSample(float& sourceV, float& senseV) {
  bool clipped = false;
  if (sampleCount == 0) {
    float coarseV;
    if (!adsVolts(kAdsSense, kAdsWidestPga, coarseV, clipped)) {
      fail("ADS_I2C", "ADS1115 stopped responding. Output set to zero.");
      return false;
    }
    sensePga = pickPga(coarseV);
  }
  if (!adsVolts(kAdsSource, kAdsWidestPga, sourceV, clipped) ||
      !adsVolts(kAdsSense, sensePga, senseV, clipped)) {
    fail("ADS_I2C", "ADS1115 stopped responding. Output set to zero.");
    return false;
  }
  if (clipped) {
    fail("ADC_SATURATION", "ADS1115 sense input saturated in the uA range.");
    return false;
  }
  return true;
}

void completePoint(uint32_t now) {
  const Range pointRange = range;
  const float sourceV = sumSourceV / kSamples;
  const float senseV = sumSenseV / kSamples;
  // UNO A1 and the DAC share the USB 5 V reference, so this check is immune to its error
  // and to a clipping buffer on the uA shield.
  const float trackedV = sumTrackCounts * kUnoVoltsPerCount / kSamples;
  const float targetV = kDacVoltsPerCode * dacCode;
  if (fabsf(trackedV - targetV) > kOutputToleranceV) {
    fail("OUTPUT_TRACKING", "Buffer output does not follow DAC. Check A0, A1, op amp and ground.");
    return;
  }
  const float currentA = senseV / senseOhms(pointRange);
  if (pointRange == Range::ua && trackedV >= kCalMinSourceV && trackedV <= kCalMaxSourceV) {
    calAdsV += sourceV;
    calUnoV += trackedV;
  }
  if (pointRange == Range::ma && switchCurrentA > 0) {
    // The DAC is set so the DUT keeps its operating point, plus one step (at most about
    // 0.24 mA more). A current far outside that band means the relay did not move.
    const float stepA = kDacVoltsPerCode * kDacStep / kMaSenseOhms;
    const bool jumped = currentA > switchCurrentA + 1.5f * stepA || currentA < switchCurrentA * 0.5f;
    switchCurrentA = -1;
    if (jumped) {
      fail("RANGE_FAULT", "Current jumped at the uA to mA change. Range relay may be stuck.");
      return;
    }
  }
  const bool lastPoint = dacCode >= kLastDacCode;
  if (lastPoint) zeroOutput();
  JsonDocument point;
  point["seq"] = seq++;
  point["sweep"] = 0;
  point["range"] = rangeName(pointRange);
  point["v"] = sourceV - senseV;
  point["a"] = currentA;
  point["elapsedMs"] = now - startedAt;
  sendEvent("point", runId, point);

  if (lastPoint) {
    finishSweep(pointRange, "출력 전압 상한 도달 · 명목 4.297V");
  } else if (pointRange == Range::ua && senseV >= kUaToMaSenseV) {
    if (autoRange) switchToMa(sourceV - senseV, currentA);
    else finishSweep(Range::ua, "uA 레인지 상한 130µA 도달 · 고정 레인지 측정 종료");
  } else {
    dacCode = dacCode + kDacStep > kLastDacCode ? kLastDacCode : dacCode + kDacStep;
    setSweepPoint(millis());
  }
}

void measure(uint32_t now) {
  if (now - lastHeartbeat >= kHeartbeatTimeoutMs) {
    fail("LINK_TIMEOUT", "ESP32 heartbeat lost. DAC set to zero.");
    return;
  }
  if (!due(now, nextCurrentCheckAt)) return;
  nextCurrentCheckAt = now + kCurrentCheckMs;

  // Fast guard on the UNO ADC in both ranges.
  const uint16_t rawA2 = analogRead(A2);
  const float fastSenseV = unoVolts(rawA2);
  if (fastSenseV >= kCurrentCutoffA * kMaSenseOhms) {
    fail("CURRENT_LIMIT", "12 mA safety cutoff reached. Output set to zero. Check the 200 ohm sense resistor and wiring.");
    return;
  }
  if (range == Range::ua && fastSenseV >= kUaFaultSenseV) {
    fail("RANGE_FAULT", "Sense voltage too high in uA range. Range relay may be stuck. Output set to zero.");
    return;
  }
  if (range == Range::ma && fastSenseV >= kSweepCurrentLimitA * kMaSenseOhms) {
    if (!seq) {
      fail("OUTPUT_NOT_ZERO", "Current detected before a valid first point. Check output and wiring.");
    } else {
      // Do not append an unsettled or over-boundary point. Keep the last completed averages.
      finishSweep(Range::ma, "전류 상한 8mA 도달 · 마지막 안정 측정점까지 완료 · 최대 전압 미도달");
    }
    return;
  }
  if (!due(now, nextSampleAt)) return;
  nextSampleAt = now + kSampleSpacingMs;

  const uint16_t rawA1 = analogRead(A1);
  if (rawA1 == kAdcMax) {
    fail("ADC_SATURATION", "A1 saturated. Check the buffer and voltage reference.");
    return;
  }
  sumTrackCounts += rawA1;
  float sourceV;
  float senseV;
  if (range == Range::ma) {
    sourceV = unoVolts(rawA1);
    senseV = fastSenseV;
  } else if (!readUaSample(sourceV, senseV)) {
    return;
  }
  sumSourceV += sourceV;
  sumSenseV += senseV;
  if (++sampleCount == kSamples) completePoint(now);
}
}  // namespace

void setup() {
  pinMode(2, INPUT);  // D2 shares the sense node on the mA shield.
  pinMode(kRangeRelayPin, OUTPUT);
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
