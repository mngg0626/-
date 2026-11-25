#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>

// mA + uA 실드 적층용 자동 레인지 측정
//   - mA 레인지: 릴레이 OFF -> 센스 노드 -> 200 ohm -> 릴레이 NC(2번) -> GND
//   - uA 레인지: 릴레이 ON -> 센스 노드 -> 10 kohm -> 릴레이 NO(4번) -> GND (200 ohm 경로 개방)
//   - uA 레인지로 시작, 센스 전압 kUaToMaSenseV 초과 시 mA 레인지로 전환
//   - 전환 전 DAC 선행 하강: DUT에 소스 전압 일시 인가 방지,
//     직전 동작점 근처 유지
//
// start 명령 선택 필드 "range": "auto"(기본) | "ua" | "ma"
//   - auto + ADS1115 미검출 시 mA 레인지 단독 측정

namespace {
constexpr uint32_t kBaud = 115200;
constexpr float kAdcReferenceV = 5.0f;  // Nominal until checked against a meter. (추가: 측정 중 ADS1115 기준 보정, refScale)
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
constexpr uint32_t kSettleMs = 220;  // 레인지 전환 후 릴레이 접점 바운스 시간 포함
constexpr uint32_t kSampleSpacingMs = 20;
constexpr uint8_t kSamples = 10;
constexpr uint32_t kHeartbeatTimeoutMs = 1200;
constexpr uint32_t kCurrentCheckMs = 5;

// uA 실드 레인지 릴레이(G6K-2P-Y): D4 -> 1 k -> Q1 베이스
//   - D4 LOW, DUT 제거 상태 A2-GND 측정: 약 200 ohm -> true, 약 10 kohm -> false
constexpr uint8_t kRangeRelayPin = 4;
constexpr bool kUaWhenRelayHigh = true;
constexpr float kUaToMaSenseV = 1.3f;  // 10 kohm 기준 130 uA
// uA 레인지 센스 노드 상한: kUaToMaSenseV + 1 스텝
//   - 초과 시 200 ohm 경로 잔존(릴레이 고장)으로 판단, 8 mA 이전 차단
constexpr float kUaFaultSenseV = kSweepCurrentLimitA * kMaSenseOhms;  // 1.6 V
static_assert(kUaToMaSenseV + kDacVoltsPerCode * kDacStep < kUaFaultSenseV * 0.9f,
              "Leave margin between the uA switch point and the relay fault check");

// uA 실드 ADS1115, ADDR-GND 연결(주소 0x48)
constexpr uint8_t kAdsAddress = 0x48;
constexpr uint8_t kAdsSource = 0;  // 버퍼 출력 소스 노드
constexpr uint8_t kAdsSense = 1;   // 버퍼 출력 센스 노드 (A2와 동일 노드)
constexpr float kAdsFullScaleV[] = {6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f};
constexpr uint8_t kAdsWidestPga = 0;
constexpr uint8_t kAdsNarrowestPga = 5;
constexpr uint16_t kAdsRate250 = 0x5 << 5;
constexpr uint32_t kAdsTimeoutUs = 20000;

// 기준전압 보정
//   - UNO ADC: USB 5 V 기준 / ADS1115: 정밀 내부 기준
//   - uA 레인지에서 두 ADC의 소스 노드 측정 비율로 mA 레인지 UNO 값 보정 (전환 지점 단차 제거)
constexpr float kCalMinSourceV = 0.3f;  // 비교 제외 하한 (신호 과소)
constexpr float kCalMaxSourceV = 3.0f;  // 비교 제외 상한 (5 V 전원 op amp 버퍼 포화 구간)
constexpr float kRefScaleMin = 0.94f;   // USB 5 V +-6 %, 범위 밖은 배선 이상으로 간주해 무시
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
uint32_t sumTrackCounts;  // UNO A1 카운트 합, DAC 코드와 동일 기준 비교용
uint8_t sampleCount;
uint8_t sensePga;
float switchCurrentA = -1;  // 레인지 전환 시점 uA 전류, 첫 mA 측정점 검증용
float refScale = 1.0f;      // 실제 UNO 기준전압 / 5 V, uA 레인지에서 산출
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

// 단발·단일 입력 변환, I2C 오류·시간 초과 시 false
bool adsVolts(uint8_t channel, uint8_t pga, float& volts, bool& clipped) {
  const uint16_t config = 0x8000                            // 변환 1회 시작
                          | (0x4 | channel) << 12          // AINx - GND.
                          | static_cast<uint16_t>(pga) << 9
                          | 0x0100                          // 단발 모드
                          | kAdsRate250
                          | 0x0003;                         // 비교기 OFF
  if (!adsWrite(1, config)) return false;
  const uint32_t begun = micros();
  delayMicroseconds(3500);  // 250 SPS 기준 명목 4 ms
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

// ---------- 이벤트 전송 ----------
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
  setRange(Range::ma);  // 대기 중 릴레이 코일 OFF
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
  // This range event is understood, displayed and saved by the existing phone app.
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

// 수집된 uA 측정점 기준 UNO 기준전압 비율 확정
void applyReferenceScale() {
  if (calUnoV <= 0) return;  // 유효 uA 측정점 없음 -> 명목 5 V 유지
  const float scale = calAdsV / calUnoV;
  if (scale >= kRefScaleMin && scale <= kRefScaleMax) refScale = scale;
}

// 소스 선행 하강(DUT 직전 동작점 유지) -> 릴레이 OFF
void switchToMa(float dutV, float currentA) {
  switchCurrentA = currentA;
  applyReferenceScale();
  const float targetV = dutV + currentA * kMaSenseOhms;
  long code = lroundf(ceilf(targetV / kDacVoltsPerCode)) + kDacStep;
  if (code > kLastDacCode) code = kLastDacCode;
  if (code < 0) code = 0;
  dacCode = static_cast<uint16_t>(code);
  analogWrite(A0, dacCode);  // 이 시점 전류 제한은 10 k 경로
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

// ADS1115 uA 레인지 샘플 1개 측정, 센스 이득은 측정점당 1회 결정
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
  // UNO A1·DAC 동일 USB 5 V 기준 -> 기준 오차 상쇄
  // uA 실드 버퍼 포화와 무관
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
    // DAC 설정값 = DUT 동작점 + 1 스텝 (최대 약 0.24 mA 증가)
    // 범위 크게 이탈 시 릴레이 미동작으로 판단
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

  // 두 레인지 공통 UNO ADC 고속 보호 검사
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
  pinMode(2, INPUT);  // D2 may share the sense node on the mA shield.
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
