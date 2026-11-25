#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>

// mA + uA 실드를 쌓아서 쓰는 자동 레인지 측정.
//   mA 레인지: 릴레이 꺼짐 -> 센스 노드 -> 200 ohm -> 릴레이 NC(2번) -> GND
//   uA 레인지: 릴레이 켜짐 -> 센스 노드 -> 10 kohm -> 릴레이 NO(4번) -> GND (200 ohm 경로는 끊김)
// uA 레인지로 시작하고, 센스 전압이 kUaToMaSenseV를 넘으면 mA 레인지로 바꿈.
// 릴레이를 바꾸기 전에 DAC를 먼저 낮춰서, DUT에 소스 전압이 한꺼번에 걸리지 않고
// 직전 동작점 근처가 유지되게 함.
//
// start 명령의 선택 필드 "range": "auto"(기본) | "ua" | "ma".
//   auto인데 ADS1115가 없으면 mA 레인지로만 측정함.

namespace {
constexpr uint32_t kBaud = 115200;
constexpr float kAdcReferenceV = 5.0f;  // 명목값. 측정 중 ADS1115로 보정함(refScale 참고).
constexpr float kMaSenseOhms = 200.0f;
constexpr float kUaSenseOhms = 10000.0f;
constexpr float kSweepCurrentLimitA = 0.008f;  // 이 전류에 닿으면 측정을 끝냄.
constexpr float kCurrentCutoffA = 0.012f;  // 소프트웨어 차단 전류. 200 ohm 직렬 저항은 항상 달려 있음.
constexpr uint16_t kAdcMax = 16383;
constexpr float kUnoVoltsPerCount = kAdcReferenceV / kAdcMax;
constexpr float kDacVoltsPerCode = kAdcReferenceV / 1024.0f;
constexpr uint16_t kLastDacCode = 880;  // 명목 4.297 V. DUT 전압이 아니라 소스 전압임.
constexpr uint16_t kDacStep = 10;  // 명목 48.8 mV. 200 ohm에서 한 스텝당 전류 증가폭을 제한함.
constexpr float kNominalSourceMaxV = kDacVoltsPerCode * kLastDacCode;
constexpr float kOutputToleranceV = 0.25f;
static_assert(kNominalSourceMaxV <= 4.3f, "Review DUT ratings before increasing the sweep voltage");
static_assert(kSweepCurrentLimitA + kDacVoltsPerCode * kDacStep / kMaSenseOhms < kCurrentCutoffA,
              "The normal current boundary must precede the fault cutoff");
constexpr uint32_t kSettleMs = 220;  // 레인지 전환 뒤 릴레이 접점 떨림 시간도 포함함.
constexpr uint32_t kSampleSpacingMs = 20;
constexpr uint8_t kSamples = 10;
constexpr uint32_t kHeartbeatTimeoutMs = 1200;
constexpr uint32_t kCurrentCheckMs = 5;

// uA 실드의 레인지 릴레이(G6K-2P-Y): D4 -> 1 k -> Q1 베이스.
// D4 LOW, DUT 없음 상태에서 A2-GND가 약 200 ohm이면 true, 약 10 kohm이면 false로 바꿈.
constexpr uint8_t kRangeRelayPin = 4;
constexpr bool kUaWhenRelayHigh = true;
constexpr float kUaToMaSenseV = 1.3f;  // 10 kohm 기준 130 uA.
// uA 레인지에서 센스 노드는 kUaToMaSenseV + 한 스텝을 넘지 않음. 그보다 높으면
// 200 ohm 경로가 아직 붙어 있다는 뜻이므로 8 mA 전에 릴레이 고장으로 처리함.
constexpr float kUaFaultSenseV = kSweepCurrentLimitA * kMaSenseOhms;  // 1.6 V
static_assert(kUaToMaSenseV + kDacVoltsPerCode * kDacStep < kUaFaultSenseV * 0.9f,
              "Leave margin between the uA switch point and the relay fault check");

// uA 실드의 ADS1115. ADDR이 GND에 연결돼 주소는 0x48임.
constexpr uint8_t kAdsAddress = 0x48;
constexpr uint8_t kAdsSource = 0;  // 버퍼를 거친 소스 노드.
constexpr uint8_t kAdsSense = 1;   // 버퍼를 거친 센스 노드(A2와 같은 노드).
constexpr float kAdsFullScaleV[] = {6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f};
constexpr uint8_t kAdsWidestPga = 0;
constexpr uint8_t kAdsNarrowestPga = 5;
constexpr uint16_t kAdsRate250 = 0x5 << 5;
constexpr uint32_t kAdsTimeoutUs = 20000;

// UNO ADC는 USB 5 V를 기준으로 쓰고, ADS1115는 정밀한 내부 기준을 씀.
// uA 레인지에서 둘 다 소스 노드를 읽으므로 그 비율로 mA 레인지의 UNO 값을 보정함.
// 보정이 없으면 레인지가 바뀌는 지점에서 곡선에 작은 단차가 생김.
constexpr float kCalMinSourceV = 0.3f;  // 너무 작은 값은 비교에서 뺌.
constexpr float kCalMaxSourceV = 3.0f;  // 5 V 전원 op amp 버퍼가 포화할 수 있는 구간은 뺌.
constexpr float kRefScaleMin = 0.94f;   // USB 5 V +-6 %. 이 범위 밖이면 배선 이상으로 보고 무시함.
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
uint32_t sumTrackCounts;  // UNO A1 카운트 합. DAC 코드와 같은 기준으로 비교함.
uint8_t sampleCount;
uint8_t sensePga;
float switchCurrentA = -1;  // 레인지 전환 때의 uA 전류. 첫 mA 측정점에서 확인함.
float refScale = 1.0f;      // 실제 UNO 기준전압 / 5 V. uA 레인지에서 구함.
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

// 단발, 단일 입력 변환. I2C 오류나 시간 초과면 false를 돌려줌.
bool adsVolts(uint8_t channel, uint8_t pga, float& volts, bool& clipped) {
  const uint16_t config = 0x8000                            // 변환 1회 시작.
                          | (0x4 | channel) << 12          // AINx - GND.
                          | static_cast<uint16_t>(pga) << 9
                          | 0x0100                          // 단발 모드.
                          | kAdsRate250
                          | 0x0003;                         // 비교기 끔.
  if (!adsWrite(1, config)) return false;
  const uint32_t begun = micros();
  delayMicroseconds(3500);  // 250 SPS에서 명목 4 ms.
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
  setRange(Range::ma);  // 대기 중에는 릴레이 코일을 끔.
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
  // 완료는 설정한 전압 또는 전류 한계에 닿았다는 뜻임.
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

// 지금까지 모은 uA 레인지 측정점으로 UNO 기준전압 비율을 확정함.
void applyReferenceScale() {
  if (calUnoV <= 0) return;  // 쓸 만한 uA 측정점이 없으면 명목 5 V를 유지함.
  const float scale = calAdsV / calUnoV;
  if (scale >= kRefScaleMin && scale <= kRefScaleMax) refScale = scale;
}

// DUT가 직전 동작점을 유지하도록 소스를 먼저 낮추고, 그다음 릴레이를 끔.
void switchToMa(float dutV, float currentA) {
  switchCurrentA = currentA;
  applyReferenceScale();
  const float targetV = dutV + currentA * kMaSenseOhms;
  long code = lroundf(ceilf(targetV / kDacVoltsPerCode)) + kDacStep;
  if (code > kLastDacCode) code = kLastDacCode;
  if (code < 0) code = 0;
  dacCode = static_cast<uint16_t>(code);
  analogWrite(A0, dacCode);  // 이 시점에는 10 k 경로가 아직 전류를 제한함.
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
  // 한 번에 처리할 양을 제한해서 시리얼 입력 때문에 감시 처리가 밀리지 않게 함.
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

// ADS1115로 uA 레인지 샘플 1개를 읽음. 센스 이득은 측정점마다 한 번 정함.
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
  // UNO A1과 DAC는 같은 USB 5 V를 기준으로 쓰므로 그 오차가 상쇄됨.
  // uA 실드 버퍼가 포화해도 이 검사에는 영향이 없음.
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
    // DAC는 DUT 동작점에 한 스텝을 더한 값으로 맞춰져 있음(최대 약 0.24 mA 증가).
    // 이 범위를 크게 벗어나면 릴레이가 움직이지 않은 것임.
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

  // 두 레인지 모두 UNO ADC로 빠르게 보호 검사함.
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
      // 안정되지 않았거나 한계를 넘은 점은 보내지 않음. 마지막으로 완료된 평균값까지만 남김.
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
  pinMode(2, INPUT);  // mA 실드에서 D2가 센스 노드와 연결돼 있어 입력으로 둠.
  pinMode(kRangeRelayPin, OUTPUT);
  analogWriteResolution(10);
  analogReadResolution(14);
  zeroOutput();
  Wire.begin();
  Wire.setClock(400000);
  Serial1.begin(kBaud);  // D0 RX / D1 TX. USB Serial과 별개임.
}

void loop() {
  if (measuring && millis() - lastHeartbeat >= kHeartbeatTimeoutMs) measure(millis());
  readCommands();
  if (measuring) measure(millis());
}
