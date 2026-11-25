#pragma once

#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

// XIAO labels, not raw GPIO numbers: D5 is GPIO6; D4 is GPIO5.
namespace ivlcd {
constexpr int kBacklightPin = D5;
constexpr size_t kCapacity = 256;  // 자동 레인지 측정: uA 최대 89점 + mA 최대 89점
struct Point { float v; float ma; };

class Graph {
 public:
  Adafruit_ILI9341 tft{&SPI, D3, D1, D4};
  Point points[kCapacity]{};
  size_t count = 0;
  bool backlight = true;
  bool dataFault = false;
  const char* status = "READY";
  const char* range = "--";

  void begin() {
    digitalWrite(kBacklightPin, LOW);
    pinMode(kBacklightPin, OUTPUT);
    SPI.begin(D8, -1, D10, D1);  // SCK, unused MISO, MOSI, CS.
    tft.begin(16000000);          // Conservative speed for breadboard jumpers.
    tft.setRotation(1);          // 320 x 240 landscape.
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextWrap(false);
    setBacklight(true);
  }

  void setBacklight(bool enabled) {
    backlight = enabled;
    digitalWrite(kBacklightPin, enabled ? HIGH : LOW);
    dirty = true;  // Redraw the latest data when light returns.
  }

  void setStatus(const char* value) { status = value; dirty = true; }

  // UNO 레인지 이름 수신, 문자열 상수만 저장 (JSON 버퍼 임시)
  void setRange(const char* value) {
    range = strcmp(value, "ua") == 0 ? "uA" : strcmp(value, "ma") == 0 ? "mA" : "--";
    dirty = true;
  }

  void reset() {
    count = 0;
    dataFault = false;
    rendering = false;
    range = "--";
    setStatus("STARTING");
  }

  void add(uint32_t seq, float v, float a) {
    if (seq < count) {
      if (points[seq].v != v || points[seq].ma != a * 1000.0f) dataFault = true;
      return;
    }
    if (seq != count || count == kCapacity || !isfinite(v) || !isfinite(a)) {
      dataFault = true;
      setStatus("LCD DATA ERROR");
      return;
    }
    points[count++] = {v, a * 1000.0f};
    dirty = true;
  }

  void finish(uint32_t expected) {
    setStatus(!dataFault && expected > 0 && expected == count ? "COMPLETE" : "INCOMPLETE");
  }

  // Called after UART/BLE/watchdog work. A redraw is split across loop turns.
  void service() {
    if (!backlight) return;
    if (!rendering) {
      if (!dirty) return;
      dirty = false;
      drawnCount = count;
      nextPoint = 0;
      axes();
      rendering = true;
      return;
    }
    for (int budget = 0; budget < 12 && nextPoint < drawnCount; ++budget, ++nextPoint) {
      const int16_t x = px(points[nextPoint].v);
      const int16_t y = py(points[nextPoint].ma);
      if (nextPoint) tft.drawLine(px(points[nextPoint - 1].v), py(points[nextPoint - 1].ma), x, y, ILI9341_CYAN);
      tft.drawPixel(x, y, ILI9341_CYAN);
    }
    if (nextPoint == drawnCount) rendering = false;
  }

 private:
  bool dirty = true;
  bool rendering = false;
  size_t drawnCount = 0, nextPoint = 0;
  float vLow = 0, vHigh = 1, iLow = 0, iHigh = 1;
  static constexpr int16_t left = 46, top = 45, width = 258, height = 143;
  int16_t px(float v) const { return left + lroundf((v - vLow) * width / (vHigh - vLow)); }
  int16_t py(float ma) const { return top + height - lroundf((ma - iLow) * height / (iHigh - iLow)); }

  void text(int16_t x, int16_t y, const char* value, uint16_t color = ILI9341_WHITE) {
    tft.setCursor(x, y);
    tft.setTextColor(color, ILI9341_BLACK);
    tft.print(value);
  }

  void axes() {
    // Autoscale from the same accepted V/A samples sent to the phone.
    vLow = iLow = 0;
    vHigh = iHigh = 1;
    if (drawnCount) {
      vLow = vHigh = points[0].v;
      iLow = iHigh = points[0].ma;
      for (size_t i = 1; i < drawnCount; ++i) {
        vLow = fminf(vLow, points[i].v); vHigh = fmaxf(vHigh, points[i].v);
        iLow = fminf(iLow, points[i].ma); iHigh = fmaxf(iHigh, points[i].ma);
      }
      const float dv = vHigh == vLow ? fmaxf(fabsf(vLow) * 0.05f, 0.01f) : (vHigh - vLow) * 0.06f;
      const float di = iHigh == iLow ? fmaxf(fabsf(iLow) * 0.05f, 0.000001f) : (iHigh - iLow) * 0.08f;
      vLow -= dv; vHigh += dv; iLow -= di; iHigh += di;
    }
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(2);
    text(8, 4, "I-V CURVE", ILI9341_CYAN);
    tft.setTextSize(1);
    text(8, 26, dataFault ? "LCD DATA ERROR" : status);
    char label[48];
    snprintf(label, sizeof(label), "RANGE %s", range);
    text(262, 8, label, ILI9341_YELLOW);
    // 점은 mA 단위 저장, 1 mA 미만 시 축 uA 표시
    const bool micro = fmaxf(fabsf(iLow), fabsf(iHigh)) < 1.0f;
    const float unit = micro ? 1000.0f : 1.0f;
    text(1, 36, micro ? "I(uA)" : "I(mA)");
    for (int tick = 0; tick <= 4; ++tick) {
      const int16_t x = left + tick * width / 4;
      const int16_t y = top + height - tick * height / 4;
      tft.drawFastVLine(x, top, height + 1, 0x3186);
      tft.drawFastHLine(left, y, width + 1, 0x3186);
      snprintf(label, sizeof(label), "%.2f", vLow + (vHigh - vLow) * tick / 4);
      text(x - 13, top + height + 5, label);
      snprintf(label, sizeof(label), micro ? "%.1f" : "%.2f", (iLow + (iHigh - iLow) * tick / 4) * unit);
      text(1, y - 3, label);
    }
    text(286, 207, "V(V)");
    snprintf(label, sizeof(label), "%u points", static_cast<unsigned>(drawnCount));
    text(8, 212, label);
    if (drawnCount) {
      const float lastMa = points[drawnCount - 1].ma;
      if (fabsf(lastMa) < 1.0f) {
        snprintf(label, sizeof(label), "V=%.3f  I=%.2f uA", points[drawnCount - 1].v, lastMa * 1000.0f);
      } else {
        snprintf(label, sizeof(label), "V=%.3f  I=%.3f mA", points[drawnCount - 1].v, lastMa);
      }
      text(8, 228, label);
    }
  }
};
}  // namespace ivlcd
