#pragma once

#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include <math.h>

// XIAO labels, not raw GPIO numbers: D5 is GPIO6; D4 is GPIO5.
namespace ivlcd {
constexpr int kBacklightPin = D5;
constexpr size_t kCapacity = 256;  // At most 89 points plus the points repeated after a uA->mA handoff.
struct Point { float v; float a; };  // Volts, amps.

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
  void setRange(const char* value) { range = value; dirty = true; }

  void reset() {
    count = 0;
    dataFault = false;
    rendering = false;
    range = "--";
    setStatus("STARTING");
  }

  void add(uint32_t seq, float v, float a) {
    if (seq < count) {
      if (points[seq].v != v || points[seq].a != a) dataFault = true;
      return;
    }
    if (seq != count || count == kCapacity || !isfinite(v) || !isfinite(a)) {
      dataFault = true;
      setStatus("LCD DATA ERROR");
      return;
    }
    points[count++] = {v, a};
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
      const int16_t y = py(points[nextPoint].a);
      if (nextPoint) tft.drawLine(px(points[nextPoint - 1].v), py(points[nextPoint - 1].a), x, y, ILI9341_CYAN);
      tft.drawPixel(x, y, ILI9341_CYAN);
    }
    if (nextPoint == drawnCount) rendering = false;
  }

 private:
  bool dirty = true;
  bool rendering = false;
  size_t drawnCount = 0, nextPoint = 0;
  float vLow = 0, vHigh = 1, iLow = 0, iHigh = 1;  // Current axis in amps.
  float iScale = 1000.0f;                             // Amps -> displayed unit (mA or uA).
  static constexpr int16_t left = 46, top = 45, width = 258, height = 143;
  int16_t px(float v) const { return left + lroundf((v - vLow) * width / (vHigh - vLow)); }
  int16_t py(float a) const { return top + height - lroundf((a - iLow) * height / (iHigh - iLow)); }

  // Below 1 mA the curve reads better in uA.
  static void formatCurrent(char* out, size_t size, float a) {
    if (fabsf(a) < 0.001f) snprintf(out, size, "%.2f uA", a * 1e6f);
    else snprintf(out, size, "%.3f mA", a * 1e3f);
  }

  void text(int16_t x, int16_t y, const char* value, uint16_t color = ILI9341_WHITE) {
    tft.setCursor(x, y);
    tft.setTextColor(color, ILI9341_BLACK);
    tft.print(value);
  }

  void axes() {
    // Autoscale from the same accepted V/A samples sent to the phone.
    vLow = 0;
    vHigh = 1;
    iLow = 0;
    iHigh = 0.001f;
    if (drawnCount) {
      vLow = vHigh = points[0].v;
      iLow = iHigh = points[0].a;
      for (size_t i = 1; i < drawnCount; ++i) {
        vLow = fminf(vLow, points[i].v); vHigh = fmaxf(vHigh, points[i].v);
        iLow = fminf(iLow, points[i].a); iHigh = fmaxf(iHigh, points[i].a);
      }
      const float dv = vHigh == vLow ? fmaxf(fabsf(vLow) * 0.05f, 0.01f) : (vHigh - vLow) * 0.06f;
      const float di = iHigh == iLow ? fmaxf(fabsf(iLow) * 0.05f, 1e-9f) : (iHigh - iLow) * 0.08f;
      vLow -= dv; vHigh += dv; iLow -= di; iHigh += di;
    }
    const bool micro = fmaxf(fabsf(iLow), fabsf(iHigh)) < 0.001f;
    iScale = micro ? 1e6f : 1e3f;
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(2);
    text(8, 4, "I-V CURVE", ILI9341_CYAN);
    tft.setTextSize(1);
    text(8, 26, dataFault ? "LCD DATA ERROR" : status);
    text(200, 26, "RANGE ");
    text(236, 26, range, ILI9341_YELLOW);
    text(1, 36, iScale > 1e4f ? "I(uA)" : "I(mA)");
    char label[48];
    for (int tick = 0; tick <= 4; ++tick) {
      const int16_t x = left + tick * width / 4;
      const int16_t y = top + height - tick * height / 4;
      tft.drawFastVLine(x, top, height + 1, 0x3186);
      tft.drawFastHLine(left, y, width + 1, 0x3186);
      snprintf(label, sizeof(label), "%.2f", vLow + (vHigh - vLow) * tick / 4);
      text(x - 13, top + height + 5, label);
      snprintf(label, sizeof(label), "%.2f", (iLow + (iHigh - iLow) * tick / 4) * iScale);
      text(1, y - 3, label);
    }
    text(286, 207, "V(V)");
    snprintf(label, sizeof(label), "%u points", static_cast<unsigned>(drawnCount));
    text(8, 212, label);
    if (drawnCount) {
      char current[24];
      formatCurrent(current, sizeof(current), points[drawnCount - 1].a);
      snprintf(label, sizeof(label), "V=%.3f  I=%s", points[drawnCount - 1].v, current);
      text(8, 228, label);
    }
  }
};
}  // namespace ivlcd
