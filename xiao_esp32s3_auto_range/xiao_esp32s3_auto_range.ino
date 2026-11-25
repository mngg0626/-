#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <atomic>
#include <freertos/queue.h>
#include "lcd_graph.h"

namespace {
constexpr char kServiceUuid[] = "d161ebc1-8424-4741-b1b0-f637f2482ac4";
constexpr char kWriteUuid[] = "e21fc497-f130-41c2-bb6c-f1ec99efa6eb";
constexpr char kNotifyUuid[] = "843796ec-88dd-4aa7-b710-fa93e7b8594c";
constexpr uint32_t kBaud = 115200;
constexpr uint32_t kKeepaliveMs = 250;
constexpr uint32_t kReplyTimeoutMs = 2500;
constexpr uint32_t kNotifySpacingMs = 15;
constexpr size_t kAttPayload = 20;
constexpr size_t kFrameBytes = 320;
constexpr size_t kOutCapacity = 8;

enum class SweepState { idle, starting, running, stopping };
struct BleFragment {
  uint32_t epoch;
  uint8_t size;
  char data[kAttPayload];
};

// 프레임·측정 상태는 Arduino loop에서만 바꿈. BLE 콜백은 조각을 큐에 넣기만 함.
std::atomic<uint32_t> linkEpoch{0};  // 홀수면 폰이 연결된 상태임.
uint32_t sessionEpoch = 0;
QueueHandle_t fragments;
BLECharacteristic* notifications;
SweepState sweep = SweepState::idle;
char runId[33] = {};
uint32_t lastReply = 0;
uint32_t lastKeepalive = 0;
uint32_t lastNotify = 0;
ivlcd::Graph lcd;

struct LineBuffer {
  char data[kFrameBytes];
  size_t size = 0;
  bool discarding = false;

  void reset() {
    size = 0;
    discarding = false;
  }

  bool feed(char byte) {
    if (byte == '\n') {
      const bool complete = !discarding && size > 0;
      data[size] = '\0';
      reset();
      return complete;
    }
    if (byte != '\r' && !discarding) {
      if (byte == '\0' || size == sizeof(data) - 1) discarding = true;
      else data[size++] = byte;
    }
    return false;
  }
};

LineBuffer bleInput;
LineBuffer uartInput;

struct Notification {
  char data[kFrameBytes];
  size_t length;
};

Notification output[kOutCapacity];
size_t readAt = 0;
size_t count = 0;
size_t offset = 0;

class LinkCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { linkEpoch.fetch_add(1); }
  void onDisconnect(BLEServer*) override { linkEpoch.fetch_add(1); }
};

class WriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const uint32_t epoch = linkEpoch.load();
    const auto value = characteristic->getValue();
    if (!(epoch & 1) || value.length() == 0 || value.length() > kAttPayload) return;
    BleFragment fragment{};
    fragment.epoch = epoch;
    fragment.size = value.length();
    memcpy(fragment.data, value.c_str(), fragment.size);
    xQueueSend(fragments, &fragment, 0);
  }
};

void sendUart(JsonDocument& command) {
  char line[kFrameBytes];
  const size_t size = serializeJson(command, line, sizeof(line) - 1);
  line[size] = '\n';
  Serial1.write(reinterpret_cast<const uint8_t*>(line), size + 1);
}

void makeRequestId(char (&request)[40], const char* id, const char* type) {
  snprintf(request, sizeof(request), "%s:%s", id, type);
}

void urgentStop() {
  JsonDocument command;
  char request[40];
  makeRequestId(request, runId, "stop");
  command["proto"] = 1;
  command["type"] = "stop";
  command["id"] = runId;
  command["requestId"] = request;
  sendUart(command);
}

void queueLine(const char* line) {
  const size_t length = strlen(line);
  if (count == kOutCapacity || length >= kFrameBytes) {
    // 폰이 데이터를 못 따라오면 DUT 전원을 끊음.
    if (sweep != SweepState::idle) urgentStop();
    sweep = SweepState::idle;
    lcd.setStatus("BLE QUEUE ERROR");
    return;
  }
  auto& entry = output[(readAt + count) % kOutCapacity];
  memcpy(entry.data, line, length);
  entry.data[length] = '\n';
  entry.length = length + 1;
  ++count;
}

void queueEvent(JsonDocument& event, const char* type, const char* id) {
  event["proto"] = 1;
  event["type"] = type;
  event["id"] = id;
  char line[kFrameBytes];
  if (serializeJson(event, line, sizeof(line)) < sizeof(line)) queueLine(line);
}

void reject(const char* id, const char* type, const char* message) {
  JsonDocument event;
  char request[40];
  makeRequestId(request, id, type);
  event["command"] = type;
  event["requestId"] = request;
  event["ok"] = false;
  event["reason"] = message;
  queueEvent(event, "ack", id);
}

void handlePhoneCommand(const char* line) {
  JsonDocument command;
  if (deserializeJson(command, line, DeserializationOption::NestingLimit(3)) || command["proto"] != 1) return;
  const char* id = command["id"] | "";
  if (strlen(id) != 32 || strspn(id, "0123456789abcdef") != 32) return;
  const char* type = command["type"] | "";
  // LCD 명령은 ESP32에서만 처리하고 UNO로 넘기지 않음.
  if (strcmp(type, "backlight") == 0) {
    char request[48];
    snprintf(request, sizeof(request), "%s:backlight", id);
    if (strcmp(command["requestId"] | "", request) != 0) return;
    if (!command["enabled"].isNull() && !command["enabled"].is<bool>()) return;
    if (command["enabled"].is<bool>()) lcd.setBacklight(command["enabled"].as<bool>());
    JsonDocument reply;
    reply["enabled"] = lcd.backlight;
    queueEvent(reply, "backlight", id);
    return;
  }
  const bool start = strcmp(type, "start") == 0;
  const bool stop = strcmp(type, "stop") == 0;
  if (!start && !stop) return;
  char request[40];
  makeRequestId(request, id, type);
  if (strcmp(command["requestId"] | "", request) != 0) return;

  if (start) {
    if (sweep != SweepState::idle) {
      if (strcmp(id, runId) != 0) reject(id, "start", "Measurement already in progress");
      return;
    }
    memcpy(runId, id, sizeof(runId));
    sweep = SweepState::starting;
    lcd.reset();
    lastKeepalive = millis();
  } else {
    // stop이 마지막 측정점과 엇갈릴 수 있음. UNO는 직전 측정 ID를 아직 기억함.
    if (strcmp(id, runId) != 0) {
      reject(id, "stop", "Measurement ID not active on ESP32");
      return;
    }
    if (sweep == SweepState::stopping) return;
    sweep = SweepState::stopping;
    lcd.setStatus("STOPPING");
  }
  lastReply = millis();
  sendUart(command);
}

void handleUartEvent(const char* line) {
  JsonDocument event;
  if (sweep == SweepState::idle || deserializeJson(event, line, DeserializationOption::NestingLimit(4)) ||
      event["proto"] != 1 || strcmp(event["id"] | "", runId) != 0) return;
  const char* type = event["type"] | "";
  if (strcmp(type, "ack") == 0) {
    const char* command = event["command"] | "";
    const bool isStart = sweep == SweepState::starting && strcmp(command, "start") == 0;
    const bool isStop = sweep == SweepState::stopping && strcmp(command, "stop") == 0;
    char request[40];
    if (!isStart && !isStop) return;
    makeRequestId(request, runId, command);
    if (strcmp(event["requestId"] | "", request) != 0 || !event["ok"].is<bool>()) return;
    sweep = isStart && event["ok"].as<bool>() ? SweepState::running : SweepState::idle;
    lcd.setStatus(sweep == SweepState::running ? "MEASURING" : event["ok"].as<bool>() ? "STOPPED" : "COMMAND ERROR");
  } else if (strcmp(type, "error") == 0) {
    if (sweep == SweepState::stopping) return;  // 실제 stop ACK를 기다림.
    sweep = SweepState::idle;
    lcd.setStatus("MEASUREMENT ERROR");
  } else if (strcmp(type, "done") == 0) {
    if (sweep != SweepState::running) return;
    sweep = SweepState::idle;
    lcd.finish(event["count"] | 0U);
  } else if (strcmp(type, "point") == 0 || strcmp(type, "range") == 0) {
    if (sweep != SweepState::running && sweep != SweepState::stopping) return;
    if (strcmp(type, "range") == 0) lcd.setRange(event["range"] | "");
  } else {
    return;
  }
  if (sweep != SweepState::stopping) lastReply = millis();
  if (strcmp(type, "point") == 0) {
    const char* range = event["range"] | "";
    if (event["seq"].is<uint32_t>() && event["v"].is<float>() && event["a"].is<float>() &&
        event["sweep"] == 0 && (strcmp(range, "ma") == 0 || strcmp(range, "ua") == 0)) {
      lcd.add(event["seq"].as<uint32_t>(), event["v"].as<float>(), event["a"].as<float>());
    } else {
      lcd.dataFault = true;
      lcd.setStatus("LCD DATA ERROR");
    }
  }
  queueLine(line);
}

void handleLinkChange(uint32_t epoch) {
  if (sweep != SweepState::idle) {
    urgentStop();
    lcd.setStatus("DISCONNECTED / STOP");
  }
  sweep = SweepState::idle;
  runId[0] = '\0';
  bleInput.reset();
  uartInput.reset();
  count = 0;
  offset = 0;
  sessionEpoch = epoch;
  if (!(epoch & 1)) BLEDevice::startAdvertising();
}

void serviceTimers(uint32_t now) {
  if (sweep == SweepState::idle) return;
  if (now - lastReply >= kReplyTimeoutMs) {
    urgentStop();
    const SweepState previous = sweep;
    sweep = SweepState::idle;
    lcd.setStatus("UART TIMEOUT");
    if (previous == SweepState::stopping) {
      reject(runId, "stop", "No stop confirmation from Arduino. Check output.");
    } else {
      JsonDocument event;
      event["code"] = "UART_TIMEOUT";
      event["message"] = "No response from Arduino. Check power, TX/RX and ground.";
      queueEvent(event, "error", runId);
    }
    return;
  }
  if (sweep != SweepState::stopping && now - lastKeepalive >= kKeepaliveMs) {
    lastKeepalive = now;
    JsonDocument heartbeat;
    heartbeat["proto"] = 1;
    heartbeat["id"] = runId;
    heartbeat["type"] = "keepalive";
    sendUart(heartbeat);
  }
}

void sendNotification(uint32_t now) {
  if (!count || now - lastNotify < kNotifySpacingMs || linkEpoch.load() != sessionEpoch) return;
  lastNotify = now;
  auto& entry = output[readAt];
  const size_t remaining = entry.length - offset;
  const size_t bytes = remaining < kAttPayload ? remaining : kAttPayload;
  notifications->setValue(reinterpret_cast<uint8_t*>(entry.data + offset), bytes);
  notifications->notify();
  offset += bytes;
  if (offset == entry.length) {
    offset = 0;
    readAt = (readAt + 1) % kOutCapacity;
    --count;
  }
}
}  // namespace

void setup() {
  lcd.begin();  // 측정 전, BLE 광고 시작 전에 초기화함.
  Serial1.setRxBufferSize(2048);  // 그래프를 지우는 동안 몰려오는 point/range/done을 받아 둠.
  Serial1.begin(kBaud, SERIAL_8N1, D7, D6);
  fragments = xQueueCreate(24, sizeof(BleFragment));
  if (!fragments) return;
  BLEDevice::init("IV-Meter-mA");  // 앱이 기기를 찾도록 이름은 그대로 둠.
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new LinkCallbacks());
  BLEService* service = server->createService(kServiceUuid);
  notifications = service->createCharacteristic(kNotifyUuid, BLECharacteristic::PROPERTY_NOTIFY);
  notifications->addDescriptor(new BLE2902());
  BLECharacteristic* commands = service->createCharacteristic(kWriteUuid, BLECharacteristic::PROPERTY_WRITE);
  commands->setCallbacks(new WriteCallbacks());
  service->start();

  // 광고 설정을 BLE 스택에 맡긴 뒤 시작함.
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(kServiceUuid);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  if (!fragments) {
    delay(50);
    return;
  }
  const uint32_t epoch = linkEpoch.load();
  if (epoch != sessionEpoch) handleLinkChange(epoch);

  BleFragment fragment;
  for (int i = 0; i < 24 && xQueueReceive(fragments, &fragment, 0) == pdPASS; ++i) {
    if (fragment.epoch != sessionEpoch || !(sessionEpoch & 1)) continue;
    for (size_t index = 0; index < fragment.size; ++index) {
      if (bleInput.feed(fragment.data[index])) handlePhoneCommand(bleInput.data);
    }
  }
  for (size_t i = 0; i < kFrameBytes && Serial1.available(); ++i) {
    if (uartInput.feed(static_cast<char>(Serial1.read())) && (sessionEpoch & 1)) handleUartEvent(uartInput.data);
  }
  if ((sessionEpoch & 1) && linkEpoch.load() == sessionEpoch) {
    serviceTimers(millis());
    sendNotification(millis());
  }
  lcd.service();
  delay(1);
}
