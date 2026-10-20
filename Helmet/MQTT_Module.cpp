#include <WiFi.h>
#include <PubSubClient.h>
#include "MQTT_Module.h"

// Cấu hình thông số mạng và MQTT Broker
namespace {
  const char *WIFI_SSID     = "SSA_LAB";
  const char *WIFI_PASSWORD = "12345687990";
  const char *MQTT_SERVER   = "118.70.240.188";
  const int   MQTT_PORT     = 1883;
  const char *MQTT_TOPIC    = "ssalab/firefighter1";

  WiFiClient espClient;
  PubSubClient mqttClient(espClient);

  // Quản lý trạng thái kết nối
  unsigned long lastReconnectAttempt = 0;
  const unsigned long RECONNECT_INTERVAL_MS = 3000;

  // Lớp đếm kích thước chuỗi JSON trực tiếp không cấp phát bộ đệm RAM
  class CountingPrint : public Print {
    public:
    size_t count = 0;
    size_t write(uint8_t) override { count++; return 1; }
    size_t write(const uint8_t *, size_t size) override { count += size; return size; }
  };

  // Lớp ghi dữ liệu trực tiếp ra socket mạng theo từng phân đoạn nhỏ 256 bytes
  class MqttChunkedPrint : public Print {
    public:
      explicit MqttChunkedPrint(PubSubClient &client) : client_(client), used_(0), ok_(true) {}

      size_t write(uint8_t c) override {
        if (!ok_) return 0;
        buffer_[used_++] = c;
        if (used_ >= sizeof(buffer_) && !flushChunk()) return 0;
        return 1;
      }

      size_t write(const uint8_t *data, size_t size) override {
        if (!ok_ || !data) return 0;
        size_t accepted = 0;
        while (accepted < size && ok_) {
          const size_t room = sizeof(buffer_) - used_;
          const size_t remaining = size - accepted;
          const size_t copyLen = (remaining < room) ? remaining : room;
          memcpy(buffer_ + used_, data + accepted, copyLen);
          used_ += copyLen;
          accepted += copyLen;
          if (used_ >= sizeof(buffer_) && !flushChunk()) break;
        }
        return accepted;
      }

      bool finish() { return ok_ && flushChunk(); }
      bool good() const { return ok_; }

    private:
      bool flushChunk() {
        if (!ok_) return false;
        if (used_ == 0) return true;
        size_t sent = 0;
        const uint32_t started = millis();
        while (sent < used_) {
          if (!client_.connected()) { ok_ = false; return false; }
          const size_t n = client_.write(buffer_ + sent, used_ - sent);
          if (n > 0) {
            sent += n;
          } else {
            if ((uint32_t)(millis() - started) > 3000UL) { ok_ = false; return false; }
            yield(); // Nhường quyền xử lý cho tiến trình ngầm của ESP32 mà không làm trễ CPU
          }
        }
        used_ = 0;
        return true;
      }

      PubSubClient &client_;
      uint8_t buffer_[256];
      size_t used_;
      bool ok_;
  };

//  // Chuẩn hóa định dạng thời gian thực (NTP)
//  String getTimestamp() {
//    struct timeval tv;
//    gettimeofday(&tv, NULL);
//    struct tm timeinfo;
//    localtime_r(&tv.tv_sec, &timeinfo);
//    char buf[40];
//    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
//    char full[48];
//    snprintf(full, sizeof(full), "%s.%03d", buf, (int)(tv.tv_usec / 1000));
//    return String(full);
//  }

  // Xuất số thực có kiểm tra giá trị không xác định
  void printFloatExact(Print &out, float value) {
    if (!isfinite(value)) { out.print("null"); return; }
    char text[24];
    snprintf(text, sizeof(text), "%.4g", (double)value);
    out.print(text);
  }

  // Xuất mảng số thực định dạng JSON
  void printFloatArray(Print &out, const char *key, const float *values, int count) {
    out.print('"'); out.print(key); out.print("\":[");
    for (int i = 0; i < count; i++) {
      printFloatExact(out, values[i]);
      if (i + 1 < count) out.print(',');
    }
    out.print(']');
  }

  // Xuất mảng dữ liệu của một trục cụ thể
  void printAxisArray(Print &out, const char *key, const InertialSample *samples, int count, int axisIndex) {
    out.print('"'); out.print(key); out.print("\":[");
    for (int i = 0; i < count; i++) {
      float val = 0.0f;
      switch (axisIndex) {
        case 0: val = samples[i].accX; break;
        case 1: val = samples[i].accY; break;
        case 2: val = samples[i].accZ; break;
        case 3: val = samples[i].gyrX; break;
        case 4: val = samples[i].gyrY; break;
        case 5: val = samples[i].gyrZ; break;
      }
      printFloatExact(out, val);
      if (i + 1 < count) out.print(',');
    }
    out.print(']');
  }

  // Tuần tự hóa toàn bộ cấu trúc Payload JSON ra đối tượng Print
  void serializePayload(Print &out, const HelmetData &helmet, const WaistData *waist, bool waistOnline) {
    out.print("{\"id\":"); out.print(1);
    out.print(",\"helmet\":{");
    out.print("\"gasLevel\":"); out.print(helmet.gasLevel);
    out.print(",\"aqi\":"); out.print(helmet.aqi);
    out.print(",\"pm25\":"); printFloatExact(out, helmet.pm25);
    out.print(",\"pm10\":"); printFloatExact(out, helmet.pm10);
    out.print(",\"temperature\":"); printFloatExact(out, helmet.temperature);
    out.print(",\"pressure\":"); printFloatExact(out, helmet.pressure);
    out.print(",\"altitude\":"); printFloatExact(out, helmet.altitude);
    out.print(",\"floor\":"); out.print(helmet.floor);
    out.print(",\"environmentSample\":{");
    printFloatArray(out, "co", helmet.coSamples, helmet.sampleCount);
    out.print(',');
    printFloatArray(out, "smoke", helmet.smokeSamples, helmet.sampleCount);
    out.print("}},\"waist\":");

    if (waistOnline && waist != nullptr) {
      out.print("{\"rawAct\":\""); out.print(waist->rawAct);
      out.print("\",\"hmmAct\":\""); out.print(waist->hmmAct);
      out.print("\",\"stepCount\":"); out.print(waist->stepCount);
      out.print(",\"stepDistance\":"); printFloatExact(out, waist->stepDistance);
      out.print(",\"compass\":"); printFloatExact(out, waist->compass);
      out.print(",\"heading\":\""); out.print(waist->heading);
      out.print("\",\"posX\":"); printFloatExact(out, waist->posX);
      out.print(",\"posY\":"); printFloatExact(out, waist->posY);
      out.print(",\"inertialSample\":{");
      printAxisArray(out, "ax", waist->samples, SAMPLES_PER_SECOND, 0); out.print(',');
      printAxisArray(out, "ay", waist->samples, SAMPLES_PER_SECOND, 1); out.print(',');
      printAxisArray(out, "az", waist->samples, SAMPLES_PER_SECOND, 2); out.print(',');
      printAxisArray(out, "gx", waist->samples, SAMPLES_PER_SECOND, 3); out.print(',');
      printAxisArray(out, "gy", waist->samples, SAMPLES_PER_SECOND, 4); out.print(',');
      printAxisArray(out, "gz", waist->samples, SAMPLES_PER_SECOND, 5);

      out.print("}}");
    } else {
      out.print("null");
      }
    out.print('}');
  }

  // Quản lý việc kết nối lại máy chủ không chặn CPU
  void handleReconnection() {
    if (WiFi.status() != WL_CONNECTED) { return; }
    if (millis() - lastReconnectAttempt >= RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = millis();
      String clientId = "Helmet_" + String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFFULL), HEX);
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println(F("[MQTT] Đã kết nối thành công đến Broker."));
      }
    }
  }
} // namespace

void setupMQTT() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println(F("[WiFi] Đang khởi tạo kết nối..."));
  // Cấu hình thời gian qua NTP
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(2048);
}

void runMQTT() {
  if (!mqttClient.connected()) {
    handleReconnection();
  } else {
    mqttClient.loop();
  }
}

bool isMQTTConnected() { return mqttClient.connected(); }
bool publishTelemetryMQTT(const HelmetData &helmetData, const WaistData *waistData, bool isWaistOnline) {
  if (!mqttClient.connected()) { return false; }

  // Duyệt ảo tính toán chính xác dung lượng payload JSON
  CountingPrint counter;
  serializePayload(counter, helmetData, waistData, isWaistOnline);
  const size_t payloadLength = counter.count;

  // Bắt đầu phiên xuất bản với độ dài xác định (QoS 0)
  if (!mqttClient.beginPublish(MQTT_TOPIC, payloadLength, false)) { return false; }

  // Truyền dữ liệu trực tiếp qua socket bằng bộ đệm phân đoạn
  MqttChunkedPrint sink(mqttClient);
  serializePayload(sink, helmetData, waistData, isWaistOnline);

  if (!sink.finish() || !sink.good() || !mqttClient.endPublish()) {
    mqttClient.disconnect();
    return false;
  }
  return true;
}
