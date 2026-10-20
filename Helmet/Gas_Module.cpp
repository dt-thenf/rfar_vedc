#include "Gas_Module.h"
#include "micromlgen_RF.h"

namespace {
  // Cấu hình chân phần cứng cảm biến khí
  const uint8_t MQ9_PIN = 34;
  const uint8_t MQ135_PIN = 35;

  // Đối tượng mô hình phân loại hỏa hoạn
  Eloquent::ML::Port::FireClassifier model_RF;

  // Bộ đệm xuất bản dữ liệu (1 giây = 50 mẫu)
  float co_js_buffer[GAS_JS_WINDOW_SIZE];
  float smoke_js_buffer[GAS_JS_WINDOW_SIZE];
  uint8_t js_index = 0;

  // Bộ đệm trích xuất đặc trưng (2 giây = 100 mẫu)
  float co_ml_window[GAS_ML_WINDOW_SIZE];
  float smoke_ml_window[GAS_ML_WINDOW_SIZE];
  uint8_t ml_index = 0;

  // Biến lưu trữ kết quả phân loại và trạng thái cảnh báo
  uint8_t current_gas_level = 0;
  uint8_t current_gas_alert = 0;

  // Hàm chuyển đổi điện áp ADC sang PPM cho cảm biến MQ-9
  float calculateMQ9_PPM() {
    float v = (analogRead(MQ9_PIN) / 4095.0f) * 3.3f;
    if (v <= 0.0001f) return 0.0f;
    return 599.65f * powf(((5.0f - v) / v / 2.217f), -2.244f);
  }

  // Hàm chuyển đổi điện áp ADC sang PPM cho cảm biến MQ-135
  float calculateMQ135_PPM() {
    float v = (analogRead(MQ135_PIN) / 4095.0f) * 3.3f;
    if (v <= 0.0001f) return 0.0f;
    return 110.47f * powf(((5.0f - v) / v / 19.674f), -2.862f);
  }

  // Trích xuất đặc trưng
  void extractFeatures(const float* data, int n, float* out, int offset) {
    float sum = 0.0f, sq_sum = 0.0f;
    float max_v = data[0], min_v = data[0];
    for (int i = 0; i < n; i++) {
      float val = data[i];
      sum += val;
      sq_sum += val * val;
      if (val > max_v) max_v = val;
      if (val < min_v) min_v = val;
    }
    float mean = sum / (float)n;
    out[offset + 0] = mean;
    out[offset + 1] = sqrtf(fmaxf(0.0f, (sq_sum / (float)n) - (mean * mean)));
    out[offset + 2] = sqrtf(sq_sum / (float)n);
    out[offset + 3] = max_v;
    out[offset + 4] = min_v;
    out[offset + 5] = max_v - min_v;
  }
} // namespace

void setupGas() {
  pinMode(MQ9_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);

  js_index = 0, ml_index = 0;
  current_gas_level = 0;
  current_gas_alert = 0;

  memset(co_js_buffer, 0, sizeof(co_js_buffer));
  memset(smoke_js_buffer, 0, sizeof(smoke_js_buffer));
  memset(co_ml_window, 0, sizeof(co_ml_window));
  memset(smoke_ml_window, 0, sizeof(smoke_ml_window));
}

void processGasData() {
  float co_ppm = calculateMQ9_PPM();
  float smoke_ppm = calculateMQ135_PPM();

  // Tích lũy vào bộ đệm xuất bản dữ liệu
  co_js_buffer[js_index] = co_ppm;
  smoke_js_buffer[js_index] = smoke_ppm;
  js_index++;
  if (js_index >= GAS_JS_WINDOW_SIZE) {
    js_index = 0;
  }

  // Tích lũy vào cửa sổ học máy
  if (ml_index < GAS_ML_WINDOW_SIZE) {
    co_ml_window[ml_index] = co_ppm;
    smoke_ml_window[ml_index] = smoke_ppm;
    ml_index++;
  }

  // Thực thi suy luận
  if (ml_index >= GAS_ML_WINDOW_SIZE) {
    float ml_features[12];
    extractFeatures(co_ml_window, GAS_ML_WINDOW_SIZE, ml_features, 0);
    extractFeatures(smoke_ml_window, GAS_ML_WINDOW_SIZE, ml_features, 6);
    current_gas_level = (uint8_t)model_RF.predict(ml_features);
    current_gas_alert = (current_gas_level >= 3) ? 1 : 0;
    // Serial.printf("[GAS] Level: %u | State: %s (%u)\n", current_gas_level, current_gas_alert == 1 ? "Alert" : "Normal", current_gas_alert);
    ml_index = 0;
  }
}

uint8_t getGasLevel() { return current_gas_level; }
uint8_t getGasAlert() { return current_gas_alert; }
const float* getCoSamples() { return co_js_buffer; }
const float* getSmokeSamples() { return smoke_js_buffer; }
uint8_t getGasSampleCount() { return GAS_JS_WINDOW_SIZE; }
