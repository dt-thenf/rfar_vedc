#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

#include <Arduino.h>
#include "BLE_Receiver.h"

// Định nghĩa cấu hình mạng và thông số tại thiết bị helmet
struct HelmetData {
  uint8_t gasLevel;           // Mức độ khí độc (1 byte)
  int aqi;                    // Chỉ số chất lượng không khí (4 bytes)
  float pm25;                 // Bụi PM2.5 (4 bytes)
  float pm10;                 // Bụi PM10 (4 bytes)
  float temperature;          // Nhiệt độ môi trường (4 bytes)
  float pressure;             // Áp suất khí quyển (4 bytes)
  float altitude;             // Độ cao (4 bytes)
  uint8_t floor;              // Tầng nhà (1 byte)
  const float *coSamples;     // Con trỏ mảng dữ liệu chuỗi thời gian CO
  const float *smokeSamples;  // Con trỏ mảng dữ liệu chuỗi thời gian khói
  uint8_t sampleCount;        // Số lượng mẫu trong cửa sổ (1 byte)
};

// Khởi tạo kết nối Wi-Fi, NTP và cấu hình client MQTT
void setupMQTT();
// Tiến trình duy trì kết nối mạng và xử lý hàng đợi (gọi định kỳ trong loop)
void runMQTT();
// Kiểm tra trạng thái sẵn sàng của đường truyền MQTT
bool isMQTTConnected();
// Xuất bản bản tin dữ liệu môi trường và dữ liệu quán tính tổng hợp lên Broker
bool publishTelemetryMQTT(const HelmetData &helmetData, const WaistData *waistData, bool isWaistOnline);

#endif // MQTT_MODULE_H
