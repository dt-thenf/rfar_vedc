#ifndef HTTP_MODULE_H
#define HTTP_MODULE_H

#include <Arduino.h>
#include "Data_Struct.h"
#include "MQTT_Module.h"

// Khởi tạo các cấu hình cho HTTPS Client
void setupHTTP();
// Hàm đóng gói URL và thực hiện gửi dữ liệu lên máy chủ XAMPP (chạy sau sự kiện MQTT)
bool sendTelemetryHTTPS(const HelmetData &helmetData, const WaistData *waistData, bool isWaistOnline);

#endif // HTTP_MODULE_H
