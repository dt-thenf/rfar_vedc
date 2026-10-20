#ifndef COMPASS_MODULE_H
#define COMPASS_MODULE_H

#include <Arduino.h>

// Khởi tạo module la bàn và bộ ước lượng vector trọng lực
void setupCompass();
// Tiếp nhận dữ liệu IMU 9 trục, cập nhật bộ ước lượng vector trọng lực và tính hướng
void processCompassData(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ);
// Trả về góc phương vị (0.0 - 359.9 độ)
float getCompassValue();
// Trả về hướng (N, NE, E, SE, S, SW, W, NW)
String getCompassHeading();
// Kiểm tra trạng thái sẵn sàng của la bàn (đã khởi tạo trọng lực và có dữ liệu từ trường)
bool isCompassReady();

#endif // COMPASS_MODULE_H
