#ifndef ALTITUDE_MODULE_H
#define ALTITUDE_MODULE_H

#include <Arduino.h>

// Khởi tạo cảm biến BMP280 và xác định áp suất/nhiệt độ mốc ban đầu
bool setupAltitude(int8_t startFloor = 1, float floorHeight = 2.8f);
// Tiến trình đọc cảm biến, lọc Kalman và ước lượng tầng
void processAltitudeData();
// Lấy nhiệt độ môi trường (độ C)
float getTemperature();
// Lấy áp suất khí quyển (hPa)
float getPressure();
// Lấy độ cao tương đối so với vị trí khởi tạo (m)
float getAltitude();
// Lấy số tầng ước lượng hiện tại
uint8_t getFloor();

#endif // ALTITUDE_MODULE_H
