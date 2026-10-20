#ifndef DUST_MODULE_H
#define DUST_MODULE_H

#include <Arduino.h>

// Khởi tạo phần cứng UART và cấu hình chế độ Active cho SDS011
void setupDust();
// Tiến trình kiểm tra bộ đệm UART ngầm
void processDustData();
// Trả về nồng độ bụi PM2.5 (µg/m³)
float getPM25();
// Trả về nồng độ bụi PM10 (µg/m³)
float getPM10();
// Trả về chỉ số AQI (0 - 500)
int getDustAQI();

#endif // DUST_MODULE_H
