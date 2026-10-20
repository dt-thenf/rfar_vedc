#ifndef GAS_MODULE_H
#define GAS_MODULE_H

#include <Arduino.h>

// Định nghĩa kích thước cửa sổ
#define GAS_JS_WINDOW_SIZE 50
#define GAS_ML_WINDOW_SIZE 100

// Khởi tạo phần cứng và bộ đệm cảm biến khí
void setupGas();
// Thu thập mẫu ADC, tích lũy bộ đệm và thực hiện suy luận mô hình học máy
void processGasData();
// Trả về mức độ phân loại khí độc
uint8_t getGasLevel();
// Trả về tình trạng (0: Normal nếu gasLevel < 3; 1: Alert nếu gasLevel >= 3)
uint8_t getGasAlert();
// Trả về con trỏ mảng 50 mẫu chuỗi thời gian nồng độ CO
const float* getCoSamples();
// Trả về con trỏ mảng 50 mẫu chuỗi thời gian nồng độ khói
const float* getSmokeSamples();
// Trả về kích thước mảng dữ liệu xuất bản
uint8_t getGasSampleCount();

#endif // GAS_MODULE_H
