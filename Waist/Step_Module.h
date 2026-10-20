#ifndef STEP_MODULE_H
#define STEP_MODULE_H

#include <Arduino.h>
#include "Step_Queue.h"

// Cấu hình tham số cửa sổ trượt (HALF_WINDOW = 25 -> Cửa sổ tổng = 51 mẫu)
#define HALF_WINDOW 25
// Khởi tạo module đếm bước
void setupStepCounter();
// Tiếp nhận dữ liệu gia tốc 3 trục (g), tính toán Amag-1 và thực hiện đếm bước
void processStepData(float accX, float accY, float accZ);
// Trả về số bước chân
uint32_t getStepCount();
// Trả về khoảng cách ước tính (m) dựa trên số bước chân
float getStepDistance();

#endif // STEP_MODULE_H
