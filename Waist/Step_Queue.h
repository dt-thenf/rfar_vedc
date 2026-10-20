#ifndef STEP_QUEUE_H
#define STEP_QUEUE_H

#include <stdint.h>
#include "Arduino.h"

// Cấu hình kích thước hàng đợi bước chân
// Với HALF_WINDOW = 25, cửa sổ trượt cần tối thiểu 51 mẫu, đặt Size = 60 để có biên dư dả an toàn
#define STEP_QUEUE_SIZE 60 

// Cấu trúc dữ liệu lưu trữ một mẫu gia tốc RMS
typedef struct {
  uint32_t sampleIndex; // Chỉ mục thời gian (tăng dần mỗi 10ms)
  float amag;           // Giá trị độ lớn gia tốc tổng hợp (đã trừ đi trọng lực)
} StepSample_t;

// Cấu trúc hàng đợi vòng cấp phát tĩnh
typedef struct {
  int16_t front;
  int16_t rear;
  int16_t size;
  StepSample_t queueArr[STEP_QUEUE_SIZE];
} StepQueue_t;

// Khai báo các hàm quản lý hàng đợi
void stepQueue_Init(StepQueue_t *q);
uint8_t stepQueue_IsEmpty(StepQueue_t *q);
uint8_t stepQueue_IsFull(StepQueue_t *q);
void stepQueue_Push(StepQueue_t *q, uint32_t index, float amagValue);
StepSample_t stepQueue_Pop(StepQueue_t *q);
void stepQueue_Clear(StepQueue_t *q);

#endif // STEP_QUEUE_H
