#include "Step_Queue.h"

// Khởi tạo hàng đợi về trạng thái ban đầu
void stepQueue_Init(StepQueue_t *q) {
  q->front = -1;
  q->rear = -1;
  q->size = STEP_QUEUE_SIZE;
  memset(q->queueArr, 0, sizeof(q->queueArr));
}

// Kiểm tra hàng đợi rỗng
uint8_t stepQueue_IsEmpty(StepQueue_t *q) {
  return (q->front == -1);
}

// Kiểm tra hàng đợi đầy
uint8_t stepQueue_IsFull(StepQueue_t *q) {
  return ((q->rear + 1) % q->size == q->front);
}

// Thêm dữ liệu mới vào hàng đợi
void stepQueue_Push(StepQueue_t *q, uint32_t index, float amagValue) {
  // Nếu hàng đợi đầy, tự động loại bỏ phần tử cũ nhất (front) để nhường chỗ cho mẫu mới
  if (stepQueue_IsFull(q)) {
    stepQueue_Pop(q);
  }

  if (stepQueue_IsEmpty(q)) {
    q->front = 0;
  }

  // Tăng chỉ số rear theo vòng tròn
  q->rear = (q->rear + 1) % q->size;

  // Nạp dữ liệu vào vị trí mới
  q->queueArr[q->rear].sampleIndex = index;
  q->queueArr[q->rear].amag = amagValue;
}

// Lấy dữ liệu cũ nhất ra khỏi hàng đợi
StepSample_t stepQueue_Pop(StepQueue_t *q) {
  StepSample_t blankData = {0, 0.0f};
  if (stepQueue_IsEmpty(q)) {
    return blankData;
  }

  StepSample_t data = q->queueArr[q->front];

  if (q->front == q->rear) { // Nếu hàng đợi chỉ còn đúng 1 phần tử
    q->front = -1;
    q->rear = -1;
  } else {
    q->front = (q->front + 1) % q->size;
  }
  return data;
}

// Xóa toàn bộ dữ liệu trong hàng đợi
void stepQueue_Clear(StepQueue_t *q) {
  q->front = -1;
  q->rear = -1;
}
