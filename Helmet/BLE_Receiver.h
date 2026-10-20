#ifndef BLE_RECEIVER_H
#define BLE_RECEIVER_H

#include <Arduino.h>
#include "Data_Struct.h"

// Cấu hình trạng thái kết nối và bộ đệm dữ liệu nhận từ thiết bị waist
struct WaistData {
  char rawAct[4];     // Nhãn hành động thô/heuristic thời gian thực (4 bytes)
  char hmmAct[4];     // Nhãn hành động được HMM sửa lỗi từ quá khứ (4 bytes)
  uint32_t stepCount; // Số bước chân (4 bytes)
  float stepDistance; // Khoảng cách bước (4 bytes)
  float compass;      // Giá trị la bàn (4 bytes)
  char heading[4];    // Hướng đông tây nam bắc (4 bytes)
  float posX;         // Tọa độ X (4 bytes)
  float posY;         // Tọa độ Y (4 bytes)
  InertialSample samples[SAMPLES_PER_SECOND];
  uint32_t lastReceivedTimeMs;
};

// Khởi tạo tiến trình quét và kết nối NimBLE Client
void setupBLEReceiver();
// Tiến trình duy trì kết nối và tự động quét lại (gọi định kỳ trong loop)
void runBLEReceiver();
// Kiểm tra trạng thái liên kết BLE với thiết bị waist
bool isWaistConnected();
// Kiểm tra xem có khối dữ liệu 1 giây hoàn chỉnh mới được ghép xong không
bool isWaistDataReady();
// Đọc khối dữ liệu snapshot mới nhất (tự động xóa cờ báo sẵn sàng sau khi đọc)
bool getwaistSnapshot(WaistData &outputData);

#endif // BLE_RECEIVER_H
