#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

#include <Arduino.h>

#define SAMPLES_PER_SECOND 50 // 50 mẫu mỗi giây (50 Hz downsampled)
#define SAMPLES_PER_PACKET 17 // 17 mẫu mỗi gói tin (tổng cộng 3 gói)

// Cấu trúc chứa 1 mẫu cảm biến chuyển động (6 số float = 24 bytes)
struct __attribute__((packed)) InertialSample {
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
};

// Cấu trúc dữ liệu tổng thể gửi qua BLE mỗi 1 giây
struct __attribute__((packed)) InertialData {
  uint8_t packetIndex;            // Số thứ tự gói (0 đến 2)
  uint8_t isLastPacket;           // Cờ báo gói cuối cùng (0: chưa hết, 1: gói cuối)
  uint8_t currentSamplesInPacket; // Số lượng mẫu thực tế trong gói này
  // Phần 1: Thông tin chung (32 bytes)
  char rawAct[4];     // Nhãn hành động thô/heuristic thời gian thực (4 bytes)
  char hmmAct[4];     // Nhãn hành động được HMM sửa lỗi từ quá khứ (4 bytes)
  uint32_t stepCount; // Số bước chân (4 bytes)
  float stepDistance; // Khoảng cách bước (4 bytes)
  float compass;      // Giá trị la bàn (4 bytes)
  char heading[4];    // Hướng đông tây nam bắc (4 bytes)
  float posX;         // Tọa độ X (4 bytes)
  float posY;         // Tọa độ Y (4 bytes)
  // Phần 2: Mảng dữ liệu quán tính trong 1 giây (17 mẫu x 24 bytes = 408 bytes)
  InertialSample samples[SAMPLES_PER_PACKET];
};

#define SERVICE_UUID        "4fafc201-75d5-4b30-80a5-f389320c1547"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#endif // DATA_STRUCT_H
