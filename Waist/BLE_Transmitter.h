#ifndef BLE_TRANSMITTER_H
#define BLE_TRANSMITTER_H

#include <Arduino.h>
#include "Data_Struct.h"

// Khởi tạo BLE Server
void setupBLETransmitter();
// Kiểm tra trạng thái kết nối Client
bool isBLEConnected();
// Kích hoạt phiên truyền dữ liệu mỗi khi hoàn tất chu kỳ thu thập
void startBLESendingSession(const String &currentRawAct, const String &currentHMMAct, uint32_t step, float distance, float compass, const String &heading, float posX, float posY, InertialSample* samplesArray);
// Tiến trình phát xen kẽ 1 gói tin mỗi chu kỳ 20 ms
void processBLESending();

#endif // BLE_TRANSMITTER_H
