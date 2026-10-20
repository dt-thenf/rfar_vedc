#ifndef POSITIONING_MODULE_H
#define POSITIONING_MODULE_H

#include <Arduino.h>

// Cấu trúc lưu trữ dữ liệu vị trí và chẩn đoán định vị PDR
struct PDRPosition {
  float xM;
  float yM;
  float xMaxM;
  float yMaxM;

  float stepLengthM;
  String lockedHeading;
  bool isMoving;
  bool isBoundaryHit;

  // Dữ liệu chẩn đoán nội bộ
  float rawXM;
  float rawYM;
  float distanceSinceAnchorM;
  float headingSpreadDeg;
  float relativeConfidence;
  uint32_t boundaryHitCount;
  uint32_t turnWhileMovingCount;
  String pdrState;
};

// Khởi tạo định vị 2D và thiết lập vị trí gốc mặc định (50 m, 25 m)
void setupPositioning();
// Cập nhật tọa độ định vị PDR theo chu kỳ lấy mẫu hệ thống
void processPositioning(const String &rawAct, uint32_t totalSteps, float totalDistanceM, float headingDeg, float gyroMagnitudeDps, bool isCompassReady);
// Lấy toàn bộ cấu trúc trạng thái vị trí
PDRPosition getPDRPosition();
// Lấy tọa độ X và Y phục vụ truyền thông (đã giới hạn trong không gian 100 x 50 m)
float getPositionX();
float getPositionY();
// Đặt lại tọa độ vị trí tức thời
void resetPosition(float xM, float yM);

#endif // POSITIONING_MODULE_H
