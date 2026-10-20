#include <SdsDustSensor.h>
#include "Dust_Module.h"

namespace {
  // Cấu hình chân RX, TX cho Serial2 kết nối với SDS011
  const uint8_t SDS_RX_PIN = 27; // Kết nối chân TX của SDS011
  const uint8_t SDS_TX_PIN = 26; // Kết nối chân RX của SDS011

  // Khởi tạo đối tượng cảm biến sử dụng HardwareSerial (Serial2)
  SdsDustSensor sds(Serial2);

  // Biến lưu trữ giá trị bụi mịn và chỉ số AQI
  float current_pm25 = 0.0f;
  float current_pm10 = 0.0f;
  int current_aqi = 0;

  // Hàm nội suy tính AQI cho từng chất theo chuẩn VN-AQI (Quyết định số 1459/QĐ-TCMT của Tổng cục Môi trường Việt Nam)
  int calculateSubAQI(float value, const float bp[], const int aqis[], int size) {
    if (value <= 0.0f) return 0;
    if (value > bp[size - 1]) return aqis[size - 1]; // Vượt ngưỡng tối đa

    for (int i = 0; i < size - 1; i++) {
      if (value >= bp[i] && value <= bp[i+1]) {
        float aqi = ((float)(aqis[i+1] - aqis[i]) / (bp[i+1] - bp[i])) * (value - bp[i]) + aqis[i];
        return (int)roundf(aqi);
      }
    }
    return 0;
  }

  // Hàm tính toán AQI tổng hợp từ PM2.5 và PM10
  int computeTotalAQI(float pm25, float pm10) {
    // Định nghĩa các điểm gãy (Breakpoints) tiêu chuẩn
    const float bp_pm25[] = {0.0, 25.0, 50.0, 80.0, 150.0, 250.0, 500.0};
    const int aqi_pm25[]   = {0, 50, 100, 150, 200, 300, 500};

    const float bp_pm10[] = {0.0, 50.0, 150.0, 250.0, 350.0, 420.0, 600.0};
    const int aqi_pm10[]   = {0, 50, 100, 150, 200, 300, 500};

    int aqi_25 = calculateSubAQI(pm25, bp_pm25, aqi_pm25, 7);
    int aqi_10 = calculateSubAQI(pm10, bp_pm10, aqi_pm10, 7);

    // AQI tổng hợp là giá trị cao nhất giữa 2 chất
    return (aqi_25 > aqi_10) ? aqi_25 : aqi_10;
  }
} // namespace

void setupDust() {
  // Khởi chạy giao tiếp Serial2 nối với cảm biến với tốc độ baudrate là 9600 (mặc định của SDS011)
  // Tham số: baudrate, cấu hình, chân RX, chân TX
  Serial2.begin(9600, SERIAL_8N1, SDS_RX_PIN, SDS_TX_PIN);
  // Khởi động thư viện
  sds.begin();
  // Đợi một chút ngắn và ép cảm biến về Active Mode
  delay(1000);
  // Ép cảm biến chuyển sang chế độ Active Mode (Tự động đẩy dữ liệu) để tối ưu
  // Nếu muốn dùng Query Mode cố định, bạn có thể đổi thành sds.setQueryReportingMode()
  sds.setActiveReportingMode();
}

void processDustData() {
  // Đọc dữ liệu từ cảm biến
  PmResult pm = sds.readPm();
  if (pm.isOk()) {
    current_pm25 = pm.pm25;
    current_pm10 = pm.pm10;
    // Tự động tính toán lại chỉ số AQI khi có dữ liệu mới
    current_aqi = computeTotalAQI(current_pm25, current_pm10);
  }
}

float getPM25() { return current_pm25; }
float getPM10() { return current_pm10; }
int getDustAQI() { return current_aqi; }
