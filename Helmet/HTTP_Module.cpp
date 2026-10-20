#include <WiFi.h>
#include <time.h>
#include <sys/time.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "HTTP_Module.h"

namespace {
  const char *URL = "https://smartsensingapp.phenikaa-uni.edu.vn/firefighter/APIdata/GetInertial.php?data=";

  // Hàm tạo chuỗi dấu thời gian theo định dạng YYMMDDHHMMSS
  // Tham số offset_seconds cho phép lùi thời gian về quá khứ để đồng bộ luồng HMM
  String getTimestamp(int offset_seconds = 0) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    // Áp dụng độ lệch thời gian (nếu có)
    time_t raw_time = tv.tv_sec - offset_seconds;
    struct tm timeinfo;
    localtime_r(&raw_time, &timeinfo);
    char buf[24];
    // %y: 2 số cuối năm, %m: tháng, %d: ngày, %H: giờ, %M: phút, %S: giây
    strftime(buf, sizeof(buf), "%y%m%d%H%M%S", &timeinfo);
    return String(buf);
  }

  // Hàm mã hóa URL chuẩn theo RFC 3986
  String urlEncode(const String &input) {
    String output;
    output.reserve(input.length() * 2);
    const char *hex = "0123456789ABCDEF";
    for (size_t i = 0; i < input.length(); i++) {
      unsigned char c = input[i];
      if ((c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9') ||
       c == '-' || c == '_'  || c == '.' || c == '~') {
      output += (char)c;
      } else {
        output += '%';
        output += hex[(c >> 4) & 0x0F];
        output += hex[c & 0x0F];
      }
    }
    return output;
  }

  // Hàm bổ trợ xuất mảng số thực cho chuỗi JSON của HTTP
  String buildFloatArrayJson(const float *arr, int count) {
    String s = "[";
    s.reserve(count * 6);
    for (int i = 0; i < count; i++) {
      s += String(arr[i], 3);
      if (i + 1 < count) s += ",";
    }
    s += "]";
    return s;
  }

  // Hàm bổ trợ xuất mảng trục cảm biến quán tính
  String buildAxisArrayJson(const InertialSample *samples, int count, int axis) {
    String s = "[";
    s.reserve(count * 6);
    for (int i = 0; i < count; i++) {
      float val = 0.0f;
      switch (axis) {
        case 0: val = samples[i].accX; break;
        case 1: val = samples[i].accY; break;
        case 2: val = samples[i].accZ; break;
        case 3: val = samples[i].gyrX; break;
        case 4: val = samples[i].gyrY; break;
        case 5: val = samples[i].gyrZ; break;
      }
      s += String(val, 3);
      if (i + 1 < count) s += ",";
    }
    s += "]";
    return s;
  }
} // namespace

void setupHTTP() { 
  // Không giữ cấu hình tĩnh để tránh chiếm dụng vùng nhớ heap
}

bool sendTelemetryHTTPS(const HelmetData &helmetData, const WaistData *waistData, bool isWaistOnline) {
  if (WiFi.status() != WL_CONNECTED) { return false; }

  // Lấy thời gian thực hiện tại và tính toán lùi mốc thời gian
  String currentTimeLabel = getTimestamp(0);
  String hmmTimeLabel = getTimestamp(6);

  // Tạo chuỗi JSON
  String jsonPayload = "{\"id\":" + String(1) + ",";
  jsonPayload += "\"LabelTime\":\"" + currentTimeLabel + "\",";
  jsonPayload += "\"HMMsLabelTime\":\"" + hmmTimeLabel + "\",";
  jsonPayload += "\"helmet\":{";
  jsonPayload += "\"gasLevel\":" + String(helmetData.gasLevel) + ",";
  jsonPayload += "\"aqi\":" + String(helmetData.aqi) + ",";
  jsonPayload += "\"pm25\":" + String(helmetData.pm25, 2) + ",";
  jsonPayload += "\"pm10\":" + String(helmetData.pm10, 2) + ",";
  jsonPayload += "\"temperature\":" + String(helmetData.temperature, 1) + ",";
  jsonPayload += "\"pressure\":" + String(helmetData.pressure, 1) + ",";
  jsonPayload += "\"altitude\":" + String(helmetData.altitude, 2) + ",";
  jsonPayload += "\"floor\":" + String(helmetData.floor); // + ","
//  jsonPayload += "\"environmentSample\":{";
//  jsonPayload += "\"co\":" + buildFloatArrayJson(helmetData.coSamples, helmetData.sampleCount) + ",";
//  jsonPayload += "\"smoke\":" + buildFloatArrayJson(helmetData.smokeSamples, helmetData.sampleCount);
//  jsonPayload += "}},\"waist\":";
  jsonPayload += "},\"waist\":";

  if (isWaistOnline && waistData != nullptr) {
    jsonPayload += "{\"rawAct\":\"" + String(waistData->rawAct) + "\",";
    jsonPayload += "\"hmmACT\":\"" + String(waistData->hmmAct) + "\",";
    jsonPayload += "\"stepCount\":" + String(waistData->stepCount) + ",";
    jsonPayload += "\"stepDistance\":" + String(waistData->stepDistance, 2) + ",";
    jsonPayload += "\"compass\":" + String(waistData->compass, 1) + ",";
    jsonPayload += "\"heading\":\"" + String(waistData->heading) + "\",";
    jsonPayload += "\"posX\":" + String(waistData->posX, 4) + ",";
    jsonPayload += "\"posY\":" + String(waistData->posY, 4); // + ","
//    jsonPayload += "\"inertialSample\":{";
//    jsonPayload += "\"ax\":" + buildAxisArrayJson(waistData->samples, SAMPLES_PER_SECOND, 0) + ",";
//    jsonPayload += "\"ay\":" + buildAxisArrayJson(waistData->samples, SAMPLES_PER_SECOND, 1) + ",";
//    jsonPayload += "\"az\":" + buildAxisArrayJson(waistData->samples, SAMPLES_PER_SECOND, 2) + ",";
//    jsonPayload += "\"gx\":" + buildAxisArrayJson(waistData->samples, SAMPLES_PER_SECOND, 3) + ",";
//    jsonPayload += "\"gy\":" + buildAxisArrayJson(waistData->samples, SAMPLES_PER_SECOND, 4) + ",";
//    jsonPayload += "\"gz\":" + buildAxisArrayJson(waistData->samples, SAMPLES_PER_SECOND, 5);
    jsonPayload += "}";
  } else {
    jsonPayload += "null";
  }
  jsonPayload += "}"; // Serial.println(jsonPayload);

  // Mã hóa URL cho chuỗi JSON và ghép nối đường dẫn
  String requestUrl = String(URL) + urlEncode(jsonPayload);

  // Khởi tạo đối tượng cục bộ trong từng phiên truyền
  WiFiClientSecure httpsClient;
  HTTPClient http;
  httpsClient.setInsecure(); // Bỏ qua kiểm tra chứng chỉ SSL để giảm độ trễ
  http.useHTTP10(true);      // Tránh lỗi kết nối Keep-Alive
  http.setTimeout(3000);     // Ngưỡng timeout tối đa 3 giây

  // Thực thi gửi yêu cầu HTTPS GET
  if (!http.begin(httpsClient, requestUrl)) {
    Serial.println(F("[HTTP] Khởi tạo kết nối thất bại."));
    return false;
  }
  int httpCode = http.GET();
  bool success = (httpCode == HTTP_CODE_OK);
  if (!success) { Serial.printf("[HTTP] Code: %d | Info: %s | URL Length: %u\n", httpCode, http.errorToString(httpCode).c_str(), requestUrl.length()); }
  http.end();
  httpsClient.stop(); // Giải phóng socket ngay sau khi hoàn tất
  return success;
}
