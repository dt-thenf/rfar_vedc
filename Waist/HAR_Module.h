#ifndef HAR_MODULE_H
#define HAR_MODULE_H

#include <Arduino.h>

// Cấu hình thiết lập HMM Viterbi
#define N_STATES 9 // Số trạng thái ẩn
#define T_WINDOW 4 // Bộ đệm chuỗi gồm
#define EPSILON 1e-12f

// Khởi tạo module nhận dạng hoạt động
void setupHAR();
// Tiếp nhận dữ liệu gia tốc 3 trục (g), tính toán đặc trưng và thực hiện nhận dạng
void processHARData(float accX, float accY, float accZ);
// Trả về hành động
String getRawAct();
String getHMMAct();
// Trả về tình trạng (0: Normal, 1: Alert)
uint8_t getActAlert();
// Hiển thị báo cáo trực quan
void printHMMReport(String new_act);

#endif // HAR_MODULE_H

//    // Nhánh 1: Ưu tiên xử lý FA/FL
//    if (new_act == "FA") {
//      cur_act = "FA", pre_act = "FA";
//      fl_counter = 0;
//      is_transitioning = false;
//    }
//    // Xác nhận ngã bất động (FL) lần 1 khi chuỗi FA -> LY xảy ra
//    else if (pre_act == "FA" && new_act == "LY") {
//      cur_act = "FL", pre_act = "FL";
//      fl_counter = 1;
//      is_transitioning = false;
//    }
//    // Duy trì cảnh báo FL lần 2 nếu tiếp tục nằm bất động
//    else if (pre_act == "FL" && fl_counter == 1 && new_act == "LY") {
//      cur_act = "FL", pre_act = "FL";
//      fl_counter = 2;
//      is_transitioning = false;
//    }
//    // Sau 2 lần phát cảnh báo FL, tiếp nhận trực tiếp kết quả dự đoán không qua TS
//    else if (pre_act == "FL" && fl_counter >= 2) {
//      cur_act = new_act;
//      pre_act = cur_act;
//      fl_counter = 0;
//      is_transitioning = false;
//    }
//
//    // Nhánh 2: Các hoạt động ADLs
//    else {
//      fl_counter = 0;
//      // TH1: Chu kỳ trước là TS, tiếp nhận trực tiếp kết quả dự đoán
//      if (is_transitioning) {
//        cur_act = new_act;
//        pre_act = cur_act;
//        is_transitioning = false;
//      }
//      // TH2: Chu kỳ trước là trạng thái ổn định
//      else if (new_act == pre_act) {
//        cur_act = pre_act;
//      }
//      // TH3: Chuyển đổi thuộc nhóm được phép bỏ qua TS
//      else if (canTransitionDirectly(pre_act, new_act)) {
//        cur_act = new_act;
//        pre_act = cur_act;
//        is_transitioning = false;
//      }
//      // TH4: Chuyển dổi bắt buộc qua TS
//      else {
//        cur_act = "TS";
//        is_transitioning = true;
//      }
//    }



//    // Nhánh 1: Ưu tiên xử lý FA/FL
//    if (new_act == "FA") {
//      cur_act = "FA", pre_act = "FA";
//      fl_counter = 0;
//      is_transitioning = false;
//    }
//    // Xác nhận ngã bất động (FL) lần 1 khi chuỗi FA -> LY xảy ra
//    else if (pre_act == "FA" && new_act == "LY") {
//      cur_act = "FL", pre_act = "FL";
//      fl_counter = 1;
//      is_transitioning = false;
//    }
//    // Duy trì cảnh báo FL lần 2 nếu tiếp tục nằm bất động
//    else if (pre_act == "FL" && fl_counter == 1 && new_act == "LY") {
//      cur_act = "FL", pre_act = "FL";
//      fl_counter = 2;
//      is_transitioning = false;
//    }
//    // Sau 2 lần phát cảnh báo FL, tiếp nhận trực tiếp kết quả dự đoán không qua TS
//    else if (pre_act == "FL" && fl_counter >= 2) {
//      cur_act = new_act;
//      pre_act = cur_act;
//      fl_counter = 0;
//      is_transitioning = false;
//    }
//
//    // Nhánh 2: Các hoạt động ADLs
//    else {
//      fl_counter = 0;
//      // Chu kỳ trước là TS, tiếp nhận trực tiếp kết quả dự đoán
//      if (is_transitioning) {
//        cur_act = new_act;
//        pre_act = cur_act;
//        is_transitioning = false;
//      }
//      else {
//        // Chu kỳ trước là trạng thái ổn định
//        if (new_act == pre_act) {
//          cur_act = pre_act;
//        }
//        else {
//          // Gán TS khi phát hiện sự thay đổi hành động
//          cur_act = "TS";
//          is_transitioning = true;
//        }
//      }
//    }
