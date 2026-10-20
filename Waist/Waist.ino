#include <ADXL345.h>
#include <ITG3200.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <NimBLEDevice.h>
#include <TaskScheduler.h>
#include "HAR_Module.h"
#include "Step_Module.h"
#include "Compass_Module.h"
#include "BLE_Transmitter.h"
#include "Positioning_Module.h"

// Khởi tạo các đối tượng cảm biến
ADXL345 adxl;
ITG3200 gyro;
Adafruit_HMC5883_Unified hmc = Adafruit_HMC5883_Unified(12345);
Scheduler runner;

// Biến tạm lưu trữ mẫu ở chu kỳ 10ms đầu tiên
static InertialSample temp_sample;
// Mảng đệm thu thập dữ liệu quán tính để gửi BLE
static InertialSample main_ble_samples[SAMPLES_PER_SECOND];
static uint16_t main_sample_counter = 0, cycle_10ms_counter = 0;

void mainSystemTask();
Task taskLoop(10, TASK_FOREVER, &mainSystemTask); // 10 ms tương đương 100 Hz

void setup() {
  Serial.begin(115200);
  // Khởi tạo cảm biến
  adxl.powerOn();
  adxl.setRangeSetting(2);
  gyro.init();
  gyro.zeroCalibrate(200,10);

  // Khởi tạo các module chức năng
  setupBLETransmitter();
  setupHAR();
  setupStepCounter();
  setupCompass();
  setupPositioning();

  // Chạy trình lập lịch hệ thống
  runner.addTask(taskLoop);
  taskLoop.enable();
}

void loop() {
  runner.execute();
}

// Hàm điều phối dữ liệu trung tâm chạy mỗi 10ms
void mainSystemTask() {
  // Đọc dữ liệu gia tốc kế ADXL345
  double xyz[3];
  adxl.getAcceleration(xyz);
  float accX = (float)xyz[0];
  float accY = (float)xyz[1];
  float accZ = (float)xyz[2];
  // Đọc dữ liệu con quay hồi chuyển ITG3200
  float gyrX, gyrY, gyrZ;
  gyro.getAngularVelocity(&gyrX, &gyrY, &gyrZ);
  // Tính toán độ lớn vận tốc góc (deg/s) cho thuật toán định vị PDR
  float gyroMagnitudeDps = sqrtf(gyrX * gyrX + gyrY * gyrY + gyrZ * gyrZ);
  // Đọc dữ liệu từ trường kế HMC5883L
  sensors_event_t event; hmc.getEvent(&event);
  float magX = event.magnetic.x;
  float magY = event.magnetic.y;
  float magZ = event.magnetic.z;

  // Phân phối dữ liệu tới các module xử lý tín hiệu
  processHARData(accX, accY, accZ);
  processStepData(accX, accY, accZ);
  processCompassData(accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ);
  processPositioning(getRawAct(), getStepCount(), getStepDistance(), getCompassValue(), gyroMagnitudeDps, isCompassReady());

  // Gom dữ liệu downsampling 50 Hz bằng bộ lọc trung bình cộng 2 mẫu
  if (cycle_10ms_counter % 2 == 0) {
    // Lưu tạm mẫu đầu tiên của cặp mẫu
    temp_sample.accX = accX;
    temp_sample.accY = accY;
    temp_sample.accZ = accZ;
    temp_sample.gyrX = gyrX;
    temp_sample.gyrY = gyrY;
    temp_sample.gyrZ = gyrZ;
  } else {
    // Chu kỳ tiếp theo: tính trung bình cộng với mẫu trước đó và nạp vào mảng đệm
    if (main_sample_counter < SAMPLES_PER_SECOND) {
      main_ble_samples[main_sample_counter].accX = (temp_sample.accX + accX) * 0.5f;
      main_ble_samples[main_sample_counter].accY = (temp_sample.accY + accY) * 0.5f;
      main_ble_samples[main_sample_counter].accZ = (temp_sample.accZ + accZ) * 0.5f;
      main_ble_samples[main_sample_counter].gyrX = (temp_sample.gyrX + gyrX) * 0.5f;
      main_ble_samples[main_sample_counter].gyrY = (temp_sample.gyrY + gyrY) * 0.5f;
      main_ble_samples[main_sample_counter].gyrZ = (temp_sample.gyrZ + gyrZ) * 0.5f;
      main_sample_counter++;
    }
  }
  cycle_10ms_counter++;

  // Khi đủ chu kỳ, lấy kết quả tổng hợp của các module gửi qua BLE
  if (cycle_10ms_counter >= 100) {
    startBLESendingSession(
      getRawAct(),
      getHMMAct(),
      getStepCount(),
      getStepDistance(),
      getCompassValue(),
      getCompassHeading(),
      getPositionX(),
      getPositionY(),
      main_ble_samples
    );
    // Giải phóng mảng đệm
    main_sample_counter = 0, cycle_10ms_counter = 0;
    memset(main_ble_samples, 0, sizeof(main_ble_samples));
  }

  // Thực thi tiến trình phát xen kẽ 1 gói tin mỗi 20 ms
  if (cycle_10ms_counter % 2 == 0) {
    processBLESending();
  }
}
