#include <NimBLEDevice.h>
#include "BLE_Transmitter.h"

// Khai báo các thực thể BLE
static BLEServer* pServer = nullptr;
static BLECharacteristic* pCharacteristic = nullptr;
static bool deviceConnected = false;

// Biến trạng thái quản lý phiên truyền
static InertialSample cachedSamples[SAMPLES_PER_SECOND];
static InertialData currentTxData;
static uint16_t samplesSentCount = 0;
static uint8_t currentPacketIndex = 0;
static bool isSendingActive = false;

// Lớp callback theo dõi trạng thái kết nối
class MyServerCallbacks: public NimBLEServerCallbacks {
  void onConnect(BLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println(F("[BLE Server] -> Client đã kết nối!"));
  };
  void onDisconnect(BLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.println(F("[BLE Server] -> Client đã ngắt kết nối. Đang phát lại..."));
    NimBLEDevice::getAdvertising()->start();
  }
};

// Cấu hình NimBLE Server
void setupBLETransmitter() {
  NimBLEDevice::init("ESP32_Waist_Transmitter");
  NimBLEDevice::setMTU(512);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pService->start();

  BLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  Serial.println(F("[BLE Server] -> Khởi tạo BLE thành công."));
}

bool isBLEConnected() { return deviceConnected; }

void startBLESendingSession(const String &currentRawAct, const String &currentHMMAct, uint32_t step, float distance, float compass, const String &heading, float posX, float posY, InertialSample* samplesArray) {
  if (!deviceConnected) return;

  // Sao chép mẫu và thông tin trạng thái vào bộ đệm đếm của phiên truyền
  memcpy(cachedSamples, samplesArray, sizeof(cachedSamples));
  memset(&currentTxData, 0, sizeof(InertialData));

  // Gán nhãn hành động
  currentRawAct.toCharArray(currentTxData.rawAct, sizeof(currentTxData.rawAct));
  currentHMMAct.toCharArray(currentTxData.hmmAct, sizeof(currentTxData.hmmAct));
  // Gán số bước và quãng đường di chuyển
  currentTxData.stepCount = step;
  currentTxData.stepDistance = distance;
  // Gán giá trị và hướng la bàn
  currentTxData.compass = compass;
  heading.toCharArray(currentTxData.heading, sizeof(currentTxData.heading));
  // Gán tọa độ
  currentTxData.posX = posX;
  currentTxData.posY = posY;

  // Khởi tạo các chỉ số truyền
  samplesSentCount = 0;
  currentPacketIndex = 0;
  isSendingActive = true;
}

void processBLESending() {
  if (!deviceConnected || !isSendingActive) { return; }

  // Xác định số lượng mẫu đóng gói cho phân đoạn hiện tại
  uint8_t samplesToPack = SAMPLES_PER_PACKET;
  if (SAMPLES_PER_SECOND - samplesSentCount < SAMPLES_PER_PACKET) {
    samplesToPack = SAMPLES_PER_SECOND - samplesSentCount;
  }

  currentTxData.packetIndex = currentPacketIndex;
  currentTxData.currentSamplesInPacket = samplesToPack;
  currentTxData.isLastPacket = (samplesSentCount + samplesToPack >= SAMPLES_PER_SECOND) ? 1 : 0;

  // Sao chép dữ liệu phân đoạn từ bộ đếm tạm
  memcpy(currentTxData.samples, &cachedSamples[samplesSentCount], samplesToPack * sizeof(InertialSample));
  // Phát gói tin qua phương thức Notify
  pCharacteristic->notify((uint8_t*)&currentTxData, sizeof(InertialData));

  samplesSentCount += samplesToPack;
  currentPacketIndex++;

  // Kết thúc phiên truyền khi tất cả các mẫu đã được phát đi
  if (samplesSentCount >= SAMPLES_PER_SECOND) {
    isSendingActive = false;
  }
}
