#include <NimBLEDevice.h>
#include "BLE_Receiver.h"

namespace {
  static BLEUUID serviceUUID(SERVICE_UUID);
  static BLEUUID charUUID(CHARACTERISTIC_UUID);

  static boolean doConnect = false;
  static boolean connected = false;
  static boolean doScan = false;

  static NimBLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
  static NimBLEAdvertisedDevice* targetDevice = nullptr;

  // Bộ đệm tĩnh tiếp nhận và lắp ráp gói tin phân mảnh
  static InertialSample assemblingSamples[SAMPLES_PER_SECOND];
  static WaistData latestReceivedData;
  static volatile bool dataReadyFlag = false;

  // Hàm callback xử lý gói tin nhị phân từ Server gửi qua Notify
  static void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // Kiểm tra kích thước gói dữ liệu nhận được có khớp cấu trúc không
    if (length != sizeof(InertialData)) {
      // Serial.printf("[BLE] Kích thước gói không khớp! Nhận được: %d bytes | Yêu cầu: %d bytes\n", length, sizeof(InertialData));
      return;
    }

    // Ép kiểu byte sang struct
    InertialData* rxData = (InertialData*)pData;
    // Tính toán vị trí offset để xếp dữ liệu
    uint8_t pIndex = rxData->packetIndex;
    uint8_t count = rxData->currentSamplesInPacket;
    uint16_t offset = pIndex * SAMPLES_PER_PACKET;

    if (offset + count <= SAMPLES_PER_SECOND) {
      memcpy(&assemblingSamples[offset], rxData->samples, count * sizeof(InertialSample));
    }
    // Serial.printf("[BLE] Nhận gói pIndex: %d | Mẫu: %d | isLast: %d\n", pIndex, count, rxData->isLastPacket);

    // Nếu nhận đủ gói cuối cùng của chu kỳ 1 giây, cập nhật toàn bộ trạng thái
    if (rxData->isLastPacket == 1) {
      memcpy(latestReceivedData.rawAct, rxData->rawAct, sizeof(latestReceivedData.rawAct));
      memcpy(latestReceivedData.hmmAct, rxData->hmmAct, sizeof(latestReceivedData.hmmAct));
      latestReceivedData.stepCount = rxData->stepCount;
      latestReceivedData.stepDistance = rxData->stepDistance;
      latestReceivedData.compass = rxData->compass;
      memcpy(latestReceivedData.heading, rxData->heading, sizeof(latestReceivedData.heading));
      latestReceivedData.posX = rxData->posX;
      latestReceivedData.posY = rxData->posY;
      latestReceivedData.lastReceivedTimeMs = millis();
      memcpy(latestReceivedData.samples, assemblingSamples, sizeof(assemblingSamples));
      dataReadyFlag = true;
    }
  }

  // Lớp callback theo dõi thiết bị quảng bá phù hợp
  class ClientScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
      if (advertisedDevice->isAdvertisingService(serviceUUID)) {
        NimBLEDevice::getScan()->stop();
        targetDevice = const_cast<NimBLEAdvertisedDevice*>(advertisedDevice);
        doConnect = true;
        doScan = false;
      }
    }
  };

  // Lớp callback quản lý vòng đời kết nối của Client
  class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
      connected = true;
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
      connected = false;
      doScan = true;
    }
  };

  static ClientScanCallbacks scanCallbacks;
  static ClientCallbacks clientCallbacks;

  // Thực hiện kết nối tới Server
  bool connectToServer() {
    if (targetDevice == nullptr) return false;

    NimBLEClient* pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCallbacks, false);
    // Cấu hình Connection Parameters tốc độ cao: 15ms - 30ms, latency = 0
    pClient->setConnectionParams(12, 24, 0, 200);
    if (!pClient->connect(targetDevice)){ return false; }

    NimBLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      pClient->disconnect();
      return false;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      pClient->disconnect();
      return false;
    }

    // Đăng ký nhận thông báo tự động (Notify) khi dữ liệu phía phát thay đổi
    if (pRemoteCharacteristic->canNotify()) {
      if (!pRemoteCharacteristic->subscribe(true, notifyCallback)) {
        pClient->disconnect();
        return false;
      }
    }

    connected = true;
    return true;
  }
} // namespace

void setupBLEReceiver() {
  NimBLEDevice::init("");
  NimBLEDevice::setMTU(512);
  NimBLEScan* pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(&scanCallbacks, false);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0, false);
}

void runBLEReceiver() {
  if (doConnect) {
    doConnect = false;
    if (!connectToServer()) {
      doScan = true;
    }
  }
  if (!connected && doScan) {
    doScan = false;
    NimBLEDevice::getScan()->start(0, false);
  }
}

bool isWaistConnected() { return connected; }
bool isWaistDataReady() { return dataReadyFlag; }
bool getwaistSnapshot(WaistData &outputData) {
  if (!dataReadyFlag) { return false; }
  memcpy(&outputData, &latestReceivedData, sizeof(WaistData));
  dataReadyFlag = false;
  return true;
}
