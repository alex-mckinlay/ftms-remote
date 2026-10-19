#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#include <M5Unified.h>
#include <atomic>
#include <vector>

// BLE UUID definitions
static BLEUUID FTMS_SERVICE_UUID((uint16_t)0x1826);
static BLEUUID TREADMILL_DATA_CHAR_UUID((uint16_t)0x2ACD);
static BLEUUID CONTROL_POINT_CHAR_UUID((uint16_t)0x2AD9);

// Merach Proprietary Vendor Service & Characteristic UUIDs
static BLEUUID MERACH_VENDOR_SERVICE_UUID("7905f431-b5ce-4e99-a40f-4b1e122d00d0");
static BLEUUID MERACH_VENDOR_CHAR_UUID((uint16_t)0xFFF2);

// State Machine States
enum RemoteState {
  STATE_DISCONNECTED,
  STATE_CONNECTING,
  STATE_CONNECTED_STOPPED,
  STATE_CONNECTED_RUNNING,
  STATE_ERROR
};

// Thread-safe POD structure for cached BLE scan results
struct ScannedDeviceInfo {
  char address[18];
  char name[32];
  int rssi;
};

// Audio request queue items (processed on main thread, never in BLE callbacks)
enum SoundReq {
  SOUND_NONE,
  SOUND_CONFIRM,
  SOUND_CANCEL,
  SOUND_CONNECT,
  SOUND_DISCONNECT,
  SOUND_ERROR
};

// Global Thread-Safe Variables & Atomics
std::atomic<RemoteState> currentState{STATE_DISCONNECTED};
std::atomic<bool> isRunning{false};
std::atomic<bool> isPaused{false};
std::atomic<bool> isTreadmillActive{false};
std::atomic<float> currentSpeed{0.0f};   // km/h (from FTMS telemetry)
std::atomic<float> currentIncline{0.0f}; // %
std::atomic<uint32_t> totalDistance{0};  // meters
std::atomic<uint16_t> elapsedTime{0};    // seconds
std::atomic<float> targetSpeed{0.0f};    // mph
std::atomic<float> targetIncline{0.0f};  // %
std::atomic<uint32_t> lastNotificationTime{0};
std::atomic<SoundReq> pendingSound{SOUND_NONE};
std::atomic<uint32_t> lastUserTargetChangeTime{0};
std::atomic<bool> isHoldingTouch{false};

// Closed-Loop Incline State Machine
std::atomic<bool> isPendingInclineChange{false};
std::atomic<float> lastObservedIncline{0.0f};
std::atomic<uint32_t> lastInclineProgressTime{0};

const uint32_t RAMP_SETTLE_WINDOW_MS = 5000;

// BLE Objects
BLEClient *pClient = nullptr;
BLERemoteCharacteristic *pTreadmillDataChar = nullptr;
BLERemoteCharacteristic *pControlPointChar = nullptr;
BLERemoteCharacteristic *pVendorWriteChar = nullptr;

// FreeRTOS Mutex for BLE cache and status message protection
SemaphoreHandle_t bleMutex = nullptr;
static std::vector<ScannedDeviceInfo> scannedDevicesCache;
static char targetDeviceAddress[18] = "";
static char statusMessage[64] = "Scanning for Walking Pad...";

// Timing & Control Variables (Main Thread Only)
uint32_t lastHeartbeatTime = 0;
bool doConnect = false;
std::atomic<bool> isScanning{false};
uint32_t lastScanTime = 0;
uint32_t lastUiRedraw = 0;
uint32_t lastTouchTime = 0;
int selectedDeviceIdx = -1;

int activeTouchBtnId = -1;
uint32_t activeTouchUntil = 0;

// Double buffer canvas
M5Canvas canvas(&M5.Display);

// ----------------------------------------------------
// Thread-Safe Helper Functions
// ----------------------------------------------------
void setStatusMessage(const char *msg) {
  if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    snprintf(statusMessage, sizeof(statusMessage), "%s", msg);
    xSemaphoreGive(bleMutex);
  } else {
    snprintf(statusMessage, sizeof(statusMessage), "%s", msg);
  }
}

void getStatusMessage(char *buf, size_t buflen) {
  if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    snprintf(buf, buflen, "%s", statusMessage);
    xSemaphoreGive(bleMutex);
  } else {
    snprintf(buf, buflen, "Initializing...");
  }
}

// ----------------------------------------------------
// Sound Feedback Helpers (Main Thread Only)
// ----------------------------------------------------
void playBeep(int freq, int duration) { M5.Speaker.tone(freq, duration); }

void processPendingSound() {
  SoundReq req = pendingSound.exchange(SOUND_NONE);
  if (req == SOUND_NONE) return;

  switch (req) {
  case SOUND_CONFIRM:
    playBeep(2000, 80);
    break;
  case SOUND_CANCEL:
    playBeep(1000, 150);
    break;
  case SOUND_CONNECT:
    playBeep(1500, 100);
    delay(100);
    playBeep(2000, 100);
    break;
  case SOUND_DISCONNECT:
    playBeep(1000, 150);
    delay(150);
    playBeep(800, 250);
    break;
  case SOUND_ERROR:
    playBeep(400, 350);
    break;
  default:
    break;
  }
}

// ----------------------------------------------------
// Native Merach Proprietary Compound Protocol Helpers
// Packet: [0x02, 0x53, 0x02, Speed_Raw, Incline_Raw, Checksum, 0x03]
// Checksum = 0x53 ^ 0x02 ^ Speed_Raw ^ Incline_Raw
// ----------------------------------------------------
void sendMerachCompoundCommand(float speedMph, float inclinePercent) {
  if (speedMph > 0.0f) {
    if (speedMph < 0.6f) speedMph = 0.6f;
    if (speedMph > 4.0f) speedMph = 4.0f;
  }
  if (inclinePercent < 0.0f) inclinePercent = 0.0f;
  if (inclinePercent > 12.0f) inclinePercent = 12.0f;

  uint8_t rawSpeed = (speedMph > 0.0f) ? (uint8_t)(speedMph * 10.0f + 0.5f) : 0;
  uint8_t rawIncl = (uint8_t)(inclinePercent + 0.5f);
  uint8_t cs = 0x53 ^ 0x02 ^ rawSpeed ^ rawIncl;

  uint8_t pkt[7] = {0x02, 0x53, 0x02, rawSpeed, rawIncl, cs, 0x03};

  BLERemoteCharacteristic *pChar = pVendorWriteChar;
  if (pChar != nullptr && currentState != STATE_DISCONNECTED) {
    try {
      pChar->writeValue(pkt, 7, true);
      Serial.printf("Sent Vendor Compound: Speed=%.1f mph (raw %u), Incl=%.1f%% (raw %u), CS=0x%02X\n",
                    speedMph, rawSpeed, inclinePercent, rawIncl, cs);
    } catch (...) {
      Serial.println("Vendor write failed with exception.");
    }
  }
}

void sendMerachHeartbeat() {
  BLERemoteCharacteristic *pChar = pVendorWriteChar;
  if (pChar != nullptr && currentState != STATE_DISCONNECTED) {
    uint8_t hb[4] = {0x02, 0x51, 0x51, 0x03};
    try {
      pChar->writeValue(hb, 4, true);
    } catch (...) {}
  }
}

// ----------------------------------------------------
// FTMS Control Point Helper Functions
// ----------------------------------------------------
bool sendControlCommand(uint8_t *data, size_t length) {
  BLERemoteCharacteristic *pChar = pControlPointChar;
  if (pChar == nullptr || currentState == STATE_DISCONNECTED) {
    Serial.println("Error: Control Point characteristic not available.");
    return false;
  }
  try {
    pChar->writeValue(data, length, true);
    return true;
  } catch (...) {
    Serial.println("Write to control point failed with exception.");
    setStatusMessage("Cmd Write Fail");
    return false;
  }
}

void requestControl() {
  uint8_t cmd = 0x00;
  sendControlCommand(&cmd, 1);
  setStatusMessage("Acquiring Control...");
  Serial.println("Sent: Request Control (0x00)");
}

void startTreadmill() {
  uint8_t cmd = 0x07;
  sendControlCommand(&cmd, 1);
  setStatusMessage("Starting Belt...");
  Serial.println("Sent: Start Belt (0x07)");

  isRunning = true;
  isPaused = false;
  currentState = STATE_CONNECTED_RUNNING;

  if (targetSpeed < 0.6f) {
    targetSpeed = 0.6f;
  }

  if (pVendorWriteChar != nullptr) {
    uint8_t vendorStart[13] = {0x02, 0x53, 0x01, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0x03};
    try {
      pVendorWriteChar->writeValue(vendorStart, 13, true);
    } catch (...) {}
    sendMerachCompoundCommand(targetSpeed, targetIncline);
  }
}

void resumeTreadmill() {
  uint8_t cmd = 0x07;
  sendControlCommand(&cmd, 1);
  setStatusMessage("Resuming Belt...");
  Serial.println("Sent: Resume Belt (0x07)");

  isPaused = false;
  isRunning = true;
  currentState = STATE_CONNECTED_RUNNING;

  if (pVendorWriteChar != nullptr) {
    sendMerachCompoundCommand(targetSpeed, targetIncline);
  }
}

void stopTreadmill() {
  uint8_t cmd[2] = {0x08, 0x01};
  sendControlCommand(cmd, 2);
  setStatusMessage("Stopping Belt...");
  Serial.println("Sent: Stop Belt (0x08, 0x01)");

  isRunning = false;
  isPaused = false;
  isTreadmillActive = false;
  targetSpeed = 0.6f;
  currentState = STATE_CONNECTED_STOPPED;

  if (pVendorWriteChar != nullptr) {
    sendMerachCompoundCommand(0.0f, targetIncline);
  }
}

void pauseTreadmill() {
  uint8_t cmd[2] = {0x08, 0x02};
  sendControlCommand(cmd, 2);
  setStatusMessage("Pausing Belt...");
  Serial.println("Sent: Pause Belt (0x08, 0x02)");

  isPaused = true;
  isRunning = true;

  if (pVendorWriteChar != nullptr) {
    sendMerachCompoundCommand(0.0f, targetIncline);
  }
}

void setSpeed(float speedMph) {
  if (speedMph < 0.6f) speedMph = 0.6f;
  if (speedMph > 4.0f) speedMph = 4.0f;
  targetSpeed = speedMph;
  lastUserTargetChangeTime = millis();

  if (pVendorWriteChar != nullptr) {
    sendMerachCompoundCommand(targetSpeed, targetIncline);
  } else {
    uint16_t rawSpeed = (uint16_t)(speedMph * 100.0f + 0.5f);
    uint8_t cmd[3] = {0x02, (uint8_t)(rawSpeed & 0xFF),
                      (uint8_t)((rawSpeed >> 8) & 0xFF)};
    sendControlCommand(cmd, 3);
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "Set Speed: %.1f mph", speedMph);
  setStatusMessage(buf);
  Serial.printf("Sent: Set Speed to %.1f mph\n", speedMph);
}

void setIncline(float inclinePercent) {
  if (inclinePercent < 0.0f) inclinePercent = 0.0f;
  if (inclinePercent > 12.0f) inclinePercent = 12.0f;
  targetIncline = inclinePercent;
  isPendingInclineChange = true;
  lastInclineProgressTime = millis();
  lastObservedIncline = (float)currentIncline;
  lastUserTargetChangeTime = millis();

  if (pVendorWriteChar != nullptr) {
    float spd = isRunning ? (float)targetSpeed : 0.0f;
    sendMerachCompoundCommand(spd, targetIncline);
  } else {
    int16_t rawIncline = (int16_t)(inclinePercent * 10.0f);
    uint8_t cmd[3] = {0x03, (uint8_t)(rawIncline & 0xFF),
                      (uint8_t)((rawIncline >> 8) & 0xFF)};
    sendControlCommand(cmd, 3);

    if (isTreadmillActive && targetSpeed > 0.0f) {
      uint16_t rawSpeed = (uint16_t)(targetSpeed * 100.0f + 0.5f);
      uint8_t spdCmd[3] = {0x02, (uint8_t)(rawSpeed & 0xFF),
                           (uint8_t)((rawSpeed >> 8) & 0xFF)};
      sendControlCommand(spdCmd, 3);
    }
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "Set Incline: %.1f%%", inclinePercent);
  setStatusMessage(buf);
  Serial.printf("Sent: Set Incline to %.1f%%\n", inclinePercent);
}

// ----------------------------------------------------
// BLE Callback Handlers (Thread Safe, Zero Delays)
// ----------------------------------------------------
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) override {
    Serial.println("BLE Client Connected Event.");
  }
  void onDisconnect(BLEClient *pclient) override {
    Serial.println("BLE Client Disconnected Event.");
    currentState = STATE_DISCONNECTED;
    pTreadmillDataChar = nullptr;
    pControlPointChar = nullptr;
    pVendorWriteChar = nullptr;
    isRunning = false;
    isPaused = false;
    isTreadmillActive = false;
    currentSpeed = 0.0f;
    currentIncline = 0.0f;
    targetSpeed = 0.6f;
    setStatusMessage("Disconnected from Pad");
    pendingSound = SOUND_DISCONNECT;
  }
};
static MyClientCallback clientCallback;

void treadmillDataCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData, size_t length, bool isNotify) {
  lastNotificationTime = millis();
  if (length < 2) return;

  uint16_t flags = pData[0] | (pData[1] << 8);
  size_t offset = 2;

  // Bit 0: Speed (2 bytes, 0.01 km/h)
  if ((flags & (1 << 0)) == 0) {
    if (offset + 2 <= length) {
      uint16_t rawSpeed = pData[offset] | (pData[offset + 1] << 8);
      currentSpeed = rawSpeed / 100.0f;
      offset += 2;
    }
  }

  // Bit 1: Average Speed Present
  if (flags & (1 << 1)) offset += 2;

  // Bit 2: Total Distance Present (3 bytes, meters)
  if (flags & (1 << 2)) {
    if (offset + 3 <= length) {
      totalDistance = pData[offset] | (pData[offset + 1] << 8) | (pData[offset + 2] << 16);
      offset += 3;
    }
  }

  // Bit 3: Inclination Present (2 bytes, 0.1%)
  if (flags & (1 << 3)) {
    if (offset + 2 <= length) {
      int16_t rawIncline = pData[offset] | (pData[offset + 1] << 8);
      currentIncline = rawIncline / 10.0f;
    }
    if (offset + 4 <= length) {
      offset += 4;
    } else {
      offset += 2;
    }
  }

  if (flags & (1 << 4)) offset += 4;
  if (flags & (1 << 5)) offset += 1;
  if (flags & (1 << 6)) offset += 1;
  if (flags & (1 << 7)) offset += 5;
  if (flags & (1 << 8)) offset += 1;
  if (flags & (1 << 9)) offset += 1;

  // Bit 10: Elapsed Time Present (2 bytes, seconds)
  if (flags & (1 << 10)) {
    if (offset + 2 <= length) {
      elapsedTime = pData[offset] | (pData[offset + 1] << 8);
      offset += 2;
    }
  }

  // Dynamically adjust remote running states
  float spd = currentSpeed;
  if (spd > 0.05f) {
    isTreadmillActive = true;
    isRunning = true;
    isPaused = false;
    if (currentState == STATE_CONNECTED_STOPPED || currentState == STATE_CONNECTING) {
      currentState = STATE_CONNECTED_RUNNING;
      setStatusMessage("Running");
    }
    float spdMph = spd * 0.621371f;
    if (spdMph < 0.6f) spdMph = 0.6f;
    if (spdMph > 4.0f) spdMph = 4.0f;

    bool userIsOverridingSpeed = isHoldingTouch || (millis() - lastUserTargetChangeTime < RAMP_SETTLE_WINDOW_MS);
    if (!userIsOverridingSpeed) {
      targetSpeed = spdMph;
    }

    // Closed-Loop Incline State Machine Evaluation
    float incl = (float)currentIncline;
    uint32_t now = millis();

    if (isPendingInclineChange) {
      float target = (float)targetIncline;
      float prevObserved = (float)lastObservedIncline;

      if (std::abs(incl - target) <= 0.15f) {
        isPendingInclineChange = false;
        targetIncline = incl;
        lastObservedIncline = incl;
      } else {
        bool movingTowards = (target > prevObserved && incl > prevObserved) ||
                             (target < prevObserved && incl < prevObserved);
        bool movingAway = (target > prevObserved && incl < prevObserved) ||
                          (target < prevObserved && incl > prevObserved);

        if (movingTowards) {
          lastInclineProgressTime = now;
          lastObservedIncline = incl;
        } else if (movingAway) {
          isPendingInclineChange = false;
          targetIncline = incl;
          lastObservedIncline = incl;
        } else {
          if (now - lastInclineProgressTime > 3000) {
            isPendingInclineChange = false;
            targetIncline = incl;
            lastObservedIncline = incl;
          }
        }
      }
    } else if (!isHoldingTouch) {
      targetIncline = incl;
      lastObservedIncline = incl;
    }
  } else {
    isTreadmillActive = false;
    if (!isPaused) {
      isRunning = false;
      targetSpeed = 0.6f;
      if (currentState != STATE_DISCONNECTED) {
        currentState = STATE_CONNECTED_STOPPED;
        setStatusMessage("Stopped");
      }
    }
  }
}

void controlPointCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                          uint8_t *pData, size_t length, bool isNotify) {
  if (length >= 3 && pData[0] == 0x80) {
    uint8_t reqOp = pData[1];
    uint8_t resCode = pData[2];

    if (resCode == 0x01) { // Success
      if (reqOp == 0x00) {
        setStatusMessage("Ready (Control Acquired)");
        if (currentState != STATE_CONNECTED_RUNNING && !isTreadmillActive) {
          currentState = STATE_CONNECTED_STOPPED;
        }
        Serial.println("Control Point: Control acquired successfully.");
      } else if (reqOp == 0x07) {
        setStatusMessage("Treadmill Started");
        isRunning = true;
        isTreadmillActive = true;
        currentState = STATE_CONNECTED_RUNNING;
        Serial.println("Control Point: Start command accepted.");
      } else if (reqOp == 0x08) {
        if (!isPaused) {
          setStatusMessage("Treadmill Stopped");
          isRunning = false;
          isTreadmillActive = false;
          targetSpeed = 0.6f;
          currentState = STATE_CONNECTED_STOPPED;
          currentSpeed = 0.0f;
          Serial.println("Control Point: Stop command accepted.");
        } else {
          setStatusMessage("Treadmill Paused");
          Serial.println("Control Point: Pause command accepted.");
        }
      } else if (reqOp == 0x02) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Speed set to %.1f mph", (float)targetSpeed);
        setStatusMessage(buf);
        Serial.println("Control Point: Speed set accepted.");
      } else if (reqOp == 0x03) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Incline set to %.1f%%", (float)targetIncline);
        setStatusMessage(buf);
        Serial.println("Control Point: Incline set accepted.");
      }
    } else { // Error
      Serial.printf("Control Point error: Op 0x%02X failed with code 0x%02X\n", reqOp, resCode);
      char buf[32];
      snprintf(buf, sizeof(buf), "Error: Cmd Fail (%u)", resCode);
      setStatusMessage(buf);
      pendingSound = SOUND_ERROR;
    }
  }
}

// ----------------------------------------------------
// BLE Scanner Setup & Device Collection (Thread Safe POD Cache)
// ----------------------------------------------------
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    bool isFtms = advertisedDevice.haveServiceUUID() &&
                  advertisedDevice.isAdvertisingService(FTMS_SERVICE_UUID);
    std::string stdName = advertisedDevice.getName();
    bool isMerach = (stdName.find("Merach") != std::string::npos) ||
                    (stdName.find("T-25B") != std::string::npos) ||
                    (stdName.find("Treadmill") != std::string::npos) ||
                    (stdName.find("Walking") != std::string::npos);

    if (isFtms || isMerach) {
      ScannedDeviceInfo info;
      snprintf(info.address, sizeof(info.address), "%s", advertisedDevice.getAddress().toString().c_str());
      snprintf(info.name, sizeof(info.name), "%s", stdName.c_str());
      info.rssi = advertisedDevice.getRSSI();

      if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool exists = false;
        for (const auto &dev : scannedDevicesCache) {
          if (strcmp(dev.address, info.address) == 0) {
            exists = true;
            break;
          }
        }
        if (!exists && scannedDevicesCache.size() < 8) {
          scannedDevicesCache.push_back(info);
          Serial.printf("Found Device: %s (%s), RSSI: %d\n", info.name, info.address, info.rssi);
        }
        xSemaphoreGive(bleMutex);
      }
    }
  }
};

void startScan() {
  if (isScanning) return;

  if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    scannedDevicesCache.clear();
    xSemaphoreGive(bleMutex);
  }

  selectedDeviceIdx = -1;
  isScanning = true;
  setStatusMessage("Scanning for Walking Pad...");
  Serial.println("Starting BLE scan...");

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->start(3, false); // Scan for 3 seconds

  isScanning = false;
  lastScanTime = millis();

  size_t count = 0;
  if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    count = scannedDevicesCache.size();
    xSemaphoreGive(bleMutex);
  }

  Serial.printf("BLE Scan done. Devices found: %u\n", (unsigned int)count);
  setStatusMessage("Scan Done. Tap device to connect.");
}

// ----------------------------------------------------
// BLE Connect Engine (Reuses Persistent BLEClient)
// ----------------------------------------------------
bool connectToDeviceAddress(const char *addressStr) {
  currentState = STATE_CONNECTING;
  setStatusMessage("Connecting to Pad...");
  Serial.printf("Connecting to address: %s\n", addressStr);

  if (pClient == nullptr) {
    Serial.println("Creating persistent BLE client...");
    pClient = BLEDevice::createClient();
    if (pClient == nullptr) {
      Serial.println("Failed to create BLE client!");
      currentState = STATE_DISCONNECTED;
      setStatusMessage("Client Alloc Fail");
      return false;
    }
    pClient->setClientCallbacks(&clientCallback);
  }

  BLEAddress address(addressStr);
  if (!pClient->connect(address)) {
    currentState = STATE_DISCONNECTED;
    setStatusMessage("Connect Failed");
    pendingSound = SOUND_ERROR;
    Serial.println("BLE Connect failed.");
    return false;
  }

  Serial.println("Connected. Scanning all GATT services for Merach Vendor Characteristic...");
  pVendorWriteChar = nullptr;
  delay(150);
  try {
    std::map<std::string, BLERemoteService*> *pServices = pClient->getServices();
    if (pServices != nullptr) {
      for (auto const &pair : *pServices) {
        BLERemoteService *pSvc = pair.second;
        if (pSvc != nullptr) {
          std::map<std::string, BLERemoteCharacteristic*> *pChars = pSvc->getCharacteristics();
          if (pChars != nullptr) {
            for (auto const &charPair : *pChars) {
              BLERemoteCharacteristic *pChar = charPair.second;
              if (pChar != nullptr) {
                std::string uuidStr = pChar->getUUID().toString();
                uint16_t handle = pChar->getHandle();
                if (uuidStr.find("fff2") != std::string::npos || 
                    uuidStr.find("FFF2") != std::string::npos || 
                    handle == 0x003A) {
                  pVendorWriteChar = pChar;
                  Serial.printf("SUCCESS: Captured Merach Vendor Write Char (0xFFF2) - Handle 0x%04X, UUID: %s!\n",
                                handle, uuidStr.c_str());
                  break;
                }
              }
            }
          }
        }
        if (pVendorWriteChar != nullptr) break;
      }
    }
  } catch (...) {
    Serial.println("Exception during vendor characteristic discovery scan.");
  }
  if (pVendorWriteChar == nullptr) {
    Serial.println("WARNING: Merach Vendor Write Char (0xFFF2) NOT captured! Falling back to FTMS Control Point.");
  }

  Serial.println("Connected. Discovering FTMS Service...");
  BLERemoteService *pRemoteService = pClient->getService(FTMS_SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.println("Error: Could not find FTMS Service.");
    pClient->disconnect();
    currentState = STATE_DISCONNECTED;
    setStatusMessage("No FTMS Service Found");
    return false;
  }

  pTreadmillDataChar = pRemoteService->getCharacteristic(TREADMILL_DATA_CHAR_UUID);
  pControlPointChar = pRemoteService->getCharacteristic(CONTROL_POINT_CHAR_UUID);

  if (pTreadmillDataChar == nullptr || pControlPointChar == nullptr) {
    Serial.println("Error: Required characteristics are missing.");
    pClient->disconnect();
    currentState = STATE_DISCONNECTED;
    setStatusMessage("Missing FTMS Characteristics");
    return false;
  }

  if (pTreadmillDataChar->canNotify()) {
    pTreadmillDataChar->registerForNotify(treadmillDataCallback);
    BLERemoteDescriptor *pDesc = pTreadmillDataChar->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (pDesc != nullptr) {
      uint8_t val[] = {0x01, 0x00};
      pDesc->writeValue(val, 2, true);
    }
  }

  if (pControlPointChar->canIndicate()) {
    pControlPointChar->registerForNotify(controlPointCallback);
    BLERemoteDescriptor *pDesc = pControlPointChar->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (pDesc != nullptr) {
      uint8_t val[] = {0x02, 0x00};
      pDesc->writeValue(val, 2, true);
    }
  }

  lastNotificationTime = millis();
  setStatusMessage("Initializing Connection...");
  pendingSound = SOUND_CONNECT;

  delay(300);
  requestControl();
  return true;
}

// BGR Hardware Color Helper for M5Stack CoreS3 LCD Panel
inline uint16_t makeColor(uint8_t r, uint8_t g, uint8_t b) {
  return canvas.color565(r, g, b);
}

// ----------------------------------------------------
// Apple Design System Palette
// ----------------------------------------------------
const uint16_t cAppleBlack       = makeColor(0, 0, 0);
const uint16_t cAppleDarkCard    = makeColor(28, 28, 30);
const uint16_t cAppleDarkCardAlt = makeColor(38, 38, 42);
const uint16_t cAppleBorder      = makeColor(48, 48, 54);
const uint16_t cAppleDivider     = makeColor(38, 38, 40);

const uint16_t cAppleGreen       = makeColor(48, 209, 88);
const uint16_t cAppleGreenDark   = makeColor(12, 44, 20);

const uint16_t cAppleOrange      = makeColor(255, 149, 0);
const uint16_t cAppleOrangeDark  = makeColor(48, 26, 0);

const uint16_t cAppleCyan        = makeColor(100, 210, 255);
const uint16_t cAppleCyanDark    = makeColor(14, 40, 56);

const uint16_t cAppleYellow      = makeColor(255, 204, 0);
const uint16_t cAppleRed         = makeColor(255, 0, 0);

const uint16_t cAppleWhite       = makeColor(255, 255, 255);
const uint16_t cAppleMuted       = makeColor(142, 142, 147);

// Vector Graphic Helpers
void drawVectorPlayIcon(int cx, int cy, uint16_t color) {
  canvas.fillTriangle(cx - 6, cy - 8, cx - 6, cy + 8, cx + 8, cy, color);
}

void drawVectorPauseIcon(int cx, int cy, uint16_t color) {
  canvas.fillRect(cx - 6, cy - 7, 4, 14, color);
  canvas.fillRect(cx + 2, cy - 7, 4, 14, color);
}

void drawVectorStopIcon(int cx, int cy, uint16_t color) {
  canvas.fillRoundRect(cx - 7, cy - 7, 14, 14, 2, color);
}

void drawRoundStepperButton(int cx, int cy, int r, uint16_t bg, uint16_t border, uint16_t textCol, bool isPlus, bool isPressed) {
  uint16_t finalBg = isPressed ? cAppleWhite : bg;
  uint16_t finalBorder = isPressed ? cAppleWhite : border;
  uint16_t finalIcon = isPressed ? cAppleBlack : textCol;

  canvas.fillCircle(cx, cy, r, finalBg);
  canvas.drawCircle(cx, cy, r, finalBorder);

  int barW = (int)(r * 0.55f);
  int barH = 3;
  canvas.fillRect(cx - barW / 2, cy - barH / 2, barW, barH, finalIcon);

  if (isPlus) {
    canvas.fillRect(cx - barH / 2, cy - barW / 2, barH, barW, finalIcon);
  }
}

// ----------------------------------------------------
// Double-Buffered UI Drawing (Zero Dynamic Heap Allocations)
// ----------------------------------------------------
void drawUI() {
  bool isHighlightActive = (activeTouchBtnId > 0 && millis() < activeTouchUntil);
  RemoteState state = currentState;

  canvas.fillScreen(cAppleBlack);
  canvas.setTextSize(1.0);

  // 1. Status Header
  canvas.fillRect(0, 0, 320, 26, cAppleBlack);
  canvas.drawFastHLine(0, 25, 320, cAppleDivider);

  uint16_t dotColor = (state == STATE_DISCONNECTED) ? cAppleMuted :
                      (state == STATE_CONNECTING)   ? cAppleOrange : cAppleGreen;
  canvas.fillCircle(12, 13, 4, dotColor);
  canvas.drawCircle(12, 13, 6, dotColor);

  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(cAppleWhite);
  canvas.drawString("T-25B", 22, 5);

  if (state == STATE_DISCONNECTED) {
    bool isRescanPressed = (isHighlightActive && activeTouchBtnId == 10);
    uint16_t btnBg = isRescanPressed ? cAppleWhite : cAppleDarkCard;
    uint16_t btnBorder = isRescanPressed ? cAppleWhite : cAppleBorder;
    uint16_t btnTxt = isRescanPressed ? cAppleBlack : cAppleCyan;

    canvas.fillRoundRect(225, 3, 88, 20, 10, btnBg);
    canvas.drawRoundRect(225, 3, 88, 20, 10, btnBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(btnTxt);
    canvas.drawCenterString("RE-SCAN", 269, 9);

  } else if (state == STATE_CONNECTING) {
    canvas.fillRoundRect(205, 3, 108, 20, 10, cAppleOrangeDark);
    canvas.drawRoundRect(205, 3, 108, 20, 10, cAppleOrange);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cAppleOrange);
    canvas.drawCenterString("CONNECTING...", 259, 9);

  } else {
    // Connected Dashboard
    uint16_t et = elapsedTime;
    uint32_t td = totalDistance;
    int mins = et / 60;
    int secs = et % 60;
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", mins, secs);

    float distMiles = (td / 1000.0f) * 0.621371f;
    char distBuf[16];
    snprintf(distBuf, sizeof(distBuf), "%.2fmi", distMiles);

    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(cAppleYellow);
    canvas.drawString(timeBuf, 72, 4);

    canvas.setTextColor(cAppleCyan);
    canvas.drawString(distBuf, 140, 4);

    bool running = isRunning;
    bool paused = isPaused;
    uint16_t pillBg = (running && !paused) ? cAppleGreenDark : (paused ? cAppleOrangeDark : cAppleDarkCard);
    uint16_t pillBorder = (running && !paused) ? cAppleGreen : (paused ? cAppleOrange : cAppleBorder);
    uint16_t pillTxt = (running && !paused) ? cAppleGreen : (paused ? cAppleOrange : cAppleMuted);
    const char *pillText = (running && !paused) ? "RUNNING" : (paused ? "PAUSED" : "STOPPED");

    canvas.fillRoundRect(220, 3, 94, 20, 10, pillBg);
    canvas.drawRoundRect(220, 3, 94, 20, 10, pillBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(pillTxt);
    canvas.drawCenterString(pillText, 267, 9);
  }

  // 2. DISCONNECTED / SCANNER VIEW
  if (state == STATE_DISCONNECTED) {
    std::vector<ScannedDeviceInfo> devicesCopy;
    if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      devicesCopy = scannedDevicesCache;
      xSemaphoreGive(bleMutex);
    }

    if (devicesCopy.empty()) {
      int cx = 160, cy = 85;
      canvas.drawCircle(cx, cy, 22, cAppleDarkCardAlt);
      canvas.drawCircle(cx, cy, 30, cAppleBorder);
      canvas.setFont(&fonts::Font4);
      canvas.setTextColor(cAppleCyan);
      canvas.drawCenterString("BLE", cx, cy - 12);

      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(cAppleWhite);
      canvas.drawCenterString("Searching for Walking Pad...", 160, 132);

      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(cAppleMuted);
      canvas.drawCenterString("Ensure device is powered on and near remote.", 160, 156);
    } else {
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(cAppleMuted);
      canvas.drawString("SELECT DEVICE TO CONNECT:", 12, 32);

      for (size_t i = 0; i < devicesCopy.size() && i < 4; i++) {
        int y = 46 + i * 42;
        int btnId = 11 + i;
        bool isItemPressed = (isHighlightActive && activeTouchBtnId == btnId);

        uint16_t bg = isItemPressed ? cAppleCyanDark : cAppleDarkCard;
        uint16_t border = isItemPressed ? cAppleCyan : cAppleBorder;

        canvas.fillRoundRect(8, y, 304, 38, 10, bg);
        canvas.drawRoundRect(8, y, 304, 38, 10, border);

        canvas.fillCircle(24, y + 19, 4, cAppleCyan);

        canvas.setFont(&fonts::Font2);
        canvas.setTextColor(cAppleWhite);
        const char *displayName = (strlen(devicesCopy[i].name) > 0) ? devicesCopy[i].name : "Merach Walking Pad";
        canvas.drawString(displayName, 36, y + 10);

        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(cAppleMuted);
        char rssiBuf[16];
        snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", devicesCopy[i].rssi);
        canvas.drawString(rssiBuf, 245, y + 14);
      }
    }

    char currentStatus[64];
    getStatusMessage(currentStatus, sizeof(currentStatus));
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cAppleMuted);
    canvas.drawCenterString(currentStatus, 160, 224);

  } else if (state == STATE_CONNECTING) {
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(cAppleCyan);
    canvas.drawCenterString("CONNECTING", 160, 90);

    char currentStatus[64];
    getStatusMessage(currentStatus, sizeof(currentStatus));
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(cAppleMuted);
    canvas.drawCenterString(currentStatus, 160, 130);

  } else if (state == STATE_CONNECTED_STOPPED) {
    // Ready State (Stopped Dashboard with active Incline controls and Start button)
    // SPEED CARD
    canvas.fillRoundRect(6, 26, 308, 80, 14, cAppleDarkCard);
    canvas.drawRoundRect(6, 26, 308, 80, 14, cAppleBorder);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cAppleMuted);
    canvas.drawString("SPEED", 16, 30);

    bool p2 = (isHighlightActive && activeTouchBtnId == 2);
    drawRoundStepperButton(40, 66, 22, cAppleBlack, cAppleBorder, cAppleMuted, false, p2);

    float displaySpeed = (targetSpeed > 0.0f) ? (float)targetSpeed : 0.6f;
    char spdBuf[16];
    snprintf(spdBuf, sizeof(spdBuf), "%.1f MPH", displaySpeed);

    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1.35);
    canvas.setTextColor(cAppleWhite);
    canvas.drawCenterString(spdBuf, 160, 48);
    canvas.setTextSize(1.0);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cAppleMuted);
    canvas.drawCenterString("Ready to Start", 160, 84);

    bool p1 = (isHighlightActive && activeTouchBtnId == 1);
    drawRoundStepperButton(280, 66, 22, cAppleBlack, cAppleBorder, cAppleMuted, true, p1);

    // INCLINE CARD
    canvas.fillRoundRect(6, 112, 308, 80, 14, cAppleOrangeDark);
    canvas.drawRoundRect(6, 112, 308, 80, 14, cAppleOrange);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cAppleOrange);
    canvas.drawString("INCLINE", 16, 116);

    bool p4 = (isHighlightActive && activeTouchBtnId == 4);
    drawRoundStepperButton(40, 152, 22, cAppleBlack, cAppleOrange, cAppleOrange, false, p4);

    char inclBuf[16];
    snprintf(inclBuf, sizeof(inclBuf), "%.1f %%", (float)targetIncline);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1.35);
    canvas.setTextColor(cAppleWhite);
    canvas.drawCenterString(inclBuf, 160, 134);
    canvas.setTextSize(1.0);

    bool p3 = (isHighlightActive && activeTouchBtnId == 3);
    drawRoundStepperButton(280, 152, 22, cAppleBlack, cAppleOrange, cAppleOrange, true, p3);

    canvas.fillRoundRect(70, 182, 180, 4, 2, makeColor(35, 20, 0));
    int inclTrack = (int)(((float)targetIncline / 12.0f) * 180.0f);
    if (inclTrack > 0) canvas.fillRoundRect(70, 182, inclTrack, 4, 2, cAppleOrange);

    // START BUTTON CAPSULE
    bool p5 = (isHighlightActive && activeTouchBtnId == 5);
    uint16_t startBg = p5 ? cAppleWhite : cAppleGreen;
    canvas.fillRoundRect(6, 196, 308, 42, 14, startBg);
    canvas.drawRoundRect(6, 196, 308, 42, 14, startBg);
    drawVectorPlayIcon(160, 217, cAppleBlack);

  } else {
    // Workout Active View
    // SPEED CARD
    canvas.fillRoundRect(6, 26, 308, 80, 14, cAppleGreenDark);
    canvas.drawRoundRect(6, 26, 308, 80, 14, cAppleGreen);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cAppleGreen);
    canvas.drawString("SPEED", 16, 30);

    bool p2 = (isHighlightActive && activeTouchBtnId == 2);
    drawRoundStepperButton(40, 66, 22, cAppleBlack, cAppleGreen, cAppleGreen, false, p2);

    float displaySpeed = currentSpeed * 0.621371f;
    char spdBuf[16];
    snprintf(spdBuf, sizeof(spdBuf), "%.1f MPH", displaySpeed);

    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1.35);
    canvas.setTextColor(cAppleWhite);
    canvas.drawCenterString(spdBuf, 160, 48);
    canvas.setTextSize(1.0);

    char targetSpdBuf[32];
    snprintf(targetSpdBuf, sizeof(targetSpdBuf), "Target: %.1f mph", (float)targetSpeed);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cAppleGreen);
    canvas.drawCenterString(targetSpdBuf, 160, 84);

    bool p1 = (isHighlightActive && activeTouchBtnId == 1);
    drawRoundStepperButton(280, 66, 22, cAppleBlack, cAppleGreen, cAppleGreen, true, p1);

    canvas.fillRoundRect(70, 96, 180, 4, 2, makeColor(15, 30, 18));
    int spdTrack = (int)(((displaySpeed - 0.6f) / 3.4f) * 180.0f);
    if (spdTrack < 4) spdTrack = 4;
    if (spdTrack > 180) spdTrack = 180;
    canvas.fillRoundRect(70, 96, spdTrack, 4, 2, cAppleGreen);

    // INCLINE CARD
    canvas.fillRoundRect(6, 112, 308, 80, 14, cAppleOrangeDark);
    canvas.drawRoundRect(6, 112, 308, 80, 14, cAppleOrange);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cAppleOrange);
    canvas.drawString("INCLINE", 16, 116);

    bool p4 = (isHighlightActive && activeTouchBtnId == 4);
    drawRoundStepperButton(40, 152, 22, cAppleBlack, cAppleOrange, cAppleOrange, false, p4);

    char inclBuf[16];
    snprintf(inclBuf, sizeof(inclBuf), "%.1f %%", (float)currentIncline);
    canvas.setFont(&fonts::Font4);
    canvas.setTextSize(1.35);
    canvas.setTextColor(cAppleWhite);
    canvas.drawCenterString(inclBuf, 160, 134);
    canvas.setTextSize(1.0);

    bool p3 = (isHighlightActive && activeTouchBtnId == 3);
    drawRoundStepperButton(280, 152, 22, cAppleBlack, cAppleOrange, cAppleOrange, true, p3);

    canvas.fillRoundRect(70, 182, 180, 4, 2, makeColor(35, 20, 0));
    int inclTrack = (int)(((float)currentIncline / 12.0f) * 180.0f);
    if (inclTrack > 0) canvas.fillRoundRect(70, 182, inclTrack, 4, 2, cAppleOrange);

    // ACTION CAPSULES
    bool p5 = (isHighlightActive && activeTouchBtnId == 5);
    if (isPaused) {
      uint16_t resumeBg = p5 ? cAppleWhite : cAppleGreen;
      canvas.fillRoundRect(6, 196, 148, 42, 14, resumeBg);
      canvas.drawRoundRect(6, 196, 148, 42, 14, resumeBg);
      drawVectorPlayIcon(80, 217, cAppleBlack);
    } else {
      uint16_t pauseBg = p5 ? cAppleWhite : cAppleOrange;
      canvas.fillRoundRect(6, 196, 148, 42, 14, pauseBg);
      canvas.drawRoundRect(6, 196, 148, 42, 14, pauseBg);
      drawVectorPauseIcon(80, 217, p5 ? cAppleBlack : cAppleWhite);
    }

    bool p6 = (isHighlightActive && activeTouchBtnId == 6);
    uint16_t stopBg = p6 ? cAppleWhite : cAppleRed;
    canvas.fillRoundRect(166, 196, 148, 42, 14, stopBg);
    canvas.drawRoundRect(166, 196, 148, 42, 14, stopBg);
    drawVectorStopIcon(240, 217, p6 ? cAppleBlack : cAppleWhite);
  }

  canvas.pushSprite(0, 0);
}

// Stepper hold state
int holdingBtnId = 0;
uint32_t holdStartTime = 0;
uint32_t lastHoldRepeatTime = 0;
bool targetValueModified = false;

// ----------------------------------------------------
// Touch Screen Handler
// ----------------------------------------------------
void handleTouch() {
  M5.update();
  uint32_t now = millis();
  int touchCount = M5.Touch.getCount();

  if (touchCount > 0) {
    auto touchDetail = M5.Touch.getDetail(0);
    int tx = touchDetail.x;
    int ty = touchDetail.y;
    RemoteState state = currentState;

    if (!isHoldingTouch) {
      if (touchDetail.wasPressed()) {
        if (now - lastTouchTime < 250) return;
        lastTouchTime = now;

        Serial.printf("Touch detected: X=%d, Y=%d (State: %d)\n", tx, ty, (int)state);

        if (state == STATE_DISCONNECTED) {
          if (tx >= 220 && tx <= 315 && ty >= 0 && ty <= 26) {
            activeTouchBtnId = 10;
            activeTouchUntil = now + 150;
            pendingSound = SOUND_CONFIRM;
            startScan();
            return;
          }

          std::vector<ScannedDeviceInfo> devicesCopy;
          if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            devicesCopy = scannedDevicesCache;
            xSemaphoreGive(bleMutex);
          }

          for (size_t i = 0; i < devicesCopy.size() && i < 4; i++) {
            int yStart = 44 + i * 42;
            if (tx >= 8 && tx <= 312 && ty >= yStart && ty <= yStart + 38) {
              activeTouchBtnId = 11 + i;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
              selectedDeviceIdx = i;
              snprintf(targetDeviceAddress, sizeof(targetDeviceAddress), "%s", devicesCopy[i].address);
              doConnect = true;
              break;
            }
          }
        } else if (state == STATE_CONNECTED_STOPPED) {
          if (ty >= 26 && ty <= 108) {
            if (tx >= 0 && tx <= 85) {
              targetSpeed = (targetSpeed - 0.1f < 0.6f) ? 0.6f : targetSpeed - 0.1f;
              lastUserTargetChangeTime = now;
              activeTouchBtnId = 2;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
            } else if (tx >= 235 && tx <= 320) {
              targetSpeed = (targetSpeed + 0.1f > 4.0f) ? 4.0f : targetSpeed + 0.1f;
              lastUserTargetChangeTime = now;
              activeTouchBtnId = 1;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
            }
          } else if (ty >= 109 && ty <= 192) {
            if (tx >= 0 && tx <= 85) {
              holdingBtnId = 4;
              isHoldingTouch = true;
              holdStartTime = now;
              lastHoldRepeatTime = now;
              targetValueModified = true;
              targetIncline = (targetIncline - 1.0f < 0.0f) ? 0.0f : targetIncline - 1.0f;
              isPendingInclineChange = true;
              lastInclineProgressTime = now;
              lastObservedIncline = (float)currentIncline;
              lastUserTargetChangeTime = now;
              activeTouchBtnId = 4;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
            } else if (tx >= 235 && tx <= 320) {
              holdingBtnId = 3;
              isHoldingTouch = true;
              holdStartTime = now;
              lastHoldRepeatTime = now;
              targetValueModified = true;
              targetIncline = (targetIncline + 1.0f > 12.0f) ? 12.0f : targetIncline + 1.0f;
              isPendingInclineChange = true;
              lastInclineProgressTime = now;
              lastObservedIncline = (float)currentIncline;
              lastUserTargetChangeTime = now;
              activeTouchBtnId = 3;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
            }
          } else if (ty >= 193 && ty <= 240) {
            activeTouchBtnId = 5;
            activeTouchUntil = now + 150;
            pendingSound = SOUND_CONFIRM;
            startTreadmill();
          }
        } else {
          // Workout View
          if (ty >= 26 && ty <= 108) {
            if (tx >= 0 && tx <= 85) {
              holdingBtnId = 2;
              isHoldingTouch = true;
              holdStartTime = now;
              lastHoldRepeatTime = now;
              targetValueModified = true;
              targetSpeed = (targetSpeed - 0.1f < 0.6f) ? 0.6f : targetSpeed - 0.1f;
              lastUserTargetChangeTime = now;
              activeTouchBtnId = 2;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
            } else if (tx >= 235 && tx <= 320) {
              holdingBtnId = 1;
              isHoldingTouch = true;
              holdStartTime = now;
              lastHoldRepeatTime = now;
              targetValueModified = true;
              targetSpeed = (targetSpeed + 0.1f > 4.0f) ? 4.0f : targetSpeed + 0.1f;
              lastUserTargetChangeTime = now;
              activeTouchBtnId = 1;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
            }
          } else if (ty >= 109 && ty <= 192) {
            if (tx >= 0 && tx <= 85) {
              holdingBtnId = 4;
              isHoldingTouch = true;
              holdStartTime = now;
              lastHoldRepeatTime = now;
              targetValueModified = true;
              targetIncline = (targetIncline - 1.0f < 0.0f) ? 0.0f : targetIncline - 1.0f;
              isPendingInclineChange = true;
              lastInclineProgressTime = now;
              lastObservedIncline = (float)currentIncline;
              lastUserTargetChangeTime = now;
              activeTouchBtnId = 4;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
            } else if (tx >= 235 && tx <= 320) {
              holdingBtnId = 3;
              isHoldingTouch = true;
              holdStartTime = now;
              lastHoldRepeatTime = now;
              targetValueModified = true;
              targetIncline = (targetIncline + 1.0f > 12.0f) ? 12.0f : targetIncline + 1.0f;
              isPendingInclineChange = true;
              lastInclineProgressTime = now;
              lastObservedIncline = (float)currentIncline;
              lastUserTargetChangeTime = now;
              activeTouchBtnId = 3;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
            }
          } else if (ty >= 193 && ty <= 240) {
            if (tx >= 0 && tx <= 158) {
              activeTouchBtnId = 5;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
              if (isPaused) {
                resumeTreadmill();
              } else {
                pauseTreadmill();
              }
            } else if (tx >= 162 && tx <= 320) {
              activeTouchBtnId = 6;
              activeTouchUntil = now + 150;
              pendingSound = SOUND_CONFIRM;
              stopTreadmill();
            }
          }
        }
      }
    } else {
      if (isHoldingTouch && (now - holdStartTime >= 350)) {
        activeTouchBtnId = holdingBtnId;
        activeTouchUntil = now + 150;

        if (now - lastHoldRepeatTime >= 100) {
          lastHoldRepeatTime = now;
          if (holdingBtnId == 1) {
            targetSpeed = (targetSpeed + 0.1f > 4.0f) ? 4.0f : targetSpeed + 0.1f;
          } else if (holdingBtnId == 2) {
            targetSpeed = (targetSpeed - 0.1f < 0.6f) ? 0.6f : targetSpeed - 0.1f;
          } else if (holdingBtnId == 3) {
            targetIncline = (targetIncline + 1.0f > 12.0f) ? 12.0f : targetIncline + 1.0f;
            isPendingInclineChange = true;
            lastInclineProgressTime = now;
            lastObservedIncline = (float)currentIncline;
          } else if (holdingBtnId == 4) {
            targetIncline = (targetIncline - 1.0f < 0.0f) ? 0.0f : targetIncline - 1.0f;
            isPendingInclineChange = true;
            lastInclineProgressTime = now;
            lastObservedIncline = (float)currentIncline;
          }
          lastUserTargetChangeTime = now;
          playBeep(2200, 30);
        }
      }
    }
  } else {
    if (isHoldingTouch) {
      int releasedBtn = holdingBtnId;
      isHoldingTouch = false;
      holdingBtnId = 0;

      if (targetValueModified) {
        targetValueModified = false;
        lastUserTargetChangeTime = millis();
        if (releasedBtn == 1 || releasedBtn == 2) {
          setSpeed(targetSpeed);
          Serial.printf("RELEASED HOLD - Transmitted final speed: %.1f mph\n", (float)targetSpeed);
        } else if (releasedBtn == 3 || releasedBtn == 4) {
          setIncline(targetIncline);
          Serial.printf("RELEASED HOLD - Transmitted final incline: %.1f%%\n", (float)targetIncline);
        }
      }
    }
  }
}

// ----------------------------------------------------
// Setup
// ----------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Power.begin();
  Serial.begin(115200);
  Serial.println("M5Stack CoreS3 SE Remote Initializing (Stability Fix Active)...");

  // Create FreeRTOS Mutex
  bleMutex = xSemaphoreCreateMutex();

  // Initialize display
  M5.Display.init();
  M5.Display.setRotation(1);
  canvas.setColorDepth(16);
  canvas.createSprite(320, 240);

  M5.Speaker.setVolume(64);

  canvas.clear(canvas.color565(10, 14, 23));
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(canvas.color565(0, 240, 255));
  canvas.drawCenterString("INITIALIZING BLE...", 160, 100);
  canvas.pushSprite(0, 0);

  BLEDevice::init("M5_FTMS_Remote");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  startScan();
}

// ----------------------------------------------------
// Main Loop
// ----------------------------------------------------
void loop() {
  handleTouch();
  processPendingSound();

  RemoteState state = currentState;

  // Periodic heartbeat
  if ((state == STATE_CONNECTED_RUNNING || state == STATE_CONNECTED_STOPPED) && pVendorWriteChar != nullptr) {
    if (millis() - lastHeartbeatTime > 1000) {
      lastHeartbeatTime = millis();
      sendMerachHeartbeat();
    }
  }

  // Serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      char cmdChar = input.charAt(0);
      int val = input.substring(1).toInt();

      if (cmdChar == 's') {
        setSpeed(val / 100.0f);
      } else if (cmdChar == 'i' || cmdChar == 'j') {
        setIncline(val / 10.0f);
      } else if (cmdChar == 'u') {
        startTreadmill();
      } else if (cmdChar == 'd') {
        stopTreadmill();
      }
    }
  }

  // Connect request
  if (doConnect && strlen(targetDeviceAddress) > 0) {
    doConnect = false;
    connectToDeviceAddress(targetDeviceAddress);
  }

  // Auto scan and auto connect when disconnected
  if (state == STATE_DISCONNECTED && !isScanning && !doConnect) {
    if (millis() - lastScanTime > 10000) {
      startScan();

      std::vector<ScannedDeviceInfo> devicesCopy;
      if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        devicesCopy = scannedDevicesCache;
        xSemaphoreGive(bleMutex);
      }

      if (devicesCopy.size() == 1) {
        setStatusMessage("Auto-connecting...");
        selectedDeviceIdx = 0;
        snprintf(targetDeviceAddress, sizeof(targetDeviceAddress), "%s", devicesCopy[0].address);
        doConnect = true;
      }
    }
  }

  // Telemetry Watchdog
  if (state == STATE_CONNECTED_RUNNING || state == STATE_CONNECTED_STOPPED) {
    uint32_t lastNotif = lastNotificationTime;
    if (lastNotif > 0 && (millis() - lastNotif > 12000)) {
      Serial.println("Watchdog Timeout: No telemetry for 12s. Disconnecting...");
      setStatusMessage("Timeout: Reconnecting...");
      if (pClient != nullptr) {
        pClient->disconnect();
      }
    }
  }

  // UI Redraw ~30 FPS
  if (millis() - lastUiRedraw > 33) {
    drawUI();
    lastUiRedraw = millis();
  }

  delay(5);
}
