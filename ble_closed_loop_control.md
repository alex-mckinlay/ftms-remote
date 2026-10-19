# Embedded BLE Control & Closed-Loop Telemetry Reference Guide

This document captures reusable design patterns for Bluetooth Low Energy (BLE) hardware control and closed-loop telemetry synchronization.

---

## 1. Asynchronous BLE GATT Service & Characteristic Discovery

### Problem
In embedded BLE stacks (ESP32 IDF, NimBLE, BlueZ), calling direct single-shot service lookups `pClient->getService(UUID)` immediately after `pClient->connect()` frequently returns `nullptr` because:
1. GATT service discovery over L2CAP/ATT is asynchronous and requires a brief handshake window.
2. 128-bit custom vendor UUID representations may differ between BLE libraries and hardware peripherals.

### Solution Pattern
Implement a connection settle delay (150 ms) followed by a full GATT scan that iterates through all primary services and characteristics:

```cpp
// Allow BLE stack to finish service discovery handshake
delay(150);

pVendorWriteChar = nullptr;
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
                            
                            // Match short UUID substring or GATT handle
                            if (uuidStr.find("fff2") != std::string::npos || 
                                uuidStr.find("FFF2") != std::string::npos || 
                                handle == 0x003A) {
                                pVendorWriteChar = pChar;
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
```

---

## 2. Closed-Loop Telemetry & Motor Ramping State Machine

### Problem
Hardware actuators (motors, incline lifts, thermal elements) take variable time to move to target setpoints. Open-loop blind lockout timers (e.g. fixed 15-second timers) create latency, fail when motors stall, and ignore external manual button presses.

### Solution Pattern
Implement a **closed-loop target state machine** evaluating incoming telemetry on every notification frame:

```cpp
float incl = (float)currentIncline;
uint32_t now = millis();

if (isPendingInclineChange) {
    float target = (float)targetIncline;
    float prevObserved = (float)lastObservedIncline;

    // 1. Immediate Target Arrival: Unlock pending state instantly
    if (std::abs(incl - target) <= 0.15f) {
        isPendingInclineChange = false;
        targetIncline = incl;
        lastObservedIncline = incl;
    } 
    // 2. Directional Movement Check
    else {
        bool movingTowards = (target > prevObserved && incl > prevObserved) ||
                             (target < prevObserved && incl < prevObserved);
        bool movingAway = (target > prevObserved && incl < prevObserved) ||
                          (target < prevObserved && incl > prevObserved);

        if (movingTowards) {
            lastInclineProgressTime = now;
            lastObservedIncline = incl;
        } 
        // 3. External Physical Remote/Console Override
        else if (movingAway) {
            isPendingInclineChange = false;
            targetIncline = incl;
            lastObservedIncline = incl;
        } 
        // 4. Stall / Mechanical Limit Detection (3000ms no progress)
        else {
            if (now - lastInclineProgressTime > 3000) {
                isPendingInclineChange = false;
                targetIncline = incl;
                lastObservedIncline = incl;
            }
        }
    }
} else if (!isHoldingTouch) {
    // Normal bidirectional sync when no user command is pending
    targetIncline = incl;
    lastObservedIncline = incl;
}
```

---

## 3. Vendor Compound BLE Command Formatting Rules
- When transmitting vendor compound control frames (such as Merach `[0x02, 0x53, 0x02, Speed_Raw, Incline_Raw, Checksum, 0x03]`), format `Speed_Raw = 0x00` (0.0 mph) when the motor belt is stopped.
- Never clamp stopped speed (`0.0 mph`) to minimum running speed (`0.6 mph`), as doing so causes peripheral firmware to reject commands or execute unsafe motor starts.
