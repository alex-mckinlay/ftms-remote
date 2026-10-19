#include <iostream>
#include <cassert>
#include <cmath>
#include <cstring>
#include <atomic>
#include <chrono>
#include <vector>
#include <string>

enum RemoteState {
  STATE_DISCONNECTED,
  STATE_CONNECTING,
  STATE_CONNECTED_STOPPED,
  STATE_CONNECTED_RUNNING,
  STATE_ERROR
};

struct SimulatedState {
  RemoteState currentState{STATE_DISCONNECTED};
  bool isRunning{false};
  bool isPaused{false};
  bool isTreadmillActive{false};
  float currentSpeed{0.0f};   // km/h
  float currentIncline{0.0f}; // %
  float targetSpeed{0.0f};    // mph
  float targetIncline{0.0f};  // %
  uint32_t lastUserTargetChangeTime{0};
  bool isHoldingTouch{false};
  uint32_t nowMs{0};

  // Closed-Loop Incline State Machine
  bool isPendingInclineChange{false};
  float lastObservedIncline{0.0f};
  uint32_t lastInclineProgressTime{0};
};

const uint32_t RAMP_SETTLE_WINDOW_MS = 5000;

std::vector<uint8_t> simulateSendMerachCompoundCommand(float speedMph, float inclinePercent) {
  if (speedMph > 0.0f) {
    if (speedMph < 0.6f) speedMph = 0.6f;
    if (speedMph > 4.0f) speedMph = 4.0f;
  }
  if (inclinePercent < 0.0f) inclinePercent = 0.0f;
  if (inclinePercent > 12.0f) inclinePercent = 12.0f;

  uint8_t rawSpeed = (speedMph > 0.0f) ? (uint8_t)(speedMph * 10.0f + 0.5f) : 0;
  uint8_t rawIncl = (uint8_t)(inclinePercent + 0.5f);
  uint8_t cs = 0x53 ^ 0x02 ^ rawSpeed ^ rawIncl;

  std::vector<uint8_t> pkt = {0x02, 0x53, 0x02, rawSpeed, rawIncl, cs, 0x03};
  return pkt;
}

void simulateTreadmillDataCallback(SimulatedState &st, const uint8_t *pData, size_t length) {
  if (length < 2) return;

  uint16_t flags = pData[0] | (pData[1] << 8);
  size_t offset = 2;

  // Bit 0: Speed (2 bytes, 0.01 km/h)
  if ((flags & (1 << 0)) == 0) {
    if (offset + 2 <= length) {
      uint16_t rawSpeed = pData[offset] | (pData[offset + 1] << 8);
      st.currentSpeed = rawSpeed / 100.0f;
      offset += 2;
    }
  }

  // Bit 1: Average Speed Present
  if (flags & (1 << 1)) offset += 2;

  // Bit 2: Total Distance Present
  if (flags & (1 << 2)) offset += 3;

  // Bit 3: Inclination Present (2 bytes, 0.1%)
  if (flags & (1 << 3)) {
    if (offset + 2 <= length) {
      int16_t rawIncline = pData[offset] | (pData[offset + 1] << 8);
      st.currentIncline = rawIncline / 10.0f;
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

  if (flags & (1 << 10)) {
    if (offset + 2 <= length) offset += 2;
  }

  // Dynamically adjust remote running states (main.cpp)
  float spd = st.currentSpeed;
  if (spd > 0.05f) {
    st.isTreadmillActive = true;
    st.isRunning = true;
    st.isPaused = false;
    if (st.currentState == STATE_CONNECTED_STOPPED || st.currentState == STATE_CONNECTING) {
      st.currentState = STATE_CONNECTED_RUNNING;
    }
    float spdMph = spd * 0.621371f;
    if (spdMph < 0.6f) spdMph = 0.6f;
    if (spdMph > 4.0f) spdMph = 4.0f;

    bool userIsOverridingSpeed = st.isHoldingTouch || (st.nowMs - st.lastUserTargetChangeTime < RAMP_SETTLE_WINDOW_MS);

    if (!userIsOverridingSpeed) {
      st.targetSpeed = spdMph;
    }

    // Closed-Loop Incline State Machine Evaluation
    float incl = (float)st.currentIncline;
    uint32_t now = st.nowMs;

    if (st.isPendingInclineChange) {
      float target = (float)st.targetIncline;
      float prevObserved = (float)st.lastObservedIncline;

      if (std::abs(incl - target) <= 0.15f) {
        st.isPendingInclineChange = false;
        st.targetIncline = incl;
        st.lastObservedIncline = incl;
      } else {
        bool movingTowards = (target > prevObserved && incl > prevObserved) ||
                             (target < prevObserved && incl < prevObserved);
        bool movingAway = (target > prevObserved && incl < prevObserved) ||
                          (target < prevObserved && incl > prevObserved);

        if (movingTowards) {
          st.lastInclineProgressTime = now;
          st.lastObservedIncline = incl;
        } else if (movingAway) {
          st.isPendingInclineChange = false;
          st.targetIncline = incl;
          st.lastObservedIncline = incl;
        } else {
          if (now - st.lastInclineProgressTime > 3000) {
            st.isPendingInclineChange = false;
            st.targetIncline = incl;
            st.lastObservedIncline = incl;
          }
        }
      }
    } else if (!st.isHoldingTouch) {
      st.targetIncline = incl;
      st.lastObservedIncline = incl;
    }
  } else {
    st.isTreadmillActive = false;
    if (!st.isPaused) {
      st.isRunning = false;
      st.targetSpeed = 0.6f;
      if (st.currentState != STATE_DISCONNECTED) {
        st.currentState = STATE_CONNECTED_STOPPED;
      }
    }
  }
}

// Helper to build FTMS notification packet
std::vector<uint8_t> buildFtmsPacket(float speedKmH, float inclinePercent) {
  std::vector<uint8_t> pkt;
  uint16_t flags = 0; // Bit 0 is 0 -> Speed present
  flags |= (1 << 3);  // Bit 3 -> Incline present

  pkt.push_back(flags & 0xFF);
  pkt.push_back((flags >> 8) & 0xFF);

  uint16_t rawSpeed = (uint16_t)(speedKmH * 100.0f + 0.5f);
  pkt.push_back(rawSpeed & 0xFF);
  pkt.push_back((rawSpeed >> 8) & 0xFF);

  int16_t rawIncline = (int16_t)(inclinePercent * 10.0f);
  pkt.push_back(rawIncline & 0xFF);
  pkt.push_back((rawIncline >> 8) & 0xFF);
  pkt.push_back(0); // Ramp angle byte 1
  pkt.push_back(0); // Ramp angle byte 2

  return pkt;
}

void simulateStartTreadmill(SimulatedState &st) {
  st.isRunning = true;
  st.isPaused = false;
  st.currentState = STATE_CONNECTED_RUNNING;
  if (st.targetSpeed < 0.6f) {
    st.targetSpeed = 0.6f;
  }
}

void simulateStopTreadmill(SimulatedState &st) {
  st.isRunning = false;
  st.isPaused = false;
  st.isTreadmillActive = false;
  st.targetSpeed = 0.6f;
  st.currentState = STATE_CONNECTED_STOPPED;
}

void simulatePauseTreadmill(SimulatedState &st) {
  st.isPaused = true;
  st.isRunning = true;
}

void simulateSetSpeed(SimulatedState &st, float speedMph) {
  if (speedMph < 0.6f) speedMph = 0.6f;
  if (speedMph > 4.0f) speedMph = 4.0f;
  st.targetSpeed = speedMph;
  st.lastUserTargetChangeTime = st.nowMs;
}

void simulateSetIncline(SimulatedState &st, float inclinePercent) {
  if (inclinePercent < 0.0f) inclinePercent = 0.0f;
  if (inclinePercent > 12.0f) inclinePercent = 12.0f;
  st.targetIncline = inclinePercent;
  st.isPendingInclineChange = true;
  st.lastInclineProgressTime = st.nowMs;
  st.lastObservedIncline = st.currentIncline;
  st.lastUserTargetChangeTime = st.nowMs;
}

int main() {
  std::cout << "========================================================\n";
  std::cout << "EMPIRICAL TEST HARNESS FOR CLOSED-LOOP INCLINE CONTROL\n";
  std::cout << "========================================================\n\n";

  int passCount = 0;
  int failCount = 0;

  // TEST 1: R2 - Target speed reset on stop and safe ramping startup
  std::cout << "[TEST 1] R2: Target speed reset on stop and safe ramping startup\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_RUNNING;
    st.isRunning = true;
    st.isTreadmillActive = true;
    st.currentSpeed = 3.22f; // ~2.0 mph
    st.targetSpeed = 2.0f;

    simulateStopTreadmill(st);
    if (st.targetSpeed == 0.6f && st.currentState == STATE_CONNECTED_STOPPED && !st.isRunning) {
      std::cout << "  -> PASS: targetSpeed reset to 0.6 mph on explicit stop.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: targetSpeed was " << st.targetSpeed << " expected 0.6 mph.\n";
      failCount++;
    }

    simulateStartTreadmill(st);
    if (st.targetSpeed >= 0.6f && st.currentState == STATE_CONNECTED_RUNNING) {
      std::cout << "  -> PASS: safe ramping started at " << st.targetSpeed << " mph.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: start targetSpeed was " << st.targetSpeed << "\n";
      failCount++;
    }
  }

  // TEST 2: R1 - Rapid Touch Buttons vs Idle Telemetry Sync
  std::cout << "\n[TEST 2] R1: Rapid button presses within settle window\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_RUNNING;
    st.nowMs = 1000;
    st.currentSpeed = 1.61f; // 1.0 mph
    st.targetSpeed = 1.0f;

    for (int i = 0; i < 5; i++) {
      st.nowMs += 80;
      st.isHoldingTouch = true;
      simulateSetSpeed(st, st.targetSpeed + 0.1f);
      st.isHoldingTouch = false;
    }

    st.nowMs = 1800;
    auto pkt = buildFtmsPacket(1.93f, 0.0f);
    simulateTreadmillDataCallback(st, pkt.data(), pkt.size());

    if (std::abs(st.targetSpeed - 1.5f) < 0.01f) {
      std::cout << "  -> PASS: targetSpeed preserved at " << st.targetSpeed << " mph during override window.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: targetSpeed overwritten to " << st.targetSpeed << " mph by telemetry!\n";
      failCount++;
    }
  }

  // TEST 3: R1 Motor Acceleration Settle Window Protection
  std::cout << "\n[TEST 3] R1: Motor acceleration within 5000ms ramp settle window\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_RUNNING;
    st.nowMs = 1000;
    st.currentSpeed = 0.965f; // 0.6 mph
    st.targetSpeed = 0.6f;

    simulateSetSpeed(st, 4.0f); // User sets speed from 0.6 to 4.0 mph

    st.nowMs = 3500;
    auto pkt = buildFtmsPacket(4.02f, 0.0f); // 2.5 mph intermediate
    simulateTreadmillDataCallback(st, pkt.data(), pkt.size());

    if (std::abs(st.targetSpeed - 4.0f) < 0.01f) {
      std::cout << "  -> PASS: targetSpeed maintained at 4.0 mph during acceleration window.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: targetSpeed overwritten to " << st.targetSpeed << " mph!\n";
      failCount++;
    }
  }

  // TEST 4: Pause State Telemetry Protection
  std::cout << "\n[TEST 4] R1 & R2: Pause command followed by zero speed telemetry\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_RUNNING;
    st.nowMs = 1000;
    st.currentSpeed = 4.83f; // 3.0 mph
    st.targetSpeed = 3.0f;

    simulatePauseTreadmill(st);

    st.nowMs = 2500;
    auto pkt = buildFtmsPacket(0.0f, 0.0f);
    simulateTreadmillDataCallback(st, pkt.data(), pkt.size());

    if (st.isPaused && st.currentState == STATE_CONNECTED_RUNNING) {
      std::cout << "  -> PASS: PAUSED state preserved when belt reached 0 km/h.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: PAUSED state destroyed by zero-speed telemetry! isPaused="
                << st.isPaused << ", state=" << st.currentState << "\n";
      failCount++;
    }
  }

  // TEST 5: Bidirectional External Changes Sync
  std::cout << "\n[TEST 5] R1: Bidirectional tracking for external speed & incline changes\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_RUNNING;
    st.nowMs = 1000;
    st.lastUserTargetChangeTime = 0;
    st.isPendingInclineChange = false;
    st.currentSpeed = 1.61f; // 1.0 mph
    st.currentIncline = 0.0f;

    st.nowMs = 20000;
    auto pkt1 = buildFtmsPacket(2.5f, 4.0f);
    simulateTreadmillDataCallback(st, pkt1.data(), pkt1.size());

    bool speedUpOk = (std::abs(st.targetSpeed - (2.5f * 0.621371f)) < 0.05f) && (st.targetIncline == 4.0f);

    st.nowMs = 40000;
    auto pkt2 = buildFtmsPacket(1.0f, 1.0f);
    simulateTreadmillDataCallback(st, pkt2.data(), pkt2.size());

    bool dropOk = (std::abs(st.targetSpeed - (1.0f * 0.621371f)) < 0.05f) && (st.targetIncline == 1.0f);

    if (speedUpOk && dropOk) {
      std::cout << "  -> PASS: Bidirectional sync tracked external increases and drops cleanly.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: External sync failed. speedUpOk=" << speedUpOk << ", dropOk=" << dropOk << "\n";
      failCount++;
    }
  }

  // TEST 6: Bit 3 Incline Packet Parsing Boundary
  std::cout << "\n[TEST 6] Parsing Edge Case: 6-byte incline packet buffer boundary\n";
  {
    SimulatedState st;
    uint8_t truncatedPkt[6] = {0x08, 0x00, 0x64, 0x00, 0x32, 0x00};
    simulateTreadmillDataCallback(st, truncatedPkt, 6);

    if (st.currentIncline == 5.0f) {
      std::cout << "  -> PASS: Truncated 6-byte packet incline parsed correctly as 5.0%.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: Incline parsed as " << st.currentIncline << "%\n";
      failCount++;
    }
  }

  // TEST 7: Stopped Incline Vendor Command Formatting
  std::cout << "\n[TEST 7] Stopped Incline Vendor Command Formatting\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_STOPPED;
    st.isRunning = false;
    st.targetSpeed = 0.6f;

    simulateSetIncline(st, 5.0f);

    auto cmd = simulateSendMerachCompoundCommand(0.0f, st.targetIncline);

    if (cmd.size() == 7 && cmd[0] == 0x02 && cmd[1] == 0x53 && cmd[2] == 0x02 && cmd[3] == 0x00 && cmd[4] == 0x05 && cmd[6] == 0x03) {
      std::cout << "  -> PASS: Stopped incline command formatted with Speed_Raw=0x00 and Incline_Raw=0x05 (5%).\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: Vendor compound command misformatted when stopped! Speed_Raw=0x" << std::hex << (int)cmd[3] << "\n";
      failCount++;
    }
  }

  // TEST 8: Closed-Loop Incline Target Arrival (Immediate Release)
  std::cout << "\n[TEST 8] Closed-Loop Incline Target Arrival Detection\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_RUNNING;
    st.nowMs = 1000;
    st.currentIncline = 0.0f;
    simulateSetIncline(st, 6.0f); // Target = 6.0%, isPendingInclineChange = true

    // Step 1: Telemetry shows progress (2.0% at t=2000ms)
    st.nowMs = 2000;
    auto pkt1 = buildFtmsPacket(3.22f, 2.0f);
    simulateTreadmillDataCallback(st, pkt1.data(), pkt1.size());
    bool inProgress = st.isPendingInclineChange && (st.targetIncline == 6.0f);

    // Step 2: Telemetry shows target reached (6.0% at t=4000ms)
    st.nowMs = 4000;
    auto pkt2 = buildFtmsPacket(3.22f, 6.0f);
    simulateTreadmillDataCallback(st, pkt2.data(), pkt2.size());
    bool arrived = !st.isPendingInclineChange && (st.targetIncline == 6.0f);

    if (inProgress && arrived) {
      std::cout << "  -> PASS: Target arrival immediately unlocked pending state at t=4000ms (no arbitrary wait!).\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: Closed-loop arrival failed. inProgress=" << inProgress << ", arrived=" << arrived << "\n";
      failCount++;
    }
  }

  // TEST 9: Closed-Loop External Override Detection
  std::cout << "\n[TEST 9] Closed-Loop External Physical Incline Override Detection\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_RUNNING;
    st.nowMs = 1000;
    st.currentIncline = 5.0f;
    st.lastObservedIncline = 5.0f;

    // User sets target to 10.0%
    simulateSetIncline(st, 10.0f);

    // Physical remote on treadmill presses INCLINE DOWN (telemetry drops to 4.0%)
    st.nowMs = 2000;
    auto pkt = buildFtmsPacket(3.22f, 4.0f);
    simulateTreadmillDataCallback(st, pkt.data(), pkt.size());

    if (!st.isPendingInclineChange && st.targetIncline == 4.0f) {
      std::cout << "  -> PASS: External physical incline drop immediately overrode target and synced to 4.0%.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: External override failed. pending=" << st.isPendingInclineChange << ", target=" << st.targetIncline << "\n";
      failCount++;
    }
  }

  // TEST 10: Closed-Loop Incline Motion Stall / Limit Detection
  std::cout << "\n[TEST 10] Closed-Loop Incline Stall / Mechanical Limit Detection\n";
  {
    SimulatedState st;
    st.currentState = STATE_CONNECTED_RUNNING;
    st.nowMs = 1000;
    st.currentIncline = 0.0f;
    simulateSetIncline(st, 12.0f); // Target = 12%

    // Motor tilts to 8.0% at t=3000ms, then mechanical limit is reached
    st.nowMs = 3000;
    auto pkt1 = buildFtmsPacket(3.22f, 8.0f);
    simulateTreadmillDataCallback(st, pkt1.data(), pkt1.size());

    // 3500ms pass with no further incline change (t=6500ms)
    st.nowMs = 6500;
    auto pkt2 = buildFtmsPacket(3.22f, 8.0f);
    simulateTreadmillDataCallback(st, pkt2.data(), pkt2.size());

    if (!st.isPendingInclineChange && st.targetIncline == 8.0f) {
      std::cout << "  -> PASS: Incline stall detected at 8.0% limit after 3s; target synced cleanly.\n";
      passCount++;
    } else {
      std::cout << "  -> FAIL: Stall detection failed. pending=" << st.isPendingInclineChange << ", target=" << st.targetIncline << "\n";
      failCount++;
    }
  }

  std::cout << "\n========================================================\n";
  std::cout << "TEST SUMMARY: " << passCount << " PASSED, " << failCount << " FAILED\n";
  std::cout << "========================================================\n";

  return (failCount == 0) ? 0 : 1;
}
