import time
import math

class RemoteState:
    STATE_DISCONNECTED = 0
    STATE_CONNECTING = 1
    STATE_CONNECTED_STOPPED = 2
    STATE_CONNECTED_RUNNING = 3
    STATE_ERROR = 4

class SimulatedState:
    def __init__(self):
        self.currentState = RemoteState.STATE_DISCONNECTED
        self.isRunning = False
        self.isPaused = False
        self.isTreadmillActive = False
        self.currentSpeed = 0.0   # km/h
        self.currentIncline = 0.0 # %
        self.targetSpeed = 0.0    # mph
        self.targetIncline = 0.0  # %
        self.lastUserTargetChangeTime = 0
        self.isHoldingTouch = False
        self.nowMs = 0

        # Closed-Loop Incline State Machine
        self.isPendingInclineChange = False
        self.lastObservedIncline = 0.0
        self.lastInclineProgressTime = 0

RAMP_SETTLE_WINDOW_MS = 5000

def send_merach_compound_command(speed_mph: float, incline_percent: float) -> bytes:
    if speed_mph > 0.0:
        if speed_mph < 0.6: speed_mph = 0.6
        if speed_mph > 4.0: speed_mph = 4.0
    if incline_percent < 0.0: incline_percent = 0.0
    if incline_percent > 12.0: incline_percent = 12.0

    raw_speed = int(speed_mph * 10.0 + 0.5) if speed_mph > 0.0 else 0
    raw_incl = int(incline_percent + 0.5)
    cs = 0x53 ^ 0x02 ^ raw_speed ^ raw_incl

    return bytes([0x02, 0x53, 0x02, raw_speed, raw_incl, cs, 0x03])

def treadmill_data_callback(st: SimulatedState, pData: bytes):
    if len(pData) < 2:
        return

    flags = pData[0] | (pData[1] << 8)
    offset = 2

    # Bit 0: Speed (2 bytes, 0.01 km/h)
    if (flags & (1 << 0)) == 0:
        if offset + 2 <= len(pData):
            rawSpeed = pData[offset] | (pData[offset + 1] << 8)
            st.currentSpeed = rawSpeed / 100.0
            offset += 2

    # Bit 1: Average Speed
    if flags & (1 << 1):
        offset += 2

    # Bit 2: Total Distance
    if flags & (1 << 2):
        offset += 3

    # Bit 3: Inclination Present
    if flags & (1 << 3):
        if offset + 2 <= len(pData):
            rawIncline = int.from_bytes(pData[offset:offset+2], byteorder='little', signed=True)
            st.currentIncline = rawIncline / 10.0
        if offset + 4 <= len(pData):
            offset += 4
        else:
            offset += 2

    spd = st.currentSpeed
    if spd > 0.05:
        st.isTreadmillActive = True
        st.isRunning = True
        st.isPaused = False
        if st.currentState in (RemoteState.STATE_CONNECTED_STOPPED, RemoteState.STATE_CONNECTING):
            st.currentState = RemoteState.STATE_CONNECTED_RUNNING
        spdMph = spd * 0.621371
        if spdMph < 0.6: spdMph = 0.6
        if spdMph > 4.0: spdMph = 4.0

        userIsOverridingSpeed = st.isHoldingTouch or (st.nowMs - st.lastUserTargetChangeTime < RAMP_SETTLE_WINDOW_MS)
        if not userIsOverridingSpeed:
            st.targetSpeed = spdMph

        # Closed-Loop Incline State Machine
        incl = float(st.currentIncline)
        now = st.nowMs

        if st.isPendingInclineChange:
            target = float(st.targetIncline)
            prevObserved = float(st.lastObservedIncline)

            if abs(incl - target) <= 0.15:
                st.isPendingInclineChange = False
                st.targetIncline = incl
                st.lastObservedIncline = incl
            else:
                movingTowards = (target > prevObserved and incl > prevObserved) or (target < prevObserved and incl < prevObserved)
                movingAway = (target > prevObserved and incl < prevObserved) or (target < prevObserved and incl > prevObserved)

                if movingTowards:
                    st.lastInclineProgressTime = now
                    st.lastObservedIncline = incl
                elif movingAway:
                    st.isPendingInclineChange = False
                    st.targetIncline = incl
                    st.lastObservedIncline = incl
                else:
                    if now - st.lastInclineProgressTime > 3000:
                        st.isPendingInclineChange = False
                        st.targetIncline = incl
                        st.lastObservedIncline = incl
        elif not st.isHoldingTouch:
            st.targetIncline = incl
            st.lastObservedIncline = incl
    else:
        st.isTreadmillActive = False
        if not st.isPaused:
            st.isRunning = False
            st.targetSpeed = 0.6
            if st.currentState != RemoteState.STATE_DISCONNECTED:
                st.currentState = RemoteState.STATE_CONNECTED_STOPPED

def build_ftms_packet(speed_kmh: float, incline_percent: float) -> bytes:
    flags = 1 << 3 # Bit 3 set (Incline)
    raw_speed = int(speed_kmh * 100.0 + 0.5)
    raw_incline = int(incline_percent * 10.0)
    
    pkt = bytearray()
    pkt.extend(flags.to_bytes(2, 'little'))
    pkt.extend(raw_speed.to_bytes(2, 'little'))
    pkt.extend(raw_incline.to_bytes(2, 'little', signed=True))
    pkt.extend((0).to_bytes(2, 'little')) # ramp angle
    return bytes(pkt)

def stop_treadmill(st: SimulatedState):
    st.isRunning = False
    st.isPaused = False
    st.isTreadmillActive = False
    st.targetSpeed = 0.6
    st.currentState = RemoteState.STATE_CONNECTED_STOPPED

def start_treadmill(st: SimulatedState):
    st.isRunning = True
    st.isPaused = False
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    if st.targetSpeed < 0.6:
        st.targetSpeed = 0.6

def pause_treadmill(st: SimulatedState):
    st.isPaused = True
    st.isRunning = True

def set_speed(st: SimulatedState, speed_mph: float):
    if speed_mph < 0.6: speed_mph = 0.6
    if speed_mph > 4.0: speed_mph = 4.0
    st.targetSpeed = speed_mph
    st.lastUserTargetChangeTime = st.nowMs

def set_incline(st: SimulatedState, incline_percent: float):
    if incline_percent < 0.0: incline_percent = 0.0
    if incline_percent > 12.0: incline_percent = 12.0
    st.targetIncline = incline_percent
    st.isPendingInclineChange = True
    st.lastInclineProgressTime = st.nowMs
    st.lastObservedIncline = float(st.currentIncline)
    st.lastUserTargetChangeTime = st.nowMs

def run_tests():
    print("========================================================")
    print("EMPIRICAL TEST SUITE FOR CLOSED-LOOP INCLINE CONTROL (PY)")
    print("========================================================")
    
    passes = 0
    failures = 0

    # Test 1: R2 - Belt stop resets targetSpeed to 0.6 mph & safe ramping
    print("\n[TEST 1] R2: Target Speed Reset on Belt Stop & Safe Ramping")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    st.isRunning = True
    st.targetSpeed = 2.5
    stop_treadmill(st)
    if st.targetSpeed == 0.6 and st.currentState == RemoteState.STATE_CONNECTED_STOPPED:
        print("  -> PASS: targetSpeed reset to 0.6 mph on belt stop.")
        passes += 1
    else:
        print(f"  -> FAIL: targetSpeed = {st.targetSpeed}, expected 0.6 mph.")
        failures += 1
        
    start_treadmill(st)
    if st.targetSpeed == 0.6 and st.currentState == RemoteState.STATE_CONNECTED_RUNNING:
        print("  -> PASS: Startup safe ramping verified (starts at 0.6 mph).")
        passes += 1
    else:
        print(f"  -> FAIL: Startup targetSpeed = {st.targetSpeed}")
        failures += 1

    # Test 2: R1 - Rapid touch button presses within 5000ms window
    print("\n[TEST 2] R1: Rapid Touch Button Presses vs Telemetry Sync")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    st.nowMs = 1000
    st.targetSpeed = 1.0
    for _ in range(5):
        st.nowMs += 80
        set_speed(st, st.targetSpeed + 0.1)
    
    st.nowMs = 1800
    pkt = build_ftms_packet(1.93, 0.0) # 1.2 mph intermediate speed
    treadmill_data_callback(st, pkt)
    
    if abs(st.targetSpeed - 1.5) < 0.01:
        print(f"  -> PASS: targetSpeed preserved at {st.targetSpeed:.1f} mph during override window.")
        passes += 1
    else:
        print(f"  -> FAIL: targetSpeed overwritten to {st.targetSpeed:.1f} mph during override window.")
        failures += 1

    # Test 3: R1 - Acceleration settle window
    print("\n[TEST 3] R1: Motor acceleration within 5000ms ramp settle window")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    st.nowMs = 1000
    set_speed(st, 4.0)
    
    st.nowMs = 3500
    pkt = build_ftms_packet(4.02, 0.0)
    treadmill_data_callback(st, pkt)
    
    if abs(st.targetSpeed - 4.0) < 0.01:
        print("  -> PASS: targetSpeed maintained at 4.0 mph during acceleration window.")
        passes += 1
    else:
        print(f"  -> FAIL: targetSpeed overwritten to {st.targetSpeed:.1f} mph.")
        failures += 1

    # Test 4: R1/R2 - Pause state preserved on zero speed telemetry
    print("\n[TEST 4] R1 & R2: Pause command preserved on zero speed telemetry")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    st.nowMs = 1000
    st.targetSpeed = 3.0
    pause_treadmill(st)
    
    st.nowMs = 2500
    pkt = build_ftms_packet(0.0, 0.0)
    treadmill_data_callback(st, pkt)
    
    if st.isPaused and st.currentState == RemoteState.STATE_CONNECTED_RUNNING:
        print("  -> PASS: PAUSED state preserved when belt reached 0 km/h.")
        passes += 1
    else:
        print(f"  -> FAIL: PAUSED state destroyed by zero-speed telemetry! isPaused={st.isPaused}")
        failures += 1

    # Test 5: R1 - Bidirectional Speed & Incline Sync
    print("\n[TEST 5] R1: Bidirectional Speed & Incline Sync for External Changes")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    st.nowMs = 1000
    st.lastUserTargetChangeTime = 0
    st.isPendingInclineChange = False
    
    st.nowMs = 20000
    pkt1 = build_ftms_packet(2.5, 4.0)
    treadmill_data_callback(st, pkt1)
    sync_inc = abs(st.targetSpeed - (2.5 * 0.621371)) < 0.05 and st.targetIncline == 4.0
    
    st.nowMs = 40000
    pkt2 = build_ftms_packet(1.0, 1.0)
    treadmill_data_callback(st, pkt2)
    sync_drop = abs(st.targetSpeed - (1.0 * 0.621371)) < 0.05 and st.targetIncline == 1.0
    
    if sync_inc and sync_drop:
        print("  -> PASS: Bidirectional sync tracked external increases and drops cleanly.")
        passes += 1
    else:
        print(f"  -> FAIL: Bidirectional sync failed. sync_inc={sync_inc}, sync_drop={sync_drop}")
        failures += 1

    # Test 6: Incline Packet Parsing Boundary (6-byte truncated packet)
    print("\n[TEST 6] Parsing Edge Case: 6-byte incline packet buffer boundary")
    st = SimulatedState()
    truncatedPkt = bytes([0x08, 0x00, 0x64, 0x00, 0x32, 0x00])
    treadmill_data_callback(st, truncatedPkt)
    if st.currentIncline == 5.0:
        print("  -> PASS: Truncated 6-byte packet incline parsed correctly as 5.0%.")
        passes += 1
    else:
        print(f"  -> FAIL: Incline parsed as {st.currentIncline}%")
        failures += 1

    # Test 7: Stopped Incline Vendor Command Formatting
    print("\n[TEST 7] Stopped Incline Vendor Command Formatting")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_STOPPED
    st.targetSpeed = 0.6
    set_incline(st, 5.0)
    cmd = send_merach_compound_command(0.0, st.targetIncline)
    if len(cmd) == 7 and cmd[0] == 0x02 and cmd[1] == 0x53 and cmd[2] == 0x02 and cmd[3] == 0x00 and cmd[4] == 0x05 and cmd[6] == 0x03:
        print("  -> PASS: Stopped incline command formatted with Speed_Raw=0x00 and Incline_Raw=0x05 (5%).")
        passes += 1
    else:
        print(f"  -> FAIL: Stopped vendor compound command misformatted! Speed_Raw=0x{cmd[3]:02X}")
        failures += 1

    # Test 8: Closed-Loop Incline Target Arrival (Immediate Release)
    print("\n[TEST 8] Closed-Loop Incline Target Arrival Detection")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    st.nowMs = 1000
    st.currentIncline = 0.0
    set_incline(st, 6.0)

    st.nowMs = 2000
    pkt1 = build_ftms_packet(3.22, 2.0)
    treadmill_data_callback(st, pkt1)
    inProgress = st.isPendingInclineChange and (st.targetIncline == 6.0)

    st.nowMs = 4000
    pkt2 = build_ftms_packet(3.22, 6.0)
    treadmill_data_callback(st, pkt2)
    arrived = not st.isPendingInclineChange and (st.targetIncline == 6.0)

    if inProgress and arrived:
        print("  -> PASS: Target arrival immediately unlocked pending state at t=4000ms (no arbitrary wait!).")
        passes += 1
    else:
        print(f"  -> FAIL: Closed-loop arrival failed. inProgress={inProgress}, arrived={arrived}")
        failures += 1

    # Test 9: Closed-Loop External Override Detection
    print("\n[TEST 9] Closed-Loop External Physical Incline Override Detection")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    st.nowMs = 1000
    st.currentIncline = 5.0
    st.lastObservedIncline = 5.0
    set_incline(st, 10.0)

    st.nowMs = 2000
    pkt = build_ftms_packet(3.22, 4.0)
    treadmill_data_callback(st, pkt)

    if not st.isPendingInclineChange and st.targetIncline == 4.0:
        print("  -> PASS: External physical incline drop immediately overrode target and synced to 4.0%.")
        passes += 1
    else:
        print(f"  -> FAIL: External override failed. pending={st.isPendingInclineChange}, target={st.targetIncline}")
        failures += 1

    # Test 10: Closed-Loop Incline Stall / Limit Detection
    print("\n[TEST 10] Closed-Loop Incline Stall / Mechanical Limit Detection")
    st = SimulatedState()
    st.currentState = RemoteState.STATE_CONNECTED_RUNNING
    st.nowMs = 1000
    st.currentIncline = 0.0
    set_incline(st, 12.0)

    st.nowMs = 3000
    pkt1 = build_ftms_packet(3.22, 8.0)
    treadmill_data_callback(st, pkt1)

    st.nowMs = 6500
    pkt2 = build_ftms_packet(3.22, 8.0)
    treadmill_data_callback(st, pkt2)

    if not st.isPendingInclineChange and st.targetIncline == 8.0:
        print("  -> PASS: Incline stall detected at 8.0% limit after 3s; target synced cleanly.")
        passes += 1
    else:
        print(f"  -> FAIL: Stall detection failed. pending={st.isPendingInclineChange}, target={st.targetIncline}")
        failures += 1

    print("\n========================================================")
    print(f"SUMMARY: {passes} PASSED, {failures} FAILED")
    print("========================================================")
    return 0 if failures == 0 else 1

if __name__ == "__main__":
    exit(run_tests())
