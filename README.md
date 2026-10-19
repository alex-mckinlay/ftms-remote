# M5Stack CoreS3 SE Bluetooth Remote for Merach T-25B Walking Pad

This repository contains the firmware, protocol documentation, and test suite to program an **M5Stack CoreS3 SE** IoT Controller (without battery) as a dedicated Bluetooth Low Energy (BLE) desk remote for the **Merach T-25B** walking pad/treadmill.

While the Merach T-25B advertises standard Bluetooth SIG **Fitness Machine Service (FTMS `0x1826`)** for real-time telemetry (`0x2ACD`), reliable speed and incline control requires communicating over Merach's proprietary vendor GATT service (`0xFFF2`) alongside a closed-loop telemetry state machine.

---

## Key Features
- **Hybrid BLE Protocol Engine:**
  - **Merach Proprietary Control (`0xFFF2`):** Uses Merach's 7-byte XOR-checksummed compound control frames and 1 Hz heartbeat packets for simultaneous speed (`0.6–4.0 MPH`) and incline (`0–12%`) control, with automatic fallback to standard FTMS Control Point (`0x2AD9`).
  - **Standard FTMS Telemetry (`0x2ACD`):** Reads live belt speed, incline percentage, elapsed time, and distance directly from the walking pad.
- **Closed-Loop Telemetry & Motor Ramping:**
  - Tracks live incline and speed transitions without arbitrary blind lockout timers.
  - Automatically detects target arrival, mechanical stalls/limits, and external physical remote overrides to stay in sync with the treadmill at all times.
- **Tactile Card UI (320x240 Double-Buffered):**
  - **Status Header:** Displays connection indicator, elapsed workout time (`MM:SS`), distance (`mi`), and running state pill (`STOPPED`, `RUNNING`, `PAUSED`).
  - **Speed & Incline Cards:** High-contrast green Speed card (`MPH`) and orange Incline card (`%`) with progress bars and circular `-` / `+` stepper buttons.
  - **Press-and-Hold Steppers:** Holding `-` or `+` rapidly ticks the target value on screen with tactile audio ticks and transmits the final setpoint over BLE immediately upon release to prevent BLE buffer saturation.
  - **Action Capsules:** Full-width **Start** button when stopped, transitioning to split **Pause/Resume** and **Stop** capsules while active.
- **Audible Feedback & Watchdog Protection:**
  - Distinct speaker tones via the internal AW88298 amplifier for touch confirmation, hold-ticks, BLE connect/disconnect, and errors.
  - 12-second telemetry watchdog automatically disconnects and triggers auto-reconnection if the walking pad powers off or goes out of range.
- **Battery-less Continuous Operation:** Power-optimized for continuous USB-C desk power on the M5Stack CoreS3 SE.

---

## Protocol & Architecture Documentation
- [**Merach BLE Control Protocol Reference (`merach_ble_protocol.md`)**](merach_ble_protocol.md) — Documents the proprietary service UUID, `0xFFF2` characteristic payload structure, XOR checksum calculation, and 1000 ms keep-alive frame.
- [**Embedded BLE Control & Closed-Loop Telemetry Guide (`ble_closed_loop_control.md`)**](ble_closed_loop_control.md) — Details asynchronous GATT characteristic discovery and the closed-loop incline/speed ramping state machine.

---

## Repository Structure
- `src/main.cpp` — Main ESP32-S3 / M5Unified firmware implementation.
- `platformio.ini` — PlatformIO build configuration for `m5stack-cores3`.
- `merach_ble_protocol.md` — Reverse-engineered Merach BLE protocol specification.
- `ble_closed_loop_control.md` — Closed-loop actuator state machine reference.
- `tests/` — Host-side unit tests (`test_r1_r2_logic.py`, `test_r1_r2_simulation.cpp`) and Python BLE/serial diagnostic utilities (`treadmill_direct.py`, `scan_ftms.py`, `interact_remote.py`).

---

## Flashing Instructions (macOS / Linux / Windows)

### 1. Prerequisites
1. Download and install [Visual Studio Code](https://code.visualstudio.com/).
2. Install the **PlatformIO IDE** extension from the VS Code Extensions Marketplace.

### 2. Prepare the Device (Download Mode)
The M5Stack CoreS3 SE uses the ESP32-S3 chip with native USB CDC serial communication. If the serial port is not automatically detected during upload, place the board into **Download Mode**:
1. Connect the M5Stack CoreS3 SE to your computer via a USB-C data cable.
2. Locate the red Power/Reset button on the side of the device.
3. **Press and hold the red button for 2–3 seconds** until the internal green status LED illuminates.
4. Release the button. The device is now in Bootloader/Download Mode.

### 3. Compile and Upload
1. Open the cloned repository directory in VS Code.
2. Wait for PlatformIO to automatically fetch the `espressif32` platform, `M5Unified`, and `M5GFX` libraries.
3. Click the **PlatformIO Project Tasks** icon in the left sidebar and select **m5stack-cores3** -> **General** -> **Upload** (or click the **Upload** arrow in the bottom status bar).
4. Once the upload completes, press the red Reset button once to boot into the firmware.

---

## Operating Instructions

### 1. Powering On & Connecting
- Connect the M5Stack CoreS3 SE to a USB-C power source and switch on the **Merach T-25B** walking pad.
- Ensure the walking pad is disconnected from the Merach mobile app on your phone (BLE peripherals only accept one central connection at a time).
- On boot, the controller scans for FTMS / Merach walking pads. If a single walking pad is detected, it auto-connects after 10 seconds, or you can tap the device row on screen to connect immediately.

### 2. Using the Dashboard
- **When Stopped (`STOPPED`):**
  - Tap or hold **Speed `-` / `+`** (`0.6–4.0 MPH` in `0.1 MPH` increments) to pre-select your starting speed.
  - Tap or hold **Incline `-` / `+`** (`0.0–12.0%` in `1.0%` increments) to adjust incline even while the belt is stationary.
  - Tap the green **Play (`▶`)** capsule at the bottom to start the belt.
- **When Running (`RUNNING` / `PAUSED`):**
  - Tap or press-and-hold **Speed `-` / `+`** or **Incline `-` / `+`** to adjust workout intensity in real time.
  - Tap the orange **Pause (`❚❚`)** capsule to pause the belt while preserving workout stats, or tap the green **Play (`▶`)** capsule to resume.
  - Tap the red **Stop (`◼`)** capsule to bring the belt to a complete stop and reset target speed to `0.6 MPH`.

### 3. Running the State Machine Unit Tests
You can run the standalone closed-loop state machine verification suites on your computer without hardware attached:
```bash
python3 tests/test_r1_r2_logic.py
```
Or compile and run the C++ simulation harness:
```bash
g++ -std=c++17 tests/test_r1_r2_simulation.cpp -o /tmp/test_sim && /tmp/test_sim
```

---

## License
Distributed under the [MIT License](LICENSE).

