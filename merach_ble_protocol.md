# Merach Treadmill BLE Control Protocol Reference

## Overview
Merach treadmills (such as the T-25B / T25 walking pad) advertise standard FTMS (`0x1826`), but full speed and incline control requires using Merach's proprietary GATT service and compound packet structure.

## BLE GATT Identifiers
- **Vendor Service UUID:** `7905f431-b5ce-4e99-a40f-4b1e122d00d0`
- **Vendor Write Characteristic UUID:** `0xFFF2` (Handle `0x003A`)
- **ATT Write Type:** `ATT_WRITE_REQ` (`response = true`) -- *Do NOT use write-without-response!*

## Compound Protocol Payload Format (7 Bytes)
`[0x02, 0x53, 0x02, Speed_Raw, Incline_Raw, Checksum, 0x03]`

- `Speed_Raw`: Speed in 0.1 mph units (`(uint8_t)(speedMph * 10.0f + 0.5f)`). Range: `0.6 mph` (`0x06`) to `4.0 mph` (`0x28`).
- `Incline_Raw`: Incline in integer percent (`(uint8_t)(inclinePercent + 0.5f)`). Range: `0%` (`0x00`) to `12%` (`0x0C`).
- `Checksum`: XOR combination `0x53 ^ 0x02 ^ Speed_Raw ^ Incline_Raw`.
- `Frame Delimiters`: STX = `0x02`, ETX = `0x03`.

## Periodic Keep-Alive / Heartbeat
- **Heartbeat Frame:** `[0x02, 0x51, 0x51, 0x03]`
- **Frequency:** Sent every 1000ms while running.

## Control Rules
1. **Bypass FTMS Control Point (0x2AD9):** Do not send standard FTMS set speed (`0x02`) or set incline (`0x03`) commands while using vendor commands, as FTMS commands override and halt vendor motor loops.
2. **Explicit Discovery:** Always fetch Characteristic `0xFFF2` under Service `7905f431...` directly to avoid overwriting pointers in generic BLE scan loops.
