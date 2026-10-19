import asyncio
import sys
import time
import serial
from bleak import BleakScanner, BleakClient

# ESP32 Reset helper
def hold_esp32_in_reset():
    print("Opening serial port to hold ESP32 in reset...")
    try:
        # Toggling DTR/RTS holds ESP32 in reset
        ser = serial.Serial("/dev/cu.usbmodem1101", 115200)
        ser.dtr = True
        ser.rts = True
        # Keep serial port open in background so it stays in reset
        return ser
    except Exception as e:
        print(f"Could not reset ESP32: {e}")
        return None

async def main():
    ser = None
    print("Scanning for Merach/T-25B walking pad...")
    treadmill_device = None
    
    # Scan for 5 seconds
    devices = await BleakScanner.discover(timeout=5.0)
    for d in devices:
        name = d.name or ""
        if any(k in name.lower() for k in ["merach", "t-25", "walking", "treadmill", "mrk"]):
            print(f"Found Treadmill: {d.name} ({d.address})")
            treadmill_device = d
            break
            
    if not treadmill_device:
        print("Treadmill not found in scan.")
        if ser:
            ser.close()
        return

    print(f"Connecting to {treadmill_device.name} ({treadmill_device.address})...")
    async with BleakClient(treadmill_device.address) as client:
        print("Connected!")
        
        # Discover services
        print("\n--- Services & Characteristics ---")
        for service in client.services:
            print(f"Service: {service.uuid} ({service.description})")
            for char in service.characteristics:
                print(f"  Char: {char.uuid} ({char.description}) - Properties: {char.properties}")
                
        # Let's read the Features and Range characteristics if they exist
        ftms_service_uuid = "00001826-0000-1000-8000-00805f9b34fb"
        control_point_uuid = "00002ad9-0000-1000-8000-00805f9b34fb"
        treadmill_data_uuid = "00002acd-0000-1000-8000-00805f9b34fb"
        
        # Notification handler
        def notification_handler(sender, data):
            print(f"NOTIFICATION [{sender}]: {data.hex().upper()}")
            
        print("\nSubscribing to Treadmill Data notifications...")
        await client.start_notify(treadmill_data_uuid, notification_handler)
        
        # Enable control point indications/notifications
        print("Subscribing to Control Point indications...")
        await client.start_notify(control_point_uuid, notification_handler)
        
        # Request Control
        print("\nSending: Request Control (0x00)...")
        await client.write_gatt_char(control_point_uuid, bytes([0x00]), response=True)
        await asyncio.sleep(1)
        
        # Start Treadmill
        print("Sending: Start (0x07)...")
        await client.write_gatt_char(control_point_uuid, bytes([0x07]), response=True)
        await asyncio.sleep(3)
        
        # Test 1: Incline command only (3 bytes)
        print("\nSending: Set Incline to 12.0% (3 bytes: 0x03 78 00)...")
        # 120 in hex is 78
        await client.write_gatt_char(control_point_uuid, bytes([0x03, 0x78, 0x00]), response=True)
        
        # Monitor for 12 seconds
        start_monitor = time.time()
        while time.time() - start_monitor < 12.0:
            await asyncio.sleep(0.5)
        
        # Stop Treadmill
        print("\nSending: Stop (0x08 0x01)...")
        await client.write_gatt_char(control_point_uuid, bytes([0x08, 0x01]), response=True)
        await asyncio.sleep(2)
        
        await client.stop_notify(treadmill_data_uuid)
        await client.stop_notify(control_point_uuid)

    if ser:
        print("Releasing ESP32 reset...")
        ser.close()

if __name__ == "__main__":
    asyncio.run(main())
