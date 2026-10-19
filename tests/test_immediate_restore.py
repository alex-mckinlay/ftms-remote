import asyncio
import sys
import time
from bleak import BleakScanner, BleakClient

async def main():
    print("Scanning for Merach/T-25B walking pad...")
    treadmill_device = None
    devices = await BleakScanner.discover(timeout=5.0)
    for d in devices:
        name = d.name or ""
        if any(k in name.lower() for k in ["merach", "t-25", "walking", "treadmill", "mrk"]):
            print(f"Found Treadmill: {d.name} ({d.address})")
            treadmill_device = d
            break
            
    if not treadmill_device:
        print("Treadmill not found.")
        return

    print(f"Connecting to {treadmill_device.name} ({treadmill_device.address})...")
    async with BleakClient(treadmill_device.address) as client:
        print("Connected!")
        
        control_point_uuid = "00002ad9-0000-1000-8000-00805f9b34fb"
        treadmill_data_uuid = "00002acd-0000-1000-8000-00805f9b34fb"
        
        # Notification handler
        def notification_handler(sender, data):
            if sender == treadmill_data_uuid:
                # Parse speed
                flags = data[0] | (data[1] << 8)
                offset = 2
                if (flags & 1) == 0: # Speed present
                    speed_raw = data[offset] | (data[offset+1] << 8)
                    print(f"TELEMETRY - Speed: {speed_raw} ({speed_raw/100.0:.2f} km/h)")
            else:
                print(f"CP INDICATION: {data.hex().upper()}")
                
        await client.start_notify(treadmill_data_uuid, notification_handler)
        await client.start_notify(control_point_uuid, notification_handler)
        
        # Request Control
        print("\nSending: Request Control (0x00)...")
        await client.write_gatt_char(control_point_uuid, bytes([0x00]), response=True)
        await asyncio.sleep(1)
        
        # Start Treadmill
        print("Sending: Start (0x07)...")
        await client.write_gatt_char(control_point_uuid, bytes([0x07]), response=True)
        await asyncio.sleep(3)
        
        # Set Speed to 1.0 km/h (raw 100)
        print("Sending: Set Speed to 1.0 km/h (raw 100: 0x02 64 00)...")
        await client.write_gatt_char(control_point_uuid, bytes([0x02, 0x64, 0x00]), response=True)
        await asyncio.sleep(2)
        
        # Send Incline and IMMEDIATELY restore Speed
        print("\n--- Sending Incline 12.0% and IMMEDIATELY restoring Speed ---")
        
        # We can write both sequentially as fast as asyncio allows
        t_start = time.time()
        
        # Write incline
        print("Writing Incline 12.0%...")
        await client.write_gatt_char(control_point_uuid, bytes([0x03, 0x78, 0x00]), response=True)
        
        # Write speed restore immediately after
        print("Writing Speed 1.0 km/h restore...")
        await client.write_gatt_char(control_point_uuid, bytes([0x02, 0x64, 0x00]), response=True)
        
        print(f"Both commands sent in {time.time() - t_start:.3f} seconds.")
        
        # Monitor for 8 seconds
        await asyncio.sleep(8)
        
        # Stop Treadmill
        print("\nSending: Stop (0x08 0x01)...")
        await client.write_gatt_char(control_point_uuid, bytes([0x08, 0x01]), response=True)
        await asyncio.sleep(2)
        
        await client.stop_notify(treadmill_data_uuid)
        await client.stop_notify(control_point_uuid)

if __name__ == "__main__":
    asyncio.run(main())
