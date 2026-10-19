import asyncio
from bleak import BleakScanner

async def main():
    print("Scanning for all BLE devices with advertisement data...")
    devices = await BleakScanner.discover(return_adv=True, timeout=5.0)
    found_any = False
    for addr, (device, adv_data) in devices.items():
        name = device.name or adv_data.local_name or ""
        uuids = adv_data.service_uuids or []
        
        is_match = False
        for u in uuids:
            if "1826" in u.lower():
                is_match = True
        if any(k in name.lower() for k in ["merach", "t-25b", "walking", "treadmill"]):
            is_match = True
            
        if is_match or name:
            print(f"Address: {device.address}, Name: '{name}', UUIDs: {uuids}, RSSI: {adv_data.rssi or device.rssi}")
            found_any = True
            
    if not found_any:
        print("No named or FTMS devices found.")

if __name__ == "__main__":
    asyncio.run(main())
