import sys
import time
import serial

def main():
    port = "/dev/cu.usbmodem1101"
    baud = 115200
    print(f"Connecting to remote on {port}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print("Connected! Reading serial output for 15 seconds. Press Ctrl+C to stop.")
        
        # We can also send a command if needed, but first let's just read
        start_time = time.time()
        while time.time() - start_time < 15:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(line)
            time.sleep(0.01)
            
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
