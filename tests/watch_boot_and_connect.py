import sys
import time
import serial

def main():
    port = "/dev/cu.usbmodem1101"
    baud = 115200
    print(f"Connecting to remote on {port}...")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print("Connected! Listening for boot and connection logs for 15 seconds...")
        
        # ESP32 usually resets when we open the serial port or toggle DTR/RTS
        # Let's toggle it to guarantee a reset and see boot output
        ser.dtr = False
        ser.rts = False
        time.sleep(0.1)
        ser.dtr = True
        time.sleep(0.1)
        
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
