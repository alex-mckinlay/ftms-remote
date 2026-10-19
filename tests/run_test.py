import sys
import time
import serial

def main():
    port = "/dev/cu.usbmodem1101"
    baud = 115200
    print(f"Connecting to remote on {port}...")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)  # Wait for serial connection to settle
        
        # Flush input buffer
        ser.reset_input_buffer()
        
        # Helper to read and log incoming lines
        def read_for_duration(duration):
            start = time.time()
            while time.time() - start < duration:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(line)
                time.sleep(0.01)

        print("\n--- Starting Treadmill ---")
        ser.write(b"u\n")
        read_for_duration(3.0)

        print("\n--- Setting Speed to raw 150 (1.5 mph) ---")
        ser.write(b"s150\n")
        read_for_duration(4.0)

        print("\n--- Sending Incline 3.0% using 3-byte command (i30) ---")
        ser.write(b"i30\n")
        read_for_duration(4.0)

        print("\n--- Sending Incline 2.0% using 2-byte command (j20) ---")
        ser.write(b"j20\n")
        read_for_duration(4.0)

        print("\n--- Stopping Treadmill ---")
        ser.write(b"d\n")
        read_for_duration(3.0)

        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
