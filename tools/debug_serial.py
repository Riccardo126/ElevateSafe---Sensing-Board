#!/usr/bin/env python3
"""
Debug script to see raw bytes coming from the device
"""
import serial
import time
import sys

PORTA_SERIALE = '/dev/ttyUSB0'
BAUD_RATE = 2000000

def main():
    print(f"Opening {PORTA_SERIALE} @ {BAUD_RATE} baud...")
    ser = serial.Serial(PORTA_SERIALE, BAUD_RATE, timeout=1)
    time.sleep(1)  # Wait for board startup
    
    print("Reading raw bytes for 10 seconds...")
    print("=" * 60)
    
    start_time = time.time()
    byte_count = 0
    hex_line = ""
    
    try:
        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                byte = ser.read(1)
                byte_count += 1
                hex_val = f"{byte[0]:02X}"
                hex_line += hex_val + " "
                
                # Print 16 bytes per line
                if byte_count % 16 == 0:
                    print(hex_line)
                    hex_line = ""
                    
                    # Check for magic byte
                    if byte[0] == 0xAA:
                        print(f"  ^^^ Found MAGIC BYTE (0xAA) at position {byte_count}")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    
    finally:
        if hex_line:
            print(hex_line)
        print("=" * 60)
        print(f"Total bytes received: {byte_count}")
        
        # Analyze first 100 bytes
        if byte_count > 0:
            ser.reset_input_buffer()
            time.sleep(0.5)
            ser.reset_input_buffer()
            
            print("\nWaiting for more data...")
            time.sleep(1)
            if ser.in_waiting > 0:
                first_bytes = ser.read(min(100, ser.in_waiting))
                print(f"\nFirst {len(first_bytes)} bytes:")
                for i in range(0, len(first_bytes), 16):
                    hex_str = " ".join(f"{b:02X}" for b in first_bytes[i:i+16])
                    ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in first_bytes[i:i+16])
                    print(f"  {hex_str:<48} {ascii_str}")
        
        ser.close()

if __name__ == "__main__":
    main()
