import serial
import struct
import time
import sys

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Change to your COM port (e.g., 'COM3' on Windows)
BAUD_RATE = 921600
SAMPLES_PER_BLOCK = 50
BYTES_PER_SAMPLE = 20  # 3 floats (12) + 2 ints (8) = 20 bytes
BLOCK_SIZE = SAMPLES_PER_BLOCK * BYTES_PER_SAMPLE  # 1000 bytes
PREAMBLE = 0xAA

def unpack_sensor_data(data):
    """Unpack binary SensorData from 20-byte chunk"""
    # Format: 3 floats (accel X,Y,Z) + 2 ints (doorHall, floorHall)
    accel_x, accel_y, accel_z, door_hall, floor_hall = struct.unpack('<fffii', data)
    return {
        'accel_x': accel_x,
        'accel_y': accel_y,
        'accel_z': accel_z,
        'door_hall': door_hall,
        'floor_hall': floor_hall
    }

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[INFO] Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        print(f"[INFO] Waiting for preamble (0x{PREAMBLE:02X})...\n")
        
        block_count = 0
        
        print("[DEBUG] Reading raw bytes for 2 seconds...")
        for _ in range(2000):  # Read for ~2 seconds at 1000Hz
            byte = ser.read(1)
            if byte:
                print(f"  Byte: 0x{byte[0]:02X}", end="  ")
                if byte[0] == PREAMBLE:
                    print("<-- PREAMBLE FOUND!")
                else:
                    print()
        
        while True:
            # Wait for preamble byte
            byte = ser.read(1)
            if not byte:
                continue
            
            if byte[0] == PREAMBLE:
                # Read full block (1000 bytes)
                block_data = ser.read(BLOCK_SIZE)
                
                if len(block_data) == BLOCK_SIZE:
                    block_count += 1
                    print(f"\n[Block #{block_count}] Received {len(block_data)} bytes")
                    print("-" * 80)
                    
                    # Parse all 50 samples
                    for sample_idx in range(SAMPLES_PER_BLOCK):
                        offset = sample_idx * BYTES_PER_SAMPLE
                        sample_bytes = block_data[offset:offset + BYTES_PER_SAMPLE]
                        
                        sample = unpack_sensor_data(sample_bytes)
                        
                        print(f"  Sample {sample_idx:2d}: "
                              f"X={sample['accel_x']:7.3f} g  "
                              f"Y={sample['accel_y']:7.3f} g  "
                              f"Z={sample['accel_z']:7.3f} g  "
                              f"Door={sample['door_hall']}  "
                              f"Floor={sample['floor_hall']}")
                else:
                    print(f"[ERROR] Incomplete block: got {len(block_data)}/{BLOCK_SIZE} bytes")
            
    except serial.SerialException as e:
        print(f"[ERROR] Serial connection failed: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("[INFO] Serial port closed")

if __name__ == '__main__':
    main()