import serial
import struct
import time
import sys
import os
import threading
from datetime import datetime

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 921600
SAMPLES_PER_BLOCK = 50
BYTES_PER_SAMPLE = 20
BLOCK_SIZE = SAMPLES_PER_BLOCK * BYTES_PER_SAMPLE
PREAMBLE = 0xAA
CSV_DIR = 'tools'

# State
recording = False
file_handle = None
block_count = 0
stop_event = threading.Event()

def get_progressive_filename():
    """Generate filename with timestamp and progressive number"""
    # Create data directory if it doesn't exist
    os.makedirs(CSV_DIR, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    counter = 1
    filename = os.path.join(CSV_DIR, f"sampling_{timestamp}.csv")
    
    # Check for existing files with same timestamp
    base_filename = os.path.join(CSV_DIR, f"sampling_{timestamp}")
    while os.path.exists(f"{base_filename}_{counter:02d}.csv"):
        counter += 1
    
    if counter > 1:
        filename = f"{base_filename}_{counter:02d}.csv"
    
    return filename

def unpack_sensor_data(data):
    """Unpack binary SensorData from 20-byte chunk"""
    accel_x, accel_y, accel_z, door_hall, floor_hall = struct.unpack('<fffii', data)
    return {
        'AccelX': accel_x,
        'AccelY': accel_y,
        'AccelZ': accel_z,
        'DoorHall': door_hall,
        'FloorHall': floor_hall
    }

def write_csv_header(f):
    """Write CSV header"""
    f.write("Timestamp,AccelX,AccelY,AccelZ,DoorHall,FloorHall\n")
    f.flush()

def input_thread():
    """Thread for handling user input"""
    global recording, file_handle, block_count
    
    while not stop_event.is_set():
        try:
            cmd = input().strip().lower()
            
            if cmd == 'start' or cmd == 's':
                if not recording:
                    filename = get_progressive_filename()
                    file_handle = open(filename, 'w')
                    write_csv_header(file_handle)
                    recording = True
                    block_count = 0
                    print(f"[INFO] Recording started -> {filename}")
                else:
                    print("[INFO] Already recording")
                    
            elif cmd == 'stop' or cmd == 'x':
                if recording:
                    recording = False
                    if file_handle:
                        file_handle.close()
                        file_handle = None
                    print(f"[INFO] Recording stopped ({block_count} blocks saved)")
                else:
                    print("[INFO] Not recording")
                    
            elif cmd == 'quit' or cmd == 'q':
                print("[INFO] Exiting...")
                stop_event.set()
                break
                
            elif cmd in ['help', 'h', '?']:
                print_help()
                
        except EOFError:
            break
        except KeyboardInterrupt:
            stop_event.set()
            break

def print_help():
    """Print help message"""
    print("""
[Commands]
  start (s)     - Start recording
  stop (x)      - Stop recording
  quit (q)      - Exit program
  help (h)      - Show this help
""")

def main():
    global recording, file_handle, block_count
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[INFO] Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        print_help()
        
        # Start input thread
        input_th = threading.Thread(target=input_thread, daemon=True)
        input_th.start()
        
        block_num = 0
        
        while not stop_event.is_set():
            # Wait for preamble byte
            byte = ser.read(1)
            if not byte:
                continue
            
            if byte[0] == PREAMBLE:
                # Read full block (1000 bytes)
                block_data = ser.read(BLOCK_SIZE)
                
                if len(block_data) == BLOCK_SIZE:
                    block_num += 1
                    
                    if recording:
                        block_count += 1
                        timestamp = datetime.now().timestamp()
                        
                        # Parse and write all 50 samples
                        for sample_idx in range(SAMPLES_PER_BLOCK):
                            offset = sample_idx * BYTES_PER_SAMPLE
                            sample_bytes = block_data[offset:offset + BYTES_PER_SAMPLE]
                            sample = unpack_sensor_data(sample_bytes)
                            
                            file_handle.write(
                                f"{timestamp},"
                                f"{sample['AccelX']:.6f},"
                                f"{sample['AccelY']:.6f},"
                                f"{sample['AccelZ']:.6f},"
                                f"{sample['DoorHall']},"
                                f"{sample['FloorHall']}\n"
                            )
                        
                        file_handle.flush()
                        
                        if block_num % 10 == 0:  # Print every 10 blocks
                            print(f"[Recording] Block {block_num}: {block_count} blocks saved", end='\r')
                    
        # Cleanup
        if recording and file_handle:
            file_handle.close()
            
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