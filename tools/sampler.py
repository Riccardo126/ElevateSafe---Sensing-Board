import os
import serial
import struct
import csv
import time
import threading
import math

# --- CONFIGURAZIONE ---
PORTA_SERIALE = '/dev/ttyUSB0'  # Modifica con la tua porta (es. /dev/ttyUSB0 su Linux)
BAUD_RATE = 2000000     # Deve corrispondere a Serial.begin(2000000)
FILE_OUTPUT = ""

# Definizione del formato della struct C++:
# BlockHeader: 1 byte magic (0xAA) + 1 byte sequence
# Poi 50x SensorData: float accel[3] (12 bytes) + int doorHall (4) + int floorHall (4) = 20 bytes
# Formato: 'BBfffiifffiifffii...' (header + 50 samples)
MAGIC_BYTE = 0xAA
HEADER_SIZE = 2  # magicByte (1) + blockSeq (1)
STRUCT_FORMAT = 'fffii'  # Per un singolo campione
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)
BLOCK_SIZE = HEADER_SIZE + (50 * STRUCT_SIZE)  # Header + 50 samples

# Limiti di validazione per i dati di accelerazione
ACCEL_MIN = -50.0  # g (max range IMU)
ACCEL_MAX = 50.0   # g

def is_valid_accel(accelX, accelY, accelZ):
    """Valida se i valori di accelerazione sono ragionevoli"""
    # Controlla se sono numeri validi
    if math.isnan(accelX) or math.isnan(accelY) or math.isnan(accelZ):
        return False
    if math.isinf(accelX) or math.isinf(accelY) or math.isinf(accelZ):
        return False
    
    # Controlla intervallo
    if not (ACCEL_MIN <= accelX <= ACCEL_MAX):
        return False
    if not (ACCEL_MIN <= accelY <= ACCEL_MAX):
        return False
    if not (ACCEL_MIN <= accelZ <= ACCEL_MAX):
        return False
    
    return True

def sync_stream(ser):
    """Cerca il magic byte 0xAA per sincronizzarsi"""
    print("[SYNC] Searching for magic byte 0xAA...")
    synced = False
    attempts = 0
    max_attempts = 100000  # Increased timeout
    bytes_read = 0
    first_bytes = []
    byte_histogram = {}
    
    while not synced and attempts < max_attempts:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            bytes_read += 1
            byte_val = byte[0]
            
            # Collect first bytes for diagnosis
            if len(first_bytes) < 100:
                first_bytes.append(byte_val)
            
            # Build histogram
            byte_histogram[byte_val] = byte_histogram.get(byte_val, 0) + 1
            
            if byte_val == MAGIC_BYTE:
                # Trovato il magic byte, leggi il sequence number
                if ser.in_waiting > 0:
                    seq_byte = ser.read(1)
                    print(f"[SYNC] ✓ Found magic byte! Sequence: {seq_byte[0]}")
                    synced = True
                    return True
                else:
                    print("[SYNC] Found 0xAA but sequence byte not ready yet, continuing...")
            elif attempts % 10000 == 0 and bytes_read > 0:
                print(f"[SYNC] Progress: {attempts} attempts, {bytes_read} bytes read")
        else:
            attempts += 1
            time.sleep(0.001)
    
    if not synced:
        print(f"\n[SYNC] ✗ Failed after {attempts} attempts, {bytes_read} bytes read")
        if bytes_read > 0:
            print("[SYNC] Device IS sending data, but no 0xAA found!")
            print(f"\n[DIAG] First 100 bytes (hex): {' '.join(f'{b:02X}' for b in first_bytes)}")
            
            # Check if it's ASCII text
            ascii_text = ''.join(chr(b) if 32 <= b < 127 else '.' for b in first_bytes[:50])
            print(f"[DIAG] First 50 bytes as ASCII: {ascii_text}")
            
            print("\n[DIAG] Byte distribution (top 10):")
            for byte_val, count in sorted(byte_histogram.items(), key=lambda x: -x[1])[:10]:
                if 32 <= byte_val < 127:
                    print(f"  0x{byte_val:02X} ('{chr(byte_val)}'): {count} times")
                else:
                    print(f"  0x{byte_val:02X}: {count} times")
            
            print("\n[DIAG] TROUBLESHOOTING:")
            print("  1. Is DEBUG_MODE set to false in main.cpp?")
            print("  2. Did you recompile after changing DEBUG_MODE?")
            print("  3. Did you restart the device after uploading?")
            print("  4. Try: unplug device for 5 seconds, replug, then retry")
    return synced

def print_menu():
    print("\n=== ElevateSafe Data Logger (v2 - Synchronized) ===")
    print("Commands:")
    print("  's' - Start recording")
    print("  't' - Stop recording")
    print("  'q' - Quit")
    print("===================================================")

def input_thread(command_queue):
    """Thread to read commands from user input"""
    while True:
        try:
            cmd = input("Enter command (s/t/q): ").strip().lower()
            if cmd in ['s', 't', 'q']:
                command_queue.append(cmd)
            else:
                print("Invalid command. Use 's', 't', or 'q'")
        except KeyboardInterrupt:
            command_queue.append('q')
            break

def main():
    print(f"--- ElevateSafe Data Logger ---")
    print(f"Port: {PORTA_SERIALE} @ {BAUD_RATE} baud")
    print_menu()

    try:
        ser = serial.Serial(PORTA_SERIALE, BAUD_RATE, timeout=1)
        print("[INIT] Waiting for device to stabilize...")
        time.sleep(2)  # Wait for board startup
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("[INIT] Serial port ready, device baud: {} Hz".format(BAUD_RATE))
        
        # First, sync the stream
        if not sync_stream(ser):
            print("[ERROR] Could not synchronize with device")
            print("[HINT] Check: 1) Device is powered and connected")
            print("               2) Port {} exists and is readable".format(PORTA_SERIALE))
            print("               3) DEBUG_MODE=false was uploaded to device")
            print("               4) Device was restarted after upload")
            return
        
        logging = False
        file_csv = None
        writer = None
        command_queue = []
        valid_samples = 0
        invalid_samples = 0
        
        # Start input thread
        input_th = threading.Thread(target=input_thread, args=(command_queue,), daemon=True)
        input_th.start()

        while True:
            # Process commands from queue
            while command_queue:
                cmd = command_queue.pop(0)
                
                if cmd == 's' and not logging:
                    FILE_OUTPUT = "tools/sampling_{}.csv".format(time.strftime("%Y-%m-%d_%H-%M-%S"))
                    file_csv = open(FILE_OUTPUT, mode='w', newline='')
                    writer = csv.writer(file_csv)
                    os.chmod(FILE_OUTPUT, 0o666)
                    writer.writerow(['Timestamp', 'AccelX', 'AccelY', 'AccelZ', 'DoorHall', 'FloorHall'])
                    logging = True
                    valid_samples = 0
                    invalid_samples = 0
                    print(f"\n[REC] Recording started... File: {FILE_OUTPUT}")

                elif cmd == 't' and logging:
                    logging = False
                    file_csv.close()
                    print(f"\n[STOP] Recording saved to {FILE_OUTPUT}")
                    print(f"[STATS] Valid samples: {valid_samples}, Invalid: {invalid_samples}")

                elif cmd == 'q':
                    if logging:
                        file_csv.close()
                    raise KeyboardInterrupt

            # Read full blocks (header + 50 samples)
            if logging and ser.in_waiting >= BLOCK_SIZE:
                # Read header (2 bytes: magic + seq)
                header_data = ser.read(HEADER_SIZE)
                
                # Verify magic byte
                if header_data[0] != MAGIC_BYTE:
                    invalid_samples += 1
                    print(f"[ERROR] Lost sync! Expected 0xAA, got 0x{header_data[0]:02X}")
                    sync_stream(ser)
                    continue
                
                block_seq = header_data[1]
                
                # Read 50 samples (1000 bytes)
                block_data = ser.read(50 * STRUCT_SIZE)
                
                if len(block_data) < 50 * STRUCT_SIZE:
                    invalid_samples += 1
                    print(f"[ERROR] Block incomplete: got {len(block_data)} bytes, expected {50*STRUCT_SIZE}")
                    continue
                
                # Process each sample in the block
                for i in range(50):
                    offset = i * STRUCT_SIZE
                    try:
                        unpacked = struct.unpack(STRUCT_FORMAT, block_data[offset:offset+STRUCT_SIZE])
                        accelX, accelY, accelZ, door, floor = unpacked
                        
                        # Validate data
                        if is_valid_accel(accelX, accelY, accelZ):
                            current_time = time.time()
                            writer.writerow([current_time, accelX, accelY, accelZ, door, floor])
                            file_csv.flush()  # Flush after each sample
                            valid_samples += 1
                        else:
                            invalid_samples += 1
                    except struct.error as e:
                        invalid_samples += 1
                        print(f"[ERROR] Struct unpack failed at sample {i} in block {block_seq}: {e}")
                        break
            
            time.sleep(0.001)  # Small delay to avoid CPU spinning

    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\n\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser.is_open:
            ser.close()
        print("Done.")

if __name__ == "__main__":
    main()
