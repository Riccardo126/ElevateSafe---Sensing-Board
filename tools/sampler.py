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
# PREAMBLE LUNGO: Fondamentale per evitare i falsi positivi
PREAMBLE = b'\xAA\xBB\xCC\xDD' 
CSV_DIR = 'tools'

# --- STATE ---
recording = False
file_handle = None
block_count = 0
stop_event = threading.Event()
file_lock = threading.Lock() # Per evitare crash tra thread

def get_progressive_filename():
    os.makedirs(CSV_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return os.path.join(CSV_DIR, f"sampling_{timestamp}.csv")

def unpack_sensor_data(data):
    # <fffii = Little Endian: 3 float, 2 int
    accel_x, accel_y, accel_z, door_hall, floor_hall = struct.unpack('<fffii', data)
    return (accel_x, accel_y, accel_z, door_hall, floor_hall)

def input_thread():
    global recording, file_handle, block_count
    while not stop_event.is_set():
        try:
            cmd = input().strip().lower()
            if cmd in ['start', 's']:
                if not recording:
                    fname = get_progressive_filename()
                    with file_lock:
                        file_handle = open(fname, 'w')
                        file_handle.write("Timestamp,AccelX,AccelY,AccelZ,DoorHall,FloorHall\n")
                        recording = True
                        block_count = 0
                    print(f"[INFO] Recording started -> {fname}")
            elif cmd in ['stop', 'x']:
                if recording:
                    with file_lock:
                        recording = False
                        if file_handle:
                            file_handle.close()
                            file_handle = None
                    print(f"[INFO] Stopped. {block_count} blocks saved.")
            elif cmd in ['quit', 'q']:
                stop_event.set()
        except EOFError: break

def main():
    global recording, file_handle, block_count
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        ser.flushInput()
        print(f"[DEBUG] Serial port timeout set to 0.5s")
        
        input_th = threading.Thread(target=input_thread, daemon=True)
        input_th.start()
        
        #print(f"[INFO] Connected to {SERIAL_PORT}. Cerco PREAMBLE {PREAMBLE.hex().upper()}...")
        
        sliding_window = b""
        byte_counter = 0
        preamble_found_count = 0
        last_preamble_pos = None
        
        while not stop_event.is_set():
            # 1. SINCRONIZZAZIONE (Sliding Window)
            char = ser.read(1)
            if not char: continue
            
            byte_counter += 1
            sliding_window += char
            if len(sliding_window) > 4:
                sliding_window = sliding_window[-4:]
            
            # Debug: stampa ogni 1000 byte se preamble non trovato
            if byte_counter % 1000 == 0:
                print(f"[DEBUG] Ricevuti {byte_counter} byte, ultimi 4: {sliding_window.hex().upper()} (cerco {PREAMBLE.hex().upper()})")
            
            if sliding_window == PREAMBLE:
                preamble_found_count += 1
                if last_preamble_pos is None:
                    print(f"[SUCCESS] Preamble trovato! #{preamble_found_count} @ byte {byte_counter}")
                else:
                    gap = byte_counter - last_preamble_pos
                    #print(f"[SUCCESS] Preamble trovato! #{preamble_found_count} @ byte {byte_counter} (gap={gap})")
                last_preamble_pos = byte_counter
                # 2. LETTURA BLOCCO
                block_data = ser.read(BLOCK_SIZE)
                byte_counter += len(block_data)
                if len(block_data) < BLOCK_SIZE:
                    print(f"[WARN] Blocco incompleto: letti {len(block_data)}/{BLOCK_SIZE} byte")
                    continue
                
                # 3. SCRITTURA
                with file_lock:
                    if recording and file_handle:
                        block_count += 1
                        ts = datetime.now().timestamp()
                        
                        csv_lines = []
                        for i in range(SAMPLES_PER_BLOCK):
                            offset = i * BYTES_PER_SAMPLE
                            sample_bytes = block_data[offset:offset + BYTES_PER_SAMPLE]
                            
                            # Unpack veloce
                            data = unpack_sensor_data(sample_bytes)
                            csv_lines.append(f"{ts},{data[0]:.6f},{data[1]:.6f},{data[2]:.6f},{data[3]},{data[4]}")
                        
                        file_handle.write("\n".join(csv_lines) + "\n")
                        # file_handle.flush() # Opzionale, rallenta un po' ma è più sicuro

                sliding_window = b"" # Reset per il prossimo blocco

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        stop_event.set()
        if 'ser' in locals(): ser.close()

if __name__ == '__main__':
    main()