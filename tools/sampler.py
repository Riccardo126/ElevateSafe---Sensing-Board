import serial
import struct
import threading
import os
from datetime import datetime
import time

SERIAL_PORT = 'COM3'
BAUD_RATE = 921600
BYTES_PER_SAMPLE = 20
PREAMBLE = b'\xAA\xBB\xCC\xDD'
CSV_DIR = 'tools'

recording = False
file_handle = None
stop_event = threading.Event()
file_lock = threading.Lock()

recording_start_time = None  # ← Nuovo

def unpack_sensor_data(data):
    z, ema_zslow, ema_zfast, ema_hall, cum_vz = struct.unpack('<fffff', data)
    return (z, ema_zslow, ema_zfast, ema_hall, cum_vz)

def input_thread():
    global recording, file_handle, recording_start_time
    while not stop_event.is_set():
        try:
            cmd = input().strip().lower()
            if cmd in ['start', 's']:
                if not recording:
                    fname = os.path.join(CSV_DIR, f"sampling_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv")
                    os.makedirs(CSV_DIR, exist_ok=True)
                    with file_lock:
                        file_handle = open(fname, 'w')
                        file_handle.write("Program Time [s], z, slow z, fast z, hall, v\n")
                        recording = True
                        recording_start_time = time.time()  # ← Salva il tempo di inizio
                    print(f"[INFO] Recording started -> {fname}")
            elif cmd in ['stop', 'x']:
                if recording:
                    with file_lock:
                        recording = False
                        if file_handle:
                            file_handle.close()
                            file_handle = None
                        recording_start_time = None
                    print(f"[INFO] Stopped.")
            elif cmd in ['quit', 'q']:
                stop_event.set()
        except EOFError:
            break

def main():
    global recording, file_handle, recording_start_time
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.flushInput()
        
        try:
            ser.set_buffer_size(rx_size=16384)
        except:
            pass
        
        print(f"[INFO] Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        
        input_th = threading.Thread(target=input_thread, daemon=True)
        input_th.start()
        
        sliding_window = b""
        sample_count = 0
        last_rate_check = time.time()
        
        while not stop_event.is_set():
            char = ser.read(1)
            if not char:
                continue
            
            sliding_window += char
            if len(sliding_window) > 4:
                sliding_window = sliding_window[-4:]
            
            if sliding_window == PREAMBLE:
                block_data = ser.read(BYTES_PER_SAMPLE)
                
                if len(block_data) < BYTES_PER_SAMPLE:
                    print(f"[WARN] Blocco incompleto: {len(block_data)}/{BYTES_PER_SAMPLE}")
                    continue
                
                sample_count += 1
                
                # Diagnostica rate
                now = time.time()
                if now - last_rate_check >= 1.0:
                    measured_hz = sample_count / (now - last_rate_check)
                    print(f"[RATE] {measured_hz:.1f} Hz")
                    sample_count = 0
                    last_rate_check = now
                
                with file_lock:
                    if recording and file_handle and recording_start_time:
                        # ← Usa tempo RELATIVO da inizio registrazione
                        ts = time.time() - recording_start_time
                        data = unpack_sensor_data(block_data)
                        file_handle.write(f"{ts:.6f},{data[0]:.6f},{data[1]:.6f},{data[2]:.6f},{data[3]:.6f},{data[4]:.6f}\n")
                        file_handle.flush()  # forza scrittura immediata
                
                sliding_window = b""
    
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        stop_event.set()
        if 'ser' in locals():
            ser.close()

if __name__ == '__main__':
    main()