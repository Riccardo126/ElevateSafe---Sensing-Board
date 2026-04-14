import serial
import struct
import time
import sys

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Change to your COM port (e.g., 'COM3' on Windows)
BAUD_RATE = 921600
SAMPLES_PER_BLOCK = 50
BYTES_PER_SAMPLE = 20  # 3 floats (12) + 2 ints (8)
BLOCK_SIZE = SAMPLES_PER_BLOCK * BYTES_PER_SAMPLE  # 1000 bytes
PREAMBLE = b'\xAA\xBB\xCC\xDD' # Sequenza magica

def unpack_sensor_data(data):
    """Estrae i dati da un singolo campione di 20 byte"""
    try:
        # < = Little Endian, fff = 3 float, ii = 2 int
        accel_x, accel_y, accel_z, door_hall, floor_hall = struct.unpack('<fffii', data)
        return {
            'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z,
            'door_hall': door_hall, 'floor_hall': floor_hall
        }
    except struct.error:
        return None

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.flushInput() # Pulisce i vecchi detriti nel buffer
        print(f"[INFO] Connesso a {SERIAL_PORT}. Cerco il preambolo...")

        block_count = 0
        sliding_window = b""
        byte_counter = 0
        last_preamble_pos = None

        while True:
            # 1. Fase di Sincronizzazione: Cerchiamo il preambolo un byte alla volta
            char = ser.read(1)
            if not char:
                continue
            
            byte_counter += 1
            sliding_window += char
            if len(sliding_window) > 4:
                sliding_window = sliding_window[-4:] # Teniamo solo gli ultimi 4 byte
            
            # Debug ogni 1000 byte
            if byte_counter % 1000 == 0:
                print(f"[DEBUG] Ricevuti {byte_counter} byte, ultimi 4: {sliding_window.hex().upper()}")

            if sliding_window == PREAMBLE:
                # 2. Fase di Lettura: Abbiamo trovato il preambolo!
                if last_preamble_pos is None:
                    print(f"[SUCCESS] Preamble trovato @ byte {byte_counter}!")
                else:
                    gap = byte_counter - last_preamble_pos
                    print(f"[SUCCESS] Preamble trovato @ byte {byte_counter}! (gap={gap})")
                last_preamble_pos = byte_counter
                # Leggiamo esattamente un blocco da 1000 byte
                block_data = ser.read(BLOCK_SIZE)
                byte_counter += len(block_data)
                
                if len(block_data) == BLOCK_SIZE:
                    block_count += 1
                    print(f"\n--- Blocco #{block_count} ---")
                    
                    for i in range(SAMPLES_PER_BLOCK):
                        start = i * BYTES_PER_SAMPLE
                        end = start + BYTES_PER_SAMPLE
                        sample_raw = block_data[start:end]
                        
                        s = unpack_sensor_data(sample_raw)
                        
                        if s:
                            # Stampiamo solo il primo e l'ultimo campione per non intasare la console
                            if i == 0 or i == SAMPLES_PER_BLOCK - 1:
                                print(f"  S[{i:02d}]: X:{s['accel_x']:7.3f} Y:{s['accel_y']:7.3f} Z:{s['accel_z']:7.3f} | Door:{s['door_hall']} Floor:{s['floor_hall']}")
                else:
                    print(f"[WARN] Blocco incompleto: {len(block_data)}/{BLOCK_SIZE} byte")
                
                # Resettiamo la finestra per cercare il prossimo preambolo
                sliding_window = b""

    except serial.SerialException as e:
        print(f"[ERROR] Connessione fallita: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Arresto richiesto dall'utente")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("[INFO] Porta seriale chiusa")

if __name__ == '__main__':
    main()