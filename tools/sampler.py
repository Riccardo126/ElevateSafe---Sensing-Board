import os
import serial
import struct
import csv
import time
import threading

# --- CONFIGURAZIONE ---
PORTA_SERIALE = '/dev/ttyUSB0'  # Modifica con la tua porta (es. /dev/ttyUSB0 su Linux)
BAUD_RATE = 2000000     # Deve corrispondere a Serial.begin(2000000)
FILE_OUTPUT = ""

# Definizione del formato della struct C++:
# float accelX (4 byte), float accelY (4 byte), float accelZ (4 byte), 
# int doorHall (4 byte), int floorHall (4 byte)
# Formato struct: 'fffii' (3 float, 2 int) -> Totale 20 byte
STRUCT_FORMAT = 'fffii'
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

def print_menu():
    print("\n=== ElevateSafe Data Logger ===")
    print("Commands:")
    print("  's' - Start recording")
    print("  't' - Stop recording")
    print("  'q' - Quit")
    print("===============================")

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
        logging = False
        file_csv = None
        writer = None
        command_queue = []
        
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
                    print(f"\n[REC] Recording started... File: {FILE_OUTPUT}")

                elif cmd == 't' and logging:
                    logging = False
                    file_csv.close()
                    print(f"\n[STOP] Recording saved to {FILE_OUTPUT}")

                elif cmd == 'q':
                    if logging:
                        file_csv.close()
                    raise KeyboardInterrupt

            # Read data if logging and data available
            if logging and ser.in_waiting >= STRUCT_SIZE:
                raw_data = ser.read(STRUCT_SIZE)
                
                try:
                    unpacked = struct.unpack(STRUCT_FORMAT, raw_data)
                    accelX, accelY, accelZ, door, floor = unpacked
                    
                    current_time = time.time()
                    writer.writerow([current_time, accelX, accelY, accelZ, door, floor])
                    
                except struct.error:
                    ser.reset_input_buffer()
            
            time.sleep(0.01)  # Small delay to avoid CPU spinning

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
