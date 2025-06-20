import serial
import time
import os
import subprocess

from ranger_file_selection import ranger_file_selection

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 250000

def stream_gcode(ser, gcode_filepath):
    print(f"--- Starting G-code stream: {gcode_filepath} ---")
    with open(gcode_filepath, 'r') as f:
        line_count = 0
        for line in f:
            line_count += 1
            command = line.strip()

            # Skip empty lines and comments
            if not command or command.startswith(';'):
                continue

            print(f"Gcode {line_count}: {command}")
            ser.write(command.encode() + b'\n')

            # Wait for the 'ok' from Marlin before sending the next line
            while True:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if 'ok' in response:
                    break
                if 'error' in response.lower():
                    print(f"!! ERROR reported by printer on line {line_count}: {response}")
                    raise Exception("Printer reported an error.")
    print("--- G-code file streaming complete. ---")


def main():
    
    gcode_file_to_print = ranger_file_selection()

    if not gcode_file_to_print:
        print("No valid G-code file selected. Exiting.")
        return

    try:
        print(f"Connecting to printer on {SERIAL_PORT} at {BAUDRATE}...")
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2) as ser:
            print("Connected. Initializing...")
            time.sleep(2)  # Wait for printer to reset
            ser.reset_input_buffer()
            
            # Call the streaming function with the selected file
            stream_gcode(ser, gcode_file_to_print)

    except FileNotFoundError:
        print(f"[ERROR] G-code could not be found '{gcode_file_to_print}'")
    except serial.SerialException as e:
        print(f"[ERROR] Could not connect to printer on {SERIAL_PORT}. Error: {e}")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
    finally:
        print("Script finished.")


if __name__ == "__main__":
    main()
