import serial
import time

# --- Configure these values for your printer ---
SERIAL_PORT = '/dev/ttyACM0' 
BAUDRATE = 250000

GCODE_COMMAND = 'G28 X'
# -------------------


print(f"Connecting to printer on {SERIAL_PORT} at {BAUDRATE}...")

try:
    # Connect to the printer
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        print("Connected. Waiting for printer to initialize...")
        # Marlin needs a few seconds to initialize after a serial connection
        time.sleep(2)
        
        # Clear any data that may have been in the buffer
        ser.reset_input_buffer()
        
        print(f"Sending command: '{GCODE_COMMAND}'")
        
        # Send the G-code command. Note the b'' for bytes and \n for newline
        ser.write(GCODE_COMMAND.encode() + b'\n')
        
        print("Waiting for 'ok' response...")
        
        # Wait for the printer's response
        while True:
            response = ser.readline().decode('utf-8').strip()
            if response: # If we got a response, print it
                print(f"  PRINTER> {response}")
            if 'ok' in response:
                print("Command executed successfully.")
                break # Exit the loop once 'ok' is received
            # You could add error checking here as well
            if 'error' in response.lower():
                print("An error was reported by the printer!")
                break

except serial.SerialException as e:
    print(f"Error connecting to printer: {e}")
finally:
    print("Script finished.")
