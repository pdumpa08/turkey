# Filename: control_orpheus_servos.py
# This script runs on your computer to send commands to the Hack Club Orpheus Pico.

import serial
import time
import sys
import glob

# Function to find the Pico's serial port
def find_pico_port():
    """Finds the serial port name for the Raspberry Pi Pico (including Orpheus Pico)."""
    print("Searching for Orpheus Pico port...")
    if sys.platform.startswith('win'):
        # Windows ports are COM numbers
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # Linux ports are /dev/ttyACM* or /dev/ttyUSB*
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    elif sys.platform.startswith('darwin'):
        # macOS ports are /dev/tty.usbmodem* or /dev/tty.usbserial*
        ports = glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*')
    else:
        raise EnvironmentError('Unsupported platform')

    for port in ports:
        try:
            # Try to open and immediately close the port to check if it's available
            s = serial.Serial(port)
            s.close()
            print(f"Found potential Orpheus Pico port: {port}")
            # A common identifier for Pico is often 'ttyACM' on Linux, 'usbmodem' on Mac
            # or a specific VID/PID, but checking if it's openable is a good start.
            # For more robust detection, you might check for specific USB Vendor/Product IDs
            # but for now, this broad check is usually sufficient.
            return port
        except (OSError, serial.SerialException):
            # If the port can't be opened, it's either in use or not a valid device
            pass
    return None

# --- Main Script Execution ---
if __name__ == "__main__":
    pico_port = find_pico_port()
    if pico_port is None:
        print("Error: Hack Club Orpheus Pico not found.")
        print("Please ensure it's plugged in and MicroPython firmware is running.")
        sys.exit(1) # Exit the script if Pico isn't found

    try:
        # Establish the serial connection to the Pico
        # Baud rate (115200) must match MicroPython's default
        ser = serial.Serial(pico_port, 115200, timeout=1)
        time.sleep(2) # Give the serial connection a moment to establish
        print(f"Connected to Orpheus Pico on {pico_port}")
    except Exception as e:
        print(f"Failed to connect to Orpheus Pico: {e}")
        sys.exit(1) # Exit if connection fails

    print("\n--- Orpheus Pico Servo Control ---")
    print("Enter commands in the format: servoX,angle (e.g., servo1,90)")
    print("Type 'exit' to quit this program.")

    while True:
        try:
            # Get user input from the computer's keyboard
            command = input("Enter command: ").strip()

            if command.lower() == 'exit':
                print("Sending exit command to Orpheus Pico...")
                ser.write("exit\n".encode()) # Send 'exit' to Pico
                break # Break out of the loop to close connection

            # Send the command to the Pico, followed by a newline character
            # The '\n' is crucial for MicroPython's readline() to detect the end of the command
            ser.write(f"{command}\n".encode())

            # Give the Pico a moment to process and respond
            time.sleep(0.1)

            # Read any response from the Pico's serial output
            while ser.in_waiting: # Check if there's data in the serial buffer
                line = ser.readline().decode().strip()
                if line: # If a line was read and it's not empty
                    print(f"Pico: {line}") # Print Pico's response

        except KeyboardInterrupt:
            # Handle Ctrl+C to gracefully exit
            print("\nExiting program due to KeyboardInterrupt (Ctrl+C).")
            print("Sending exit command to Orpheus Pico...")
            ser.write("exit\n".encode())
            break
        except Exception as e:
            # Catch any other unexpected errors
            print(f"An unexpected error occurred: {e}")
            break

    # Clean up: close the serial connection when done
    ser.close()
    print("Serial connection closed.")
