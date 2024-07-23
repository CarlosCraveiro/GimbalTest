import serial
import time
import threading
import os
import readline
import argparse
from datetime import datetime

# Define valid commands and auto-completion keywords
COMMANDS = ["START LOG", "STOP LOG", "SET_SURFACES", "TURN", "SET TO MAX", "SET TO MIN", "SET TO ZERO", "TEST SURFACES", "HELP", "CLOSE"]
DIRECTIONS = ["LEFT", "RIGHT", "UP", "DOWN"]
PERCENTAGES = [str(x) for x in range(101)]

# Function to check if the command is valid
def is_valid_command(command):
    if command in ["CLOSE", "HELP"]:
        return True
    if any(command.startswith(valid_command) for valid_command in COMMANDS):
        if command.startswith("SET_SURFACES"):
            try:
                parts = command.split(" ")[1].split(",")
                if len(parts) == 4 and all(-1 <= float(part) <= 1 for part in parts):
                    return True
                else:
                    return False
            except:
                return False
        elif command.startswith("TURN"):
            try:
                parts = command.split(" ")
                if len(parts) == 3 and parts[1] in DIRECTIONS:
                    percentage = float(parts[2])
                    if 0 <= percentage <= 100:
                        return True
                    else:
                        return False
            except:
                return False
        elif command in ["SET TO MAX", "SET TO MIN", "SET TO ZERO", "TEST SURFACES"]:
            return True
        return True
    return False

# Function to map percentage to servo angle
def map_percentage_to_servo_angle(percentage):
    return (percentage / 100.0)

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Serial control system for aircraft model")
parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
parser.add_argument("--baud-rate", type=int, default=9600, help="Baud rate (default: 9600)")
args = parser.parse_args()

# Open the serial port
try:
    ser = serial.Serial(args.port, args.baud_rate, timeout=1)
except Exception as e:
    print(f"Error opening serial port: {e}")
    parser.print_help()
    exit(1)

# Generate a timestamped filename
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_filename = f"log_{timestamp}.csv"

# Open a file to log the data
log_file = open(log_filename, 'w')
log_file.write("Timestamp,Pitch,Roll,Yaw,Aileron_Left,Aileron_Right,Elevator,Rudder\n")

# Create an event to signal threads to stop
stop_event = threading.Event()

# Variables to store the last input angles for the servos
last_elevator = 0.0
last_rudder = 0.0
last_aileron_left = 0.0
last_aileron_right = 0.0

# Function to read from serial and log data
def read_from_serial():
    while not stop_event.is_set():
        try:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("INFO"):
                print(f"\r{line}\nEnter command: ", end='')
            if line and line.count(',') == 7:  # Ensure the line has 8 columns
                log_file.write(line + '\n')
                log_file.flush()
        except Exception as e:
            print(f"Error reading from serial: {e}")

# Function to write commands to serial
def write_to_serial():
    global last_elevator, last_rudder, last_aileron_left, last_aileron_right
    while not stop_event.is_set():
        try:
            command = input("Enter command: ")
            if is_valid_command(command):
                if command == "CLOSE":
                    stop_event.set()
                    break
                elif command == "HELP":
                    print_help()
                elif command.startswith("TURN"):
                    parts = command.split(" ")
                    direction = parts[1]
                    percentage = float(parts[2])
                    angle = map_percentage_to_servo_angle(percentage)

                    if direction == "LEFT":
                        last_aileron_left = angle
                        last_aileron_right = -angle
                    elif direction == "RIGHT":
                        last_aileron_left = -angle
                        last_aileron_right = angle
                    elif direction == "UP":
                        last_elevator = angle
                    elif direction == "DOWN":
                        last_elevator = -angle

                    command = f"SET_SURFACES {last_aileron_right},{last_aileron_left},{last_rudder},{last_elevator}"
                    send_command(command)
                elif command == "SET TO MAX":
                    last_aileron_right = 1
                    last_aileron_left = 1
                    last_rudder = 1
                    last_elevator = 1
                    command = f"SET_SURFACES {last_aileron_right},{last_aileron_left},{last_rudder},{last_elevator}"
                    send_command(command)
                elif command == "SET TO MIN":
                    last_aileron_right = -1
                    last_aileron_left = -1
                    last_rudder = -1
                    last_elevator = -1
                    command = f"SET_SURFACES {last_aileron_right},{last_aileron_left},{last_rudder},{last_elevator}"
                    send_command(command)
                elif command == "SET TO ZERO":
                    last_aileron_right = 0
                    last_aileron_left = 0
                    last_rudder = 0
                    last_elevator = 0
                    command = f"SET_SURFACES {last_aileron_right},{last_aileron_left},{last_rudder},{last_elevator}"
                    send_command(command)
                elif command == "TEST SURFACES":
                    last_aileron_right = -1
                    last_aileron_left = -1
                    last_rudder = -1
                    last_elevator = -1
                    command = f"SET_SURFACES {last_aileron_right},{last_aileron_left},{last_rudder},{last_elevator}"
                    send_command(command)
                    time.sleep(1)  # Wait for a second
                    last_aileron_right = 1
                    last_aileron_left = 1
                    last_rudder = 1
                    last_elevator = 1
                    command = f"SET_SURFACES {last_aileron_right},{last_aileron_left},{last_rudder},{last_elevator}"
                    send_command(command)
                else:
                    send_command(command)
            else:
                print("Invalid command. Please try again.")
        except Exception as e:
            print(f"Error writing to serial: {e}")

def send_command(command):
    error_attempts = 0;
    while error_attempts < 8:
        try:
            ser.write((command + '\n').encode('utf-8'))
            print(f"Sent command: {command}")
            time.sleep(0.5)
            response = ser.readline().decode('utf-8').strip()
            if response != "RESEND":
                break
        except Exception as e:
            print(f"Error sending command: {e}")
            time.sleep(1)  # Wait for a second before retrying
            error_attempts = error_attempts + 1

def print_help():
    help_text = """
    Valid commands:
    - START LOG: Starts logging data.
    - STOP LOG: Stops logging data.
    - SET_SURFACES <Right Aileron>,<Left Aileron>,<Rudder>,<Elevator>: Adjusts the control surfaces. Values between -1 and 1.
    - TURN <LEFT, RIGHT, UP, DOWN> <percentage>: Turns the aircraft in the specified direction by the specified percentage (0 to 100).
    - SET TO MAX: Sets all control surfaces to 1.
    - SET TO MIN: Sets all control surfaces to -1.
    - SET TO ZERO: Sets all control surfaces to 0.
    - TEST SURFACES: Flexes all control surfaces from -1 to 1.
    - HELP: Shows this help message.
    - CLOSE: Closes the Python application.
    """
    print(help_text)

# Display greeting message
def print_greetings():
    greetings_text = """
    Welcome to the serial control system!
    Use the 'HELP' command to list the available commands.
    """
    print(greetings_text)

# Autocomplete function
def completer(text, state):
    options = [cmd for cmd in COMMANDS if cmd.startswith(text)]
    if len(options) == 1:
        if options[0] == "TURN":
            options = [f"TURN {dir} " for dir in DIRECTIONS]
        elif options[0] == "SET_SURFACES":
            return options[0] + " " if state == 0 else None
    elif len(options) == 0:
        if text.startswith("TURN "):
            subtext = text[5:]
            suboptions = [dir for dir in DIRECTIONS if dir.startswith(subtext)]
            if len(suboptions) == 1:
                return "TURN " + suboptions[0] + " " if state == 0 else None
            if len(suboptions) > 1:
                try:
                    return "TURN " + suboptions[state]
                except IndexError:
                    return None
        elif text.startswith("TURN "):
            parts = text.split(" ")
            if len(parts) == 2 and parts[1] in DIRECTIONS:
                subtext = text[len("TURN ") + len(parts[1]) + 1:]
                suboptions = [pct for pct in PERCENTAGES if pct.startswith(subtext)]
                if len(suboptions) > 0:
                    try:
                        return "TURN " + parts[1] + " " + suboptions[state]
                    except IndexError:
                        return None
    try:
        return options[state]
    except IndexError:
        return None

# Set up the completer
readline.set_completer(completer)
readline.parse_and_bind("tab: complete")

# Start the read and write threads
read_thread = threading.Thread(target=read_from_serial)
write_thread = threading.Thread(target=write_to_serial)
read_thread.start()
write_thread.start()

# Display the greeting message
print_greetings()

# Ensure the threads are terminated properly on exit
try:
    while read_thread.is_alive() and write_thread.is_alive():
        time.sleep(1)
except KeyboardInterrupt:
    print("Terminating...")
    stop_event.set()
finally:
    read_thread.join()
    write_thread.join()
    ser.close()
    log_file.close()
    print("Serial port and log file closed.")

