
import serial
import time
import threading
import os
import readline
import argparse
from datetime import datetime

# Define valid commands and auto-completion keywords
COMMANDS = ["START LOG", "STOP LOG", "PID ON", "PID OFF", "RESET SETPOINT",
            "SET_SURFACES", "TURN", "SET TO MAX", "SET TO MIN", "SET TO ZERO",
            "TEST SURFACES", "HELP", "CLOSE", "SET_PITCH_POSITION", "SET_PID_GAINS", "TEST"]
DIRECTIONS = ["LEFT", "RIGHT", "UP", "DOWN"]
PERCENTAGES = [str(x) for x in range(101)]

# Function to check if the command is valid
def is_valid_command(command):
    if command in ["CLOSE", "HELP", "START LOG", "STOP LOG", "PID ON", "PID OFF", "RESET SETPOINT"]:
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
        elif command.startswith("SET_PITCH_POSITION"):
            try:
                parts = command.split(" ")
                if len(parts) == 2:
                    float(parts[1])  # Check if it can be converted to a float
                    return True
            except:
                return False
        elif command.startswith("SET_PID_GAINS"):
            try:
                parts = command.split(" ")[1].split(",")
                if len(parts) == 9 and all(float(part) for part in parts):
                    return True
                else:
                    return False
            except:
                return False
        elif command.startswith("TEST"):
            try:
                parts = command.split(" ")
                if len(parts) == 4 and parts[1] in ["ELEVATOR", "RUDDER", "AILERON"]:
                    angle = float(parts[2])
                    duration = int(parts[3])
                    return True
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
parser.add_argument("--baud-rate", type=int, default=57600, help="Baud rate (default: 57600)")
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

# Create a buffer to store log data
log_buffer = []

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
            if line and line.count(',') == 4:  # Ensure the line has 5 columns
                log_buffer.append(line)
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
                    # Write buffer to file
                    with open(log_filename, 'w') as log_file:
                        log_file.write("Timestamp,Pitch,Roll,Elevator,Aileron_Left\n")
                        for entry in log_buffer:
                            log_file.write(entry + '\n')
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
                elif command.startswith("TEST"):
                    parts = command.split(" ")
                    surface = parts[1].upper()
                    angle = float(parts[2])
                    duration = int(parts[3])

                    if surface in ["ELEVATOR", "RUDDER", "AILERON_LEFT", "AILERON_RIGHT"]:
                        send_command(f"TEST {surface} {angle} {duration}")
                        log_data_for_duration(duration)
                    else:
                        print(f"Invalid control surface: {surface}")
                elif command.startswith("SET_PITCH_POSITION"):
                    parts = command.split(" ")
                    pitch_position = float(parts[1])
                    command = f"SET_PITCH_POSITION {pitch_position}"
                    send_command(command)
                elif command.startswith("SET_PID_GAINS"):
                    parts = command.split(" ")[1].split(",")
                    if len(parts) == 9:
                        pid_gains = ",".join(parts)
                        command = f"SET_PID_GAINS {pid_gains}"
                        send_command(command)
                else:
                    send_command(command)
            else:
                print("Invalid command. Please try again.")
        except Exception as e:
            print(f"Error writing to serial: {e}")

def send_command(command):
    error_attempts = 0
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
            error_attempts += 1

def log_data_for_duration(duration_ms):
    start_time = time.time()
    while (time.time() - start_time) * 1000 < duration_ms:
        if stop_event.is_set():
            break
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                log_buffer.append(line)
        except Exception as e:
            print(f"Error reading from serial during logging: {e}")

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
    - TEST <surface> <angle> <duration>: Tests the specified control surface at a given angle for a specified duration (ms).
    - SET_PITCH_POSITION <position>: Sets the pitch position to the specified value.
    - SET_PID_GAINS <P_pitch,I_pitch,D_pitch,P_roll,I_roll,D_roll,P_yaw,I_yaw,D_yaw>: Sets the PID gains for pitch, roll, and yaw.
    - PID ON: Enables PID control.
    - PID OFF: Disables PID control.
    - RESET SETPOINT: Resets the setpoints to the current mean values.
    - HELP: Shows this help message.
    - CLOSE: Closes the Python application and saves the log data.
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
    # Write buffer to file on close
    with open(log_filename, 'w') as log_file:
        log_file.write("Timestamp,Pitch,Roll,Elevator,Aileron_Left\n")
        for entry in log_buffer:
            log_file.write(entry + '\n')
    print("Serial port and log file closed.")

