#!/usr/bin/python3
# coding=utf8
# Date: 2025/06/12
# Description: This script synchronizes a 3D printer and a ROS-based robotic arm.
# It sends G-code commands to the printer and moves the robot arm to follow the toolpath.

import sys
import time
import rospy
import math
import serial
import re
import os
import subprocess

# ROS-specific imports
from kinematics import ik_transform_rev
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from ranger_file_selection import ranger_file_selection

# --- Configuration Constants ---

# -- Serial Communication 
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 250000

# -- ROS and Robotic Arm Configuration
DEFAULT_ARM_SPEED_M_PER_S = 0.05
INTERPOLATION_TIME_STEP_S = 0.04
Z_OFFSET_M = 0.01
PITCH_CONSTRAINT_MIN_DEG = -180.0
PITCH_CONSTRAINT_MAX_DEG = 0.0
SERVO_1_POS = 200
SERVO_2_POS = 500
PARK_X_M = 0.01
PARK_Y_M = 0.00
PARK_Z_M = 0.04
PARK_PITCH_DEG = -180.0

# -- Logging
LOG_FILE_PATH = "/tmp/printer_serial.log"

# --- Global Variables ---
ik_solver_global = None
publisher_global = None
ser_global = None # Global serial object for access in shutdown hook
log_viewer_process = None # Process for the external terminal window

current_feed_rate_mm_per_min = 300.0
last_position_m = {'x': 0.0, 'y': 0.0, 'z': 0.00, 'pitch': PARK_PITCH_DEG}


def parse_gcode_for_move(command):
    """
    Parses a G-code command (like G1) to extract movement and feed rate values.
    Returns a dictionary with the axis values in meters and feed rate in mm/min.
    """
    global current_feed_rate_mm_per_min
    move = {}
    if 'G1' not in command and 'G0' not in command:
        return None

    matches = re.findall(r'([XYZF])([+-]?\d*\.?\d+)', command.upper())
    for axis, value_str in matches:
        value = float(value_str)
        if axis == 'F':
            current_feed_rate_mm_per_min = value
            move['f'] = value
        else:
            move[axis.lower()] = value / 1000.0
    return move


def move_arm_to_target(target_coords_m, target_pitch_deg, duration_ms):
    """
    Moves the arm to a specific target pose (x, y, z, pitch).
    """
    global publisher_global, ik_solver_global, last_position_m

    if publisher_global is None or ik_solver_global is None:
        rospy.logerr("ROS Publisher not initialized.")
        return False

    ik_solution = ik_solver_global.setPitchRanges(
        target_coords_m,
        target_pitch_deg,
        PITCH_CONSTRAINT_MIN_DEG,
        PITCH_CONSTRAINT_MAX_DEG
    )

    if ik_solution:
        servo_data = ik_solution[1]
        try:
            bus_servo_control.set_servos(publisher_global, duration_ms, (
                (1, int(SERVO_1_POS)), (2, int(SERVO_2_POS)),
                (3, int(servo_data['servo3'])), (4, int(servo_data['servo4'])),
                (5, int(servo_data['servo5'])), (6, int(servo_data['servo6']))
            ))
            last_position_m['x'], last_position_m['y'], last_position_m['z'] = target_coords_m
            last_position_m['pitch'] = target_pitch_deg
            return True
        except Exception as e:
            rospy.logerr(f"Error sending servo commands: {e}")
            return False
    else:
        rospy.logwarn(f"No IK solution for target: {target_coords_m}, pitch: {target_pitch_deg:.2f} deg.")
        return False

def write_to_log(message):
    """Appends a message to the global log file."""
    with open(LOG_FILE_PATH, 'a') as log_file:
        log_file.write(message + '\n')

def stream_gcode_and_move_robot(ser, gcode_filepath):
    """
    Main function to stream G-code, parse commands, and move the robot arm in sync.
    """
    global current_feed_rate_mm_per_min, last_position_m

    print(f"--- Starting G-code stream: {gcode_filepath} ---")
    write_to_log(f"--- Starting G-code stream: {gcode_filepath} ---")

    with open(gcode_filepath, 'r') as f:
        line_count = 0
        for line in f:
            if rospy.is_shutdown():
                print("ROS shutdown detected. Stopping G-code stream.")
                break

            line_count += 1
            command = line.strip()

            if not command or command.startswith(';'):
                continue

            print(f"G-code Line {line_count}: {command}")
            write_to_log(f"> {command}")
            ser.write(command.encode() + b'\n')

            move_data = parse_gcode_for_move(command)

            if move_data:
                start_pos = (last_position_m['x'], last_position_m['y'], last_position_m['z'])
                target_x = move_data.get('x', start_pos[0])
                target_y = move_data.get('y', start_pos[1])
                target_z = move_data.get('z', start_pos[2]) + Z_OFFSET_M
                end_pos_m = (target_x, target_y, target_z)
                print(end_pos_m)

                delta_x = end_pos_m[0] - start_pos[0]
                delta_y = end_pos_m[1] - start_pos[1]
                delta_z = end_pos_m[2] - start_pos[2]
                distance_m = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

                if distance_m > 1e-6:
                    feed_rate_m_per_s = (current_feed_rate_mm_per_min / 60) / 1000.0
                    time_for_segment_s = distance_m / feed_rate_m_per_s
                    num_interpolation_steps = max(1, int(round(time_for_segment_s / INTERPOLATION_TIME_STEP_S)))
                    servo_command_duration_ms = int(INTERPOLATION_TIME_STEP_S * 1000)

                    rospy.loginfo(f"  Arm Move: Dist: {distance_m*1000:.2f}mm, Speed: {feed_rate_m_per_s*1000:.2f}mm/s, Steps: {num_interpolation_steps}")

                    for step in range(1, num_interpolation_steps + 1):
                        if rospy.is_shutdown(): break
                        alpha = float(step) / num_interpolation_steps
                        interp_x = start_pos[0] + alpha * delta_x
                        interp_y = start_pos[1] + alpha * delta_y
                        interp_z = start_pos[2] + alpha * delta_z
                        move_arm_to_target((interp_x, interp_y, interp_z), last_position_m['pitch'], servo_command_duration_ms)
                        time.sleep(INTERPOLATION_TIME_STEP_S)
                else:
                     rospy.loginfo("  Arm Move: No change in position, skipping.")

            while True:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                     print(f"  Printer Response: {response}")
                     write_to_log(f"< {response}")
                if 'ok' in response:
                    break
                if 'error' in response.lower():
                    error_msg = f"!! ERROR reported by printer on line {line_count}: {response}"
                    print(error_msg)
                    write_to_log(error_msg)
                    raise Exception("Printer reported an error.")

    print("--- G-code file streaming complete. ---")
    write_to_log("--- G-code file streaming complete. ---")


def stop_arm_movement():
    """
    Called on ROS shutdown (Ctrl+C). Cleans up resources.
    """
    global log_viewer_process, ser_global
    rospy.loginfo("Shutdown signal received. Cleaning up...")
    
    # Safely turn off heater if serial port is available
    if ser_global and ser_global.is_open:
        rospy.loginfo("Turning off extruder heater (M104 S0)...")
        command = "M104 S0"
        ser_global.write(command.encode() + b'\n')
    
    move_arm_to_target(
        (PARK_X_M, PARK_Y_M, PARK_Z_M),
        PARK_PITCH_DEG,
        1500
    )
    rospy.loginfo("Arm parking sequence initiated.")
    
    if log_viewer_process:
        rospy.loginfo("Terminating log viewer window...")
        log_viewer_process.terminate()

    time.sleep(1.5)
    rospy.loginfo("Cleanup complete. Exiting.")


def main():
    global ik_solver_global, publisher_global, log_viewer_process, ser_global

    rospy.init_node('gcode_robot_sync_node', log_level=rospy.INFO)
    publisher_global = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    ik_solver_global = ik_transform_rev.ArmIK()
    rospy.on_shutdown(stop_arm_movement)
    rospy.sleep(0.5)

    gcode_file_to_print = ranger_file_selection()
    if not gcode_file_to_print:
        print("No valid G-code file selected. Exiting.")
        return

    # --- Setup external log viewer ---
    if os.path.exists(LOG_FILE_PATH):
        os.remove(LOG_FILE_PATH)
    
    view_command = f'sh -c "tail -f {LOG_FILE_PATH}; echo \\"---LOGGING ENDED. Press Enter to close.---\\"; read"'
    log_viewer_process = None
    
    for terminal in ['xfce4-terminal', 'gnome-terminal', 'xterm']:
        try:
            print(f"Attempting to launch log viewer with '{terminal}'...")
            if terminal == 'gnome-terminal':
                args = [terminal, '--title=Printer Serial Log', '--', 'sh', '-c', view_command]
            else: # xfce4-terminal and xterm use -T or -e
                args = [terminal, '-T', 'Printer Serial Log', '-e', view_command]

            log_viewer_process = subprocess.Popen(args)
            print(f"Successfully launched log viewer using '{terminal}'.")
            break
        except FileNotFoundError:
            continue
        except Exception as e:
            print(f"[WARNING] Could not launch '{terminal}': {e}. Trying next...")

    if not log_viewer_process:
        print("[WARNING] Could not open a separate log window.")
        print("Printer responses will still be shown in this terminal.")

    try:
        print(f"Connecting to printer on {SERIAL_PORT} at {BAUDRATE}...")
        ser_global = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
        
        print("Connected. Waking up printer and waiting for ready signal...")
        time.sleep(2) # Wait for printer to initialize after connection
        ser_global.reset_input_buffer()
        
        # Send a blank line to wake up the printer and get an 'ok'
        ser_global.write(b'\n')
        
        while not rospy.is_shutdown():
            # response = ser_global.readline().decode('utf-8', errors='ignore').strip()
            # if response:
            #     print(f"  Printer Init: {response}")
            #     write_to_log(f"< {response}")
            # if "T:" in response:
            #     print("Marlin is ready.")
                break

        rospy.loginfo("Moving arm to initial safe position...")
        move_arm_to_target(
            (last_position_m['x'], last_position_m['y'], last_position_m['z']),
            last_position_m['pitch'],
            2000
        )
        time.sleep(2.5)

        stream_gcode_and_move_robot(ser_global, gcode_file_to_print)

    except FileNotFoundError:
        print(f"[ERROR] G-code could not be found '{gcode_file_to_print}'")
    except serial.SerialException as e:
        print(f"[ERROR] Could not connect to printer on {SERIAL_PORT}. Error: {e}")
    except Exception as e:
        print(f"[ERROR] An unexpected error occurred: {e}")
    finally:
        print("Script finished.")
        if ser_global and ser_global.is_open:
            ser_global.close()
        rospy.signal_shutdown("Program finished normally.")


if __name__ == "__main__":
    if sys.version_info.major != 3:
        print('Please run this program with python3!')
        sys.exit(0)
    main()
