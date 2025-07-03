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
ROBOT_NUMBER = 1

# -- Serial Communication 
# /dev/ttyACM0 for ArmPi_01, /dev/ttyUSB0 for ArmPi_02
if ROBOT_NUMBER == 1:
    # ArmPi_01
    SERIAL_PORT = '/dev/ttyACM0'    
elif ROBOT_NUMBER == 2:
    # ArmPi_02
    SERIAL_PORT = '/dev/ttyUSB0'

BAUDRATE = 250000

# -- ROS and Robotic Arm Configuration
DEFAULT_ARM_SPEED_M_PER_S = 0.012
INTERPOLATION_TIME_STEP_S = 0.1

if ROBOT_NUMBER == 1:
    X_OFFSET_M = 0.025
    Y_OFFSET_M = -0.00
    Z_OFFSET_M = -0.0014
else:
    X_OFFSET_M = 0.025
    Y_OFFSET_M = -0.00
    Z_OFFSET_M = -0.0010

PITCH_CONSTRAINT_MIN_DEG = -180.0
PITCH_CONSTRAINT_MAX_DEG = 0.0
SERVO_1_POS = 200
SERVO_2_POS = 500
PARK_X_M = -0.04
PARK_Y_M = -0.11
PARK_Z_M = 0.09
PARK_PITCH_DEG = -180.0
PARK_SPEED_MM_PER_MIN = 3600.0

# -- Logging
LOG_FILE_PATH = "/tmp/printer_serial.log"

# --- Global Variables ---
ik_solver_global = None
publisher_global = None
ser_global = None # Global serial object for access in shutdown hook
log_viewer_process = None # Process for the external terminal window

current_feed_rate_mm_per_min = 600.0
last_position_m = {'x': 0.0, 'y': 0.0, 'z': 0.00, 'pitch': PARK_PITCH_DEG}

first_printing_instruction = False
is_skipping_layer = False
previous_elapsed_time = 0


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
            continue

        # Z is common
        if axis == 'Z':
            move['z'] = value / 1000.0 + Z_OFFSET_M
            continue

        # X/Y offsets based on robot number
        if ROBOT_NUMBER == 1:
            if axis == 'X':
                move['x'] = value / 1000.0 + X_OFFSET_M
            elif axis == 'Y':
                move['y'] = value / 1000.0 + Y_OFFSET_M

        elif ROBOT_NUMBER == 2:
            if not first_printing_instruction:
                if axis == 'X':
                    move['x'] = value / 1000.0 + X_OFFSET_M
                elif axis == 'Y':
                    move['y'] = value / 1000.0 + Y_OFFSET_M
            else:
                if axis == 'X':
                    move['x'] = -value / 1000.0 + X_OFFSET_M
                elif axis == 'Y':
                    move['y'] = -value / 1000.0 + Y_OFFSET_M

            
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

def assigned_layer(layer_num, robot_num):
    if robot_num == 1:
        return layer_num % 2 != 0
    elif robot_num == 2:
        return layer_num % 2 == 0
    else:
        return False
    
def calculate_move_time(start_pos_m, end_pos_m, speed_mm_per_min):
    """Calculates the time in seconds to move between two points at a given speed."""
    distance_m = math.sqrt(
        (end_pos_m[0] - start_pos_m[0])**2 +
        (end_pos_m[1] - start_pos_m[1])**2 +
        (end_pos_m[2] - start_pos_m[2])**2
    )
    if speed_mm_per_min > 0:
        return distance_m * 1000 / (speed_mm_per_min / 60.0)
    return 0.0

def stream_gcode_and_move_robot(ser, gcode_filepath):
    """
    Main function to stream G-code, parse commands, and move the robot arm in sync.
    """
    global current_feed_rate_mm_per_min, last_position_m, previous_elapsed_time, time_to_park, layer_start_pos

    previous_elapsed_time = - 2.0
    time_to_park = 5.0
    layer_start_pos = (0.0, 0.0, 0.0)

    print(f"[INFO] Starting G-code stream: {gcode_filepath}")
    write_to_log(f"[INFO] Starting G-code stream: {gcode_filepath}")

    with open(gcode_filepath, 'r') as f:
        line_count = iter(f.readlines())
        first_printing_instruction = False
        is_skipping_layer = False

        for line in line_count:
            if rospy.is_shutdown():
                print("[INFO] ROS shutdown detected. Stopping G-code stream.")
                break
            
            command = line.strip()

            if not command:
                continue # Skip empty lines

            if command.startswith(';'):
                # First printing instruction detect
                if not first_printing_instruction and command.startswith(';LAYER_COUNT:'): 
                        try:
                            while not rospy.is_shutdown():
                                user_input = input("--> Proceed ? (Y/n): ").lower().strip()
                                if user_input in ['y', '']: # Default is Yes
                                    print("[INFO] User approved.")
                                    first_printing_instruction= True # Set flag
                                    rospy.sleep(3.0)
                                    break
                                elif user_input == 'n':
                                    print("[INFO] User aborted.")
                                    rospy.signal_shutdown("User aborted.")
                                    return # Exit the function
                                else:
                                    print("Invalid input.")
                        except IndexError:
                            rospy.logwarn(f"Could not parse filename on line {line_count}.")

                if command.startswith(';LAYER:'):
                    try:
                        current_layer = int(command.split(':')[1])

                        if current_layer == 0:
                            rospy.loginfo(f"Layer {current_layer}")
                            rospy.sleep(3.0)

                        if not assigned_layer(current_layer, ROBOT_NUMBER):
                            rospy.loginfo(f"Layer {current_layer}. Waiting.")
                            is_skipping_layer = True
                        
                            # Park the arm
                            start_park_pos = (last_position_m['x'], last_position_m['y'], last_position_m['z'])
                            park_pos = (PARK_X_M, PARK_Y_M, PARK_Z_M)
                            time_to_park = calculate_move_time(start_park_pos, park_pos, PARK_SPEED_MM_PER_MIN)

                            rospy.loginfo(f"Moving to park position. Estimated {time_to_park:.2f}s")
                            move_arm_to_target(park_pos, -180, int(time_to_park * 1000))
                            # time.sleep(time_to_park) # Wait for the physical move to complete

                            # Find the duration of the layer we are skipping
                            layer_duration = 0
                            layer_start_pos = (last_position_m['x'], last_position_m['y'], last_position_m['z'])

                            for skipped_line in line_count: # Continue consuming lines from the file
                             # Check if we've reached the end of the layer
                             if skipped_line.startswith(';TIME_ELAPSED:'):
                                 current_elapsed_time = float(skipped_line.split(':')[1])
                                 layer_duration = current_elapsed_time - previous_elapsed_time
                                 previous_elapsed_time = current_elapsed_time
                                 rospy.loginfo(f"Detected end of skipped layer. Layer duration: {layer_duration:.2f}s")
                                 break # End the search for this layer

                             # If it's not the end, try to parse it for a move command
                             position_update = parse_gcode_for_move(skipped_line)
                             
                             if position_update:
                                 # If an axis isn't in the command, it keeps its previous value.
                                 new_x = position_update.get('x', layer_start_pos[0])
                                 new_y = position_update.get('y', layer_start_pos[1])
                                 new_z = position_update.get('z', layer_start_pos[2])
                                 layer_start_pos = (new_x, new_y, new_z)
                            
                            # wait_time = max(0, layer_duration - time_to_park)
                            wait_time = layer_duration * 1.2 + time_to_park * 3 + 2.0
                            rospy.loginfo(f"Waiting for {wait_time:.2f}s for other robot to finish.")
                            if wait_time > 0:
                                rospy.sleep(wait_time)
                            
                            rospy.loginfo("Wait complete. Ready for next assigned layer.")

                        else: # Layer assigned to this robot
                            move_arm_to_target(layer_start_pos, PARK_PITCH_DEG, int(time_to_park * 1000))
                            rospy.loginfo(f"Layer {current_layer} is for this robot. Proceeding.")
                            is_skipping_layer = False
                            rospy.sleep(time_to_park + 0.2)

                    except (ValueError, IndexError):
                        rospy.logwarn(f"Could not parse layer number from: {command}")

                elif command.startswith(';TIME_ELAPSED:'):
                    current_elapsed_time = float(command.split(':')[1])
                    previous_elapsed_time = current_elapsed_time

                # If skipping, continue to the next line
                if is_skipping_layer:
                    continue

                continue #No further processing for comments

            # If skipping, don't process any further
            if is_skipping_layer:
                continue

            else: # For actual G-code that needs to be streamed

                rospy.loginfo(f"G-code {command}")
                write_to_log(f"> {command}")
                ser.write(command.encode() + b'\n')
                ser.write(b'M400\n')
                ser.write(b'M118 done\n')

                move_data = parse_gcode_for_move(command)

                if move_data:
                    start_pos = (last_position_m['x'], last_position_m['y'], last_position_m['z'])
                    target_x = move_data.get('x', start_pos[0])
                    target_y = move_data.get('y', start_pos[1])
                    target_z = move_data.get('z', start_pos[2])
                    end_pos_m = (target_x, target_y, target_z)
                    # print(end_pos_m)

                    delta_x = end_pos_m[0] - start_pos[0]
                    delta_y = end_pos_m[1] - start_pos[1]
                    delta_z = end_pos_m[2] - start_pos[2]
                    distance_m = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

                    if distance_m > 1e-6:
                        feed_rate_m_per_s = (current_feed_rate_mm_per_min / 60) / 1000.0
                        time_for_segment_s = distance_m / feed_rate_m_per_s
                        num_interpolation_steps = max(1, int(round(time_for_segment_s / INTERPOLATION_TIME_STEP_S)))
                        servo_command_duration_ms = int(INTERPOLATION_TIME_STEP_S * 1000)

                        rospy.loginfo(f"Arm Move: Dist: {distance_m*1000:.2f}mm, Speed: {feed_rate_m_per_s*1000:.2f}mm/s, Steps: {num_interpolation_steps}")

                        for step in range(1, num_interpolation_steps + 1):
                            if rospy.is_shutdown(): break
                            alpha = float(step) / num_interpolation_steps
                            interp_x = start_pos[0] + alpha * delta_x
                            interp_y = start_pos[1] + alpha * delta_y
                            interp_z = start_pos[2] + alpha * delta_z
                            move_arm_to_target((interp_x, interp_y, interp_z), last_position_m['pitch'], servo_command_duration_ms)
                            time.sleep(INTERPOLATION_TIME_STEP_S)
                    else:
                        rospy.loginfo("Arm Move: No change in position, skipping.")

                if move_data:
                    # For G0/G1, we use M400/M118 to wait for the move to physically finish.
                    while not rospy.is_shutdown():
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            if not 'ok' in response:
                                rospy.loginfo(f"Printer Response: {response}")
                            write_to_log(f"< {response}")
                            if 'busy' in response:
                                rospy.sleep(0.2)
                        # M118 prints the message directly, so we check for an exact match.
                        if response == 'done':
                            break
                        if 'error' in response.lower():
                            raise Exception(f"Printer reported an error on line {line_count}")

                if command.strip().startswith(('M109', 'G92')):
                    while not rospy.is_shutdown():
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        rospy.sleep(1.04)
                        if response:
                            if not 'ok' in response:
                                rospy.loginfo(f"Printer Response: {response}")
                            write_to_log(f"< {response}")
                            if 'busy' in response:
                                rospy.sleep(0.2)
                        if response == 'done':
                            break
                        if 'error' in response.lower():
                            raise Exception(f"Printer reported an error on line {line_count}")

                else:
                    # For other commands, we just wait for the standard 'ok'.
                    while not rospy.is_shutdown():
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            if not 'ok' in response:
                                rospy.loginfo(f"Printer Response: {response}")
                            write_to_log(f"< {response}")
                        if 'busy' in response:
                            rospy.sleep(0.2)
                        if 'ok' in response:
                            break
                        if 'error' in response.lower():
                            raise Exception(f"Printer reported an error on line {line_count}")

    print("G-code file streaming complete.")
    write_to_log("--- G-code file streaming complete. ---")


def stop_arm_movement():
    """
    Called on ROS shutdown (Ctrl+C). Cleans up resources.
    """
    global log_viewer_process, ser_global
    rospy.loginfo("Stop signal received.")
    
    # Safely turn off heater if serial port is available
    if ser_global and ser_global.is_open:
        rospy.loginfo("Turning off extruder heater...")
        command = "M104 S0"
        ser_global.write(command.encode() + b'\n')
    
    move_arm_to_target(
        (PARK_X_M, PARK_Y_M, PARK_Z_M),
        PARK_PITCH_DEG,
        1500
    )
    rospy.loginfo("Arm parking sequence initiated.")
    
    if log_viewer_process:
        log_viewer_process.terminate()

    time.sleep(1.5)
    rospy.loginfo("Cleanup complete.")


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


    if os.path.exists(LOG_FILE_PATH):
        os.remove(LOG_FILE_PATH)

    try:
        print(f"Connecting to printer on {SERIAL_PORT} at {BAUDRATE}...")
        ser_global = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
        
        print("Connected. Waking up printer and waiting for ready signal...")
        time.sleep(2) # Wait for printer to initialize after connection
        ser_global.reset_input_buffer()        
        ser_global.write(b'\n')
        
#        while not rospy.is_shutdown():
#            response = ser_global.readline().decode('utf-8', errors='ignore').strip()
#            if 'ok' in response:
#                print("Marlin is ready.")
#                break

        rospy.loginfo("Moving arm to initial safe position...")
        move_arm_to_target(
            (last_position_m['x'], last_position_m['y'], last_position_m['z']),
            last_position_m['pitch'],
            2000
        )
        stop_arm_movement()
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
