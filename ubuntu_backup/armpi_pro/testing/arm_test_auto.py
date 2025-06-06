#!/usr/bin/python3
# coding=utf8
# Date:2025/01/24
import sys
import time
import rospy
import csv
import math
import subprocess # Added for ranger integration
import os # Added for ranger integration (temp file handling)

#import signal # Not used in original, kept commented
#import Board as Board # Not used in original, kept commented
from kinematics import ik_transform_rev
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

print('''
----------------------------------------------------------
 * Ranger will be launched to select the CSV file.
   Navigate to your CSV, select it.
 * Create a CSV file with lines like: x,y,z,pitch
   (e.g., 0.04,0.04,0.00,-180)
 * Coordinates are in meters, pitch in degrees.
 * Ctrl+C to stop
----------------------------------------------------------
''')

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# --- Configuration Constants ---
DEFAULT_CSV_FILENAME = 'waypoints.csv' # Fallback CSV filename
MOVEMENT_SPEED_MM_PER_S = 10.0  # Movement speed in mm/s
INTERPOLATION_TIME_STEP_S = 0.04 # Time step for each interpolated move in seconds

# Pitch constraints for the IK solver (degrees)
PITCH_CONSTRAINT_MIN_DEG = -180.0
PITCH_CONSTRAINT_MAX_DEG = 0.0

# Fixed positions for servos 1 and 2
SERVO_1_POS = 200
SERVO_2_POS = 500

# Parking position for shutdown
PARK_X_M = 0.01
PARK_Y_M = 0.00
PARK_Z_M = 0.08
PARK_PITCH_DEG = -150.0
# --- End Configuration Constants ---

ik_solver_global = ik_transform_rev.ArmIK()
publisher_global = None

def select_csv_with_ranger():
    """
    Launches Ranger to allow the user to select a CSV file.
    Returns the path to the selected file, or None if selection fails or is cancelled.
    """
    temp_file_for_ranger_path = "/tmp/chosen_csv_file.txt"
    try:
        # Clear the temp file if it exists, to ensure a fresh selection
        if os.path.exists(temp_file_for_ranger_path):
            os.remove(temp_file_for_ranger_path)

        rospy.loginfo("Launching Ranger for CSV file selection...")
        rospy.loginfo("In Ranger: Navigate to your CSV file, select it (e.g., press 'l' or right arrow), then quit (e.g., press 'q').")
        
        # Run Ranger. It will write the selected file path to temp_file_for_ranger_path
        # The terminal needs to be available for Ranger to run.
        process = subprocess.Popen(['ranger', '--choosefile=' + temp_file_for_ranger_path])
        process.wait() # Wait for Ranger to close

        if process.returncode == 0:
            if os.path.exists(temp_file_for_ranger_path):
                with open(temp_file_for_ranger_path, 'r') as f:
                    selected_file = f.read().strip()
                if selected_file and selected_file.lower().endswith('.csv'):
                    rospy.loginfo(f"Ranger selection successful: {selected_file}")
                    return selected_file
                elif selected_file:
                    rospy.logwarn(f"Ranger selected a non-CSV file: {selected_file}. Please select a .csv file.")
                    return None 
                else:
                    rospy.logwarn("Ranger exited, but no file path was written to the temp file (selection likely cancelled).")
                    return None
            else:
                rospy.logwarn("Ranger exited, but the selection temp file was not found (selection likely cancelled).")
                return None
        else:
            rospy.logwarn(f"Ranger exited with error code {process.returncode}. CSV selection failed.")
            return None
    except FileNotFoundError:
        rospy.logerr("Ranger command not found. Please ensure Ranger is installed and in your PATH.")
        return None
    except Exception as e:
        rospy.logerr(f"An error occurred while trying to use Ranger for file selection: {e}")
        return None
    finally:
        # Clean up the temporary file
        if os.path.exists(temp_file_for_ranger_path):
            try:
                os.remove(temp_file_for_ranger_path)
            except Exception as e_clean:
                rospy.logwarn(f"Could not remove temporary ranger file {temp_file_for_ranger_path}: {e_clean}")


def read_waypoints_from_csv(filename):
    """
    Reads waypoints from a given CSV file.
    """
    waypoints = []
    try:
        with open(filename, 'r', newline='') as csvfile:
            reader = csv.reader(csvfile)
            for i, row in enumerate(reader):
                if not row or row[0].strip().startswith('#'): 
                    continue
                try:
                    if len(row) == 4:
                        point = [float(val) for val in row]
                        waypoints.append(point)
                    else:
                        rospy.logwarn(f"CSV file '{filename}', line {i+1}: Expected 4 values (x,y,z,pitch), got {len(row)}. Skipping: {row}")
                except ValueError as e:
                    rospy.logwarn(f"CSV file '{filename}', line {i+1}: Error converting value to float ({e}). Skipping row: {row}")
        if waypoints:
            rospy.loginfo(f"Successfully read {len(waypoints)} waypoints from '{filename}'.")
        else:
            rospy.logwarn(f"No valid waypoints found in '{filename}'.")
    except FileNotFoundError:
        rospy.logerr(f"CSV file '{filename}' not found.")
        return []
    except Exception as e:
        rospy.logerr(f"An error occurred while reading CSV file '{filename}': {e}")
        return []
    return waypoints

def move_arm_to_target(target_coords_m, target_pitch_deg, duration_ms):
    """
    Moves the arm to a specific target pose (x, y, z, pitch).
    """
    global publisher_global, ik_solver_global, PITCH_CONSTRAINT_MIN_DEG, PITCH_CONSTRAINT_MAX_DEG
    global SERVO_1_POS, SERVO_2_POS

    if publisher_global is None or ik_solver_global is None:
        rospy.logerr("Publisher or IK solver not initialized. Cannot move arm.")
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
            return True
        except Exception as e:
            rospy.logerr(f"Error sending servo commands: {e}")
            return False
    else:
        rospy.logwarn(f"No IK solution found for target: {target_coords_m}, pitch: {target_pitch_deg:.2f} deg.")
        return False

def stop_arm_movement():
    """
    Called on ROS shutdown (Ctrl+C). Moves the arm to a predefined parking position.
    """
    rospy.loginfo("Shutdown signal received. Moving arm to parking position...")
    move_arm_to_target(
        (PARK_X_M, PARK_Y_M, PARK_Z_M),
        PARK_PITCH_DEG,
        1500 
    )
    rospy.loginfo("Arm parking sequence initiated. Exiting.")
    time.sleep(1.5) 

if __name__ == '__main__':
    rospy.init_node('arm_csv_interpolated_movement', log_level=rospy.INFO)
    publisher_global = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.on_shutdown(stop_arm_movement)
    rospy.sleep(0.5)

    # --- Select CSV file ---
    selected_csv_file = select_csv_with_ranger()
    if not selected_csv_file:
        rospy.logwarn(f"Ranger selection failed or was cancelled. Falling back to default: {DEFAULT_CSV_FILENAME}")
        # Check if the default file exists in the current directory
        if not os.path.isabs(DEFAULT_CSV_FILENAME):
            script_dir = os.path.dirname(os.path.realpath(__file__))
            target_csv_file = os.path.join(script_dir, DEFAULT_CSV_FILENAME)
        else:
            target_csv_file = DEFAULT_CSV_FILENAME
        
        if not os.path.exists(target_csv_file):
            rospy.logerr(f"Default CSV file '{target_csv_file}' also not found. Please create it or select one with Ranger. Exiting.")
            sys.exit(1)
    else:
        target_csv_file = selected_csv_file
    
    rospy.loginfo(f"Using CSV file: {target_csv_file}")
    # --- End Select CSV file ---

    waypoints_m_deg = read_waypoints_from_csv(target_csv_file)

    if not waypoints_m_deg:
        rospy.logerr(f"No waypoints loaded from '{target_csv_file}'. Exiting program.")
        sys.exit(1)

    if MOVEMENT_SPEED_MM_PER_S <= 0:
        rospy.logerr("Movement speed must be positive. Exiting.")
        sys.exit(1)
    
    speed_m_per_s = MOVEMENT_SPEED_MM_PER_S / 1000.0
    servo_command_duration_ms = int(INTERPOLATION_TIME_STEP_S * 1000)
    if servo_command_duration_ms <= 0:
        rospy.logwarn("Interpolation time step is too small, servo duration is 0ms. Adjust INTERPOLATION_TIME_STEP_S.")
        servo_command_duration_ms = 50

    rospy.loginfo(f"Starting arm movement with speed: {MOVEMENT_SPEED_MM_PER_S:.2f} mm/s, time step: {INTERPOLATION_TIME_STEP_S:.2f} s.")

    rospy.loginfo("Moving to the first waypoint...")
    current_x_m, current_y_m, current_z_m, current_pitch_deg = waypoints_m_deg[0]
    
    if not move_arm_to_target((current_x_m, current_y_m, current_z_m), current_pitch_deg, 1500):
        rospy.logerr("Failed to move to the first waypoint. Exiting.")
        sys.exit(1)
    rospy.loginfo(f"Reached first waypoint: ({current_x_m:.3f}, {current_y_m:.3f}, {current_z_m:.3f}) m, Pitch: {current_pitch_deg:.2f} deg.")
    time.sleep(2.0) 

    for i in range(len(waypoints_m_deg) - 1):
        if rospy.is_shutdown():
            break
        
        start_wp = waypoints_m_deg[i]
        end_wp = waypoints_m_deg[i+1]

        start_coords_m = start_wp[:3]
        start_pitch_deg = start_wp[3]
        
        end_coords_m = end_wp[:3]
        end_pitch_deg = end_wp[3]

        rospy.loginfo(f"Segment {i+1}: From {start_coords_m} -> To {end_coords_m}")

        delta_x_m = end_coords_m[0] - start_coords_m[0]
        delta_y_m = end_coords_m[1] - start_coords_m[1]
        delta_z_m = end_coords_m[2] - start_coords_m[2]
        delta_pitch_deg = end_pitch_deg - start_pitch_deg

        distance_m = math.sqrt(delta_x_m**2 + delta_y_m**2 + delta_z_m**2)

        if distance_m < 1e-4: 
            if abs(delta_pitch_deg) > 1e-2: 
                rospy.loginfo(f"Segment {i+1}: Only pitch change. Target pitch: {end_pitch_deg:.2f} deg.")
                move_arm_to_target(tuple(end_coords_m), end_pitch_deg, servo_command_duration_ms)
                time.sleep(INTERPOLATION_TIME_STEP_S + 0.5) 
            current_x_m, current_y_m, current_z_m = end_coords_m
            current_pitch_deg = end_pitch_deg
            continue 

        time_for_segment_s = distance_m / speed_m_per_s
        num_interpolation_steps = max(1, int(round(time_for_segment_s / INTERPOLATION_TIME_STEP_S)))
        
        rospy.loginfo(f"Segment {i+1}: Dist: {distance_m:.4f}m, Time: {time_for_segment_s:.2f}s, Steps: {num_interpolation_steps}")

        for step in range(1, num_interpolation_steps + 1):
            if rospy.is_shutdown():
                break
            
            alpha = float(step) / num_interpolation_steps

            interp_x_m = start_coords_m[0] + alpha * delta_x_m
            interp_y_m = start_coords_m[1] + alpha * delta_y_m
            interp_z_m = start_coords_m[2] + alpha * delta_z_m
            interp_pitch_deg = start_pitch_deg + alpha * delta_pitch_deg
            
            rospy.logdebug(f"  Step {step}/{num_interpolation_steps}: Target ({interp_x_m:.4f}, {interp_y_m:.4f}, {interp_z_m:.4f})m, P:{interp_pitch_deg:.2f}deg")

            if move_arm_to_target((interp_x_m, interp_y_m, interp_z_m), interp_pitch_deg, servo_command_duration_ms):
                time.sleep(INTERPOLATION_TIME_STEP_S) 
            else:
                rospy.logwarn(f"  Skipping interpolated step {step} due to IK failure or servo error.")
        
        if rospy.is_shutdown():
            break
        
        if not move_arm_to_target(tuple(end_coords_m), end_pitch_deg, servo_command_duration_ms + 200): 
             rospy.logwarn(f"Segment {i+1}: Failed to precisely reach final waypoint {end_coords_m}, P:{end_pitch_deg:.2f} deg.")
        
        current_x_m, current_y_m, current_z_m = end_coords_m
        current_pitch_deg = end_pitch_deg
        
        rospy.loginfo(f"Segment {i+1} completed. Reached: ({current_x_m:.3f}, {current_y_m:.3f}, {current_z_m:.3f})m, P:{current_pitch_deg:.2f}deg.")
        time.sleep(0.5) 

    if not rospy.is_shutdown():
        rospy.loginfo("All waypoints processed. Program finished.")
    else:
        rospy.loginfo("Program interrupted during waypoint processing.")
