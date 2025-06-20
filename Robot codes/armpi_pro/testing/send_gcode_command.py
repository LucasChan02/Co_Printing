import rospy
import requests
import json
import time

# Define Moonraker URL
MOONRAKER_URL = "http://localhost:7125"  # ROS node runs on the same RPi
# If RPi has a different IP, e.g., "http://192.168.1.100:7125"

def send_gcode_to_klipper(gcode_script):
    """Sends a G-code script to Klipper via Moonraker's HTTP API."""
    url = f"{MOONRAKER_URL}/printer/gcode/script"
    payload = {"script": gcode_script}
    headers = {'Content-Type': 'application/json'} # Important for POST requests
    
    rospy.logdebug(f"Sending G-code to Klipper: '{gcode_script}'")
    try:
        response = requests.post(url, headers=headers, json=payload, timeout=10) # 10-second timeout
        response.raise_for_status()  # Raises an HTTPError for bad responses (4XX or 5XX)
        
        # Log Klipper's response to the G-code (if any)
        # Klipper G-code responses are often sent back through the "gcode response" stream,
        # which is better handled by websockets. HTTP POST usually just confirms receipt.
        # The response.json() here is Moonraker's ack of the API call.
        rospy.logdebug(f"Moonraker API response: {response.json()}")
        return response.json() 
    except requests.exceptions.HTTPError as http_err:
        rospy.logerr(f"HTTP error sending G-code '{gcode_script}': {http_err} - Response: {response.text}")
    except requests.exceptions.ConnectionError as conn_err:
        rospy.logerr(f"Connection error sending G-code '{gcode_script}': {conn_err}")
    except requests.exceptions.Timeout as timeout_err:
        rospy.logerr(f"Timeout sending G-code '{gcode_script}': {timeout_err}")
    except requests.exceptions.RequestException as req_err:
        rospy.logerr(f"Error sending G-code '{gcode_script}': {req_err}")
    return None

# Example usage within a ROS1 node:
# def some_ros_action_callback(goal):
#     # Set hotend temperature to 210C and wait
#     rospy.loginfo("Setting hotend temperature to 210C and waiting...")
#     if not wait_for_temp_klipper(210.0):
#         rospy.logerr("Failed to reach target temperature.")
#         # self.action_server.set_aborted()
#         return
#
#     # Extrude 10mm of filament at 300 mm/min feedrate
#     rospy.loginfo("Extruding 10mm of filament...")
#     send_gcode_to_klipper("G92 E0") # Reset extruder position
#     send_gcode_to_klipper("G1 E10 F300") 
#
#     # Wait for Klipper's move buffer to clear before proceeding with other actions
#     rospy.loginfo("Waiting for Klipper moves to complete (M400)...")
#     send_gcode_to_klipper("M400") 
#     rospy.loginfo("Klipper M400 acknowledged.")
#
#     # Turn off heaters
#     # send_gcode_to_klipper("M104 S0") # Hotend off
