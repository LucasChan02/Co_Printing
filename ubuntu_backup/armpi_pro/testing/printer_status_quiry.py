def get_klipper_status(objects_to_query):
    """
    Queries Klipper status for specified objects and their fields via Moonraker.
    :param objects_to_query: A dictionary where keys are Klipper object names 
                             (e.g., "extruder", "toolhead") and values are lists 
                             of field names or None to get all fields for that object.
                             Example: {"extruder": ["temperature", "target"], "toolhead": None}
    :return: A dictionary containing the status of the queried objects, or None on error.
    """
    if not objects_to_query:
        rospy.logwarn("No objects specified for Klipper status query.")
        return None

    query_strings =
    for obj_name, fields in objects_to_query.items():
        if fields and isinstance(fields, list):
            query_strings.append(f"{obj_name}={','.join(fields)}")
        elif fields is None: # Request all fields for this object
             query_strings.append(f"{obj_name}")
        # else: skip if fields is an empty list or invalid type

    if not query_strings:
        rospy.logwarn("No valid objects/fields for Klipper status query.")
        return None
        
    full_query_string = "&".join(query_strings)
    url = f"{MOONRAKER_URL}/printer/objects/query?{full_query_string}"
    
    rospy.logdebug(f"Querying Klipper status: {url}")
    try:
        response = requests.get(url, timeout=5) # 5-second timeout
        response.raise_for_status()
        
        status_data = response.json().get("result", {}).get("status", {})
        rospy.logdebug(f"Klipper status query successful: {status_data}")
        return status_data
    except requests.exceptions.HTTPError as http_err:
        rospy.logerr(f"HTTP error querying Klipper status: {http_err} - Response: {response.text}")
    except requests.exceptions.ConnectionError as conn_err:
        rospy.logerr(f"Connection error querying Klipper status: {conn_err}")
    except requests.exceptions.Timeout as timeout_err:
        rospy.logerr(f"Timeout querying Klipper status: {timeout_err}")
    except requests.exceptions.RequestException as req_err:
        rospy.logerr(f"Error querying Klipper status: {req_err}")
    return None

# Example helper function to wait for temperature using M109 (blocking on Klipper side)
def wait_for_temp_klipper(target_temp, heater_name="extruder", timeout_ros_s=120):
    """
    Commands Klipper to set heater temperature and wait using M109.
    Provides an additional ROS-side timeout for the M109 command acknowledgment.
    """
    if heater_name == "extruder":
        gcode = f"M109 S{target_temp}"
    elif heater_name == "heater_bed":
        gcode = f"M190 S{target_temp}" # M190 for bed temperature wait
    else:
        rospy.logerr(f"Unknown heater name: {heater_name}")
        return False

    rospy.loginfo(f"Sending '{gcode}' to Klipper and waiting for completion...")
    # send_gcode_to_klipper is a blocking call at the HTTP level, but M109/M190
    # makes Klipper itself wait. The timeout in send_gcode_to_klipper is for the API call.
    # We might need a longer effective timeout for the M109 to complete.
    # A simple way is to assume if the API call returns, Klipper has handled it.
    # For very long heat-up times, a more sophisticated check might be needed if
    # the HTTP connection itself times out.
    
    # This call will block until Moonraker responds. Klipper handles the actual wait.
    # If the heating takes very long, the requests.post might time out.
    # Consider increasing timeout in send_gcode_to_klipper for M109/M190 if needed,
    # or implement a polling mechanism as shown commented out below.
    response = send_gcode_to_klipper(gcode) 

    if response: # If API call was successful
        rospy.loginfo(f"Klipper acknowledged '{gcode}'. Assuming temperature reached/reaching.")
        # To be absolutely sure, one could add a small delay and then query temperature.
        time.sleep(2) # Small delay
        query = {heater_name: ["temperature", "target"]}
        status = get_klipper_status(query)
        if status and heater_name in status:
            current_temp = status[heater_name].get('temperature', 0)
            actual_target = status[heater_name].get('target', 0)
            rospy.loginfo(f"{heater_name.capitalize()} temp after {gcode}: Current={current_temp:.1f}C, Target={actual_target:.1f}C")
            if abs(current_temp - target_temp) < 5: # Check if reasonably close
                 return True
            else:
                 rospy.logwarn(f"Temperature {current_temp:.1f}C still not close to target {target_temp:.1f}C after {gcode}.")
                 return False # Or True if M109 is trusted completely
        return True # M109 acknowledged
    else:
        rospy.logerr(f"Failed to send '{gcode}' to Klipper.")
        return False

# Example Polling (alternative to M109 for ROS-side control of waiting):
# def wait_for_temp_polling(target_temp, heater_name="extruder", timeout_s=120):
#     set_temp_gcode = f"M104 S{target_temp}" if heater_name == "extruder" else f"M140 S{target_temp}"
#     send_gcode_to_klipper(set_temp_gcode)
#     rospy.loginfo(f"Waiting for {heater_name} to reach {target_temp}C (polling)...")
#     start_time = rospy.Time.now()
#     query = {heater_name: ["temperature", "target"]}
#     while (rospy.Time.now() - start_time).to_sec() < timeout_s:
#         status = get_klipper_status(query)
#         if status and heater_name in status:
#             current = status[heater_name].get('temperature', 0.0)
#             actual_target = status[heater_name].get('target', 0.0)
#             rospy.loginfo(f"{heater_name.capitalize()} Temp: {current:.1f}C, Target: {actual_target:.1f}C")
#             if actual_target >= target_temp and abs(current - target_temp) < 2.0: # Tolerance of 2 degrees
#                 rospy.loginfo(f"{heater_name.capitalize()} reached target temperature: {current:.1f}C")
#                 return True
#         rospy.sleep(2.0) # Poll every 2 seconds
#     rospy.logwarn(f"Timeout waiting for {heater_name} to reach {target_temp}C.")
#     return False
