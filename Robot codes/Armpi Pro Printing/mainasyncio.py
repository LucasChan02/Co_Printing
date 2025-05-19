# !/usr/bin/python3
# coding=utf8
# Date:2024/08/05

import sys
import rospy
import signal
import os
import time
import math
import asyncio
##import Board as Board
##import boardFunctions
import GCodetoCSV
from gParse import *
from chassis_control.msg import *
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
-------------------------------------------------------------
                    Real Printing
-------------------------------------------------------------
          
  - Will take real G-Code and move the arm to desired 
    locations

--------------------------------------------------------------
  * Ctrl+C to stop the script
--------------------------------------------------------------
''')

ik = ik_transform.ArmIK()

# Global Variables
start = True
fan_start = True
z_off = 0

# Processing before closing
def stop():
    global start

    start = False
    print('\n')
    print('Closing...')
    set_velocity.publish(0,0,0)  # Turn off all motors
    # Set default position
    target = ik.setPitchRanges((0.00, 0.12, 0.08), -145, -180, 0) # Kinematics solution
    if target: # Determine if there is a solution
        servo_data = target[1]
        # Move the arm
        bus_servo_control.set_servos(joints_pub, 1800, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(2)

# Arm movement function
async def move_arm(x, y, z, move_time):
        
    # Parameters: ((x, y, z), pitch angle, min pitch angle, max pitch angle)
    # Default: ((0.0, 0.12, 0.15), -90, -180, 0) 
    # Kinematics solution
        
    target = ik.setPitchRanges((x, y, z), -90, -180, 0) 
    
    # Determine if there is a solution
    if target:
        # success_buzz()
        servo_data = target[1]

        # Move the arm
        bus_servo_control.set_servos(joints_pub, move_time, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        
        print('Arm movement successful\n')
    else:
        print(f'\nUnable to find a solution for:\nx = {x}\ny = {y}\nz = {z}\n')
        # error_buzz()

# Returns the Euclidean distance (straight-line distance) between two points of the same plane
def dist(x1,x2,y1,y2):
    distance = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
    return distance

# Sets the z-coordinate offset for the extruder
async def set_z_offset():
    global z_off
    offset = get_float_input('Please input a z offset: ')
    # bus_servo_control.set_servos(joints_pub, 2000, ((1, 200), (2, 500), (3, 215), (4, 600),(5, 150),(6, 500)))
    await move_arm(0, .2, offset, 1500)
    check = input('Change z offset again? (y/n): ').strip().lower()
    if check not in ('n', 'no'):
        await set_z_offset()
    z_off = offset

# Returns the time in ms that the arm needs to run based on inputed coordinates + desired speed
def speed_to_time(x1, x2, y1, y2, speed):
    rate = (speed/1000)            # mm/s to meter/s
    print('Rate: ' + '{:.2f}'.format(speed) + ' mm/s ; ' + '{:.5f}'.format(rate) + ' m/s')
    
    distance = dist(x1, x2, y1, y2)     # Find direct distance by using hypotenuse function
    print('Distance: ' + '{:.2f}'.format(distance) + ' mm')
    
    final_time = (distance/rate)        # distance = rate * time -> time = distance/rate
    print('Final time: ' + '{:.2f}'.format(final_time) + ' ms\n')
    
    return final_time

# Move to initial position
def move_initial():
    print('Moving to initial position')
    # Should lower to the ground
    bus_servo_control.set_servos(joints_pub, 2000, ((1, 200), (2, 500), (3, 215), (4, 600),(5, 150),(6, 500)))

# Determines if the user's input is a float, returns a boolean
def input_function(user_input):
    try:
        float(user_input)
        return True
    except ValueError:
        print('Please input a float')
        # error_buzz()
        return False

# Will repeat user input for x, y, z if not given a valid input
def get_float_input(prompt):
    while True:
        user_input = input(prompt)
        if input_function(user_input):
            return float(user_input)

# "Printing" function
async def print_function(speed):
##    await asyncio.sleep(2)

    print('Starting print...\n')
    # success_buzzer()
    for i in range(len(xVal) - 1):
        # Ensures that i is an integer
        i = int(i)
        trueX = (xVal[i])/1000 - .1
        trueY = (yVal[i])/1000 + .1
        trueZ = zVal[i]/10 + z_off
        try:
            print(f'------------------------ Move #{i} ---------------------\n')
            
            # Calculate time to move for the current xVal[i]
            current_time = speed_to_time(xVal[i], xVal[i+1], yVal[i], yVal[i+1], speed) 
            
            # Move the arm to the coordinates of the current xVal[i]
            await move_arm(trueX, trueY, trueZ, current_time)  # Await the asynchronous move_arm function
            
            # Log the current coordinates the arm is at
            print(f'Moving to\n x: {xVal[i]/1000:.5f}\n y: {yVal[i]/1000:.5f}\n z: {zVal[i]/1000 + z_off:.5f}\n')
            print('\n***** True Coordinate Values *****\n')
            print(f'Moving to\n x: {trueX:.5f}\n y: {trueY:.5f}\n z: {trueZ:.5f}\n')

        except KeyboardInterrupt:
            print('Force closing...')
            stop()
            break

    set_velocity.publish(0,0,0)           # Stops the car if it is moving (car should not be moving in this script)
    target = ik.setPitchRanges((0.00, 0.12, 0.08), -145, -180, 0) # Kinematics solution
    
    # Return to default position
    if target: # Determine if there is a solution
        servo_data = target[1]
        # Move the arm
        bus_servo_control.set_servos(joints_pub, 1800, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(2)
        
    print('Print finished!')
    # success_buzz()
    await asyncio.sleep(2)

def desired_file():
    path = input('Please input a path to a directory: ')
    user_choice = str(input('Confirm "' + str(path) + '" is the chosen directory?').strip().lower())
    if user_choice not in ('y', 'yes'):
        desired_file()

    # /home/ubuntu/armpi_pro/testing/Printing/ should be the default directory if we want to hard code the directory       
    
    file = input('Please specify a desired G-Code file: ')
    if os.path.isfile(path + file):
        user_choice = str(input('Confirm "' + str(file) + '" is the chosen file?').strip().lower())
        if user_choice not in ('y', 'yes'):
            desired_file()
        return file
    else:
        print('Not a valid file in directory "' + str(path) + '"\n')
        desired_file()

# Starts the function for user inputs and moves the arm
async def start_function():
    while True:
        # fan_start_DC()
        start_input = input('Start program? (y/n): ').strip().lower()
        if start_input in ('y', 'yes'):
            # Clear terminal
            os.system('cls')

            # Runs the file "GCodetoCSV.py" to convert inputed G Code into a CSV 
            exec(open('GCodetoCSV.py').read())

            # Move the arm foward and lower it to the ground
            # success_buzz()
            
            await set_z_offset()
            printing_speed = get_float_input('Please input a printing speed (mm/s): ')

            # Calls the print function
            await print_function(printing_speed)
            
            # Asks if the user wants to print again
            repeat_input = input('Print again? (y/n): ').strip().lower()
            if repeat_input not in ('y', 'yes'):
                move_initial()
                stop()
                break
            
        elif start_input in ('n', 'no'):
            print('Ending program')
            stop()
            break
        
        else:
            print('Invalid input. Please enter "y" or "n".')
    
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('main.py', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    
    # Chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    
    # Servo release
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2) # Delay
    
    # Set initial position
    move_initial()
    
    # Clear terminal
    os.system('cls')
    
    # Start function ----- Python 3.7 or less:
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(start_function())
    finally:
        loop.close()

    # Python 3.8 or greater:
    # asyncio.run(start_function())
    
    
    
    
    
    


# Old functions, do not use

# def move_initial():
#     print('Moving to initial position')
#
#     # Kinematics solution
#      target = ik.setPitchRanges((0.00, 0.12, 0.08), -145, -180, 0) 
#   
#    # Determine if there is a solution
#     if target:
#         servo_data = target[1]
#         # Move the arm
#         # bus_servo_control.set_servos(joints_pub, time,((servoID, position), ...))
#         bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
#                         (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))







