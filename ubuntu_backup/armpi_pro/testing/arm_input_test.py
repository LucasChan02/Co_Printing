# !/usr/bin/python3
# coding=utf8
# Date:2024/07/12

import sys
import time
import rospy
import signal
import Board as Board
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

print('''
-------------------------------------------------------------
          Arm movement test with user input
-------------------------------------------------------------
          
  - Defined several functions to move the arm
  
  - Takes in float values to move arm to desired location
  
  - Added buzzer control to give audio feedback of current
    status of robot (need to run as root user)
    
  - Should be able to detect if the user runs the script
    as root user
    
--------------------------------------------------------------
  * Ctrl+C to stop the script
--------------------------------------------------------------
''')

if sys.version_info.major == 2:
    print('Please run this program with python3.x!')
    sys.exit(0)

ik = ik_transform.ArmIK()

# Processing before stopping
def stop():
    
    # Initial position
    target = ik.setPitchRanges((0.00, 0.12, 0.08), -145, -180, 0)
    if target:
        success_buzz()
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(2)

# Success buzzer has 1 long "beep"
def success_buzz():
    Board.setBuzzer(1)
    time.sleep(0.75)
    Board.setBuzzer(0)
    
# Error buzzer has 3 short "beeps"
def error_buzz():
    Board.setBuzzer(1)
    time.sleep(0.25)
    Board.setBuzzer(0)
    time.sleep(0.25)
    Board.setBuzzer(1)
    time.sleep(0.25)
    Board.setBuzzer(0)
    time.sleep(0.25)
    Board.setBuzzer(1)
    time.sleep(0.25)
    Board.setBuzzer(0)
    
# Arm movement function
def move_arm(x, y, z):
    
    # Determine if all arguments are floats
    if isinstance(x, float) and isinstance(y, float) and isinstance(z, float):
        
        # Parameters: ((x, y, z), pitch angle, min pitch angle, max pitch angle)
        # Default: ((0.0, 0.12, 0.15), -90, -180, 0) 
        # Kinematics solution
        
        target = ik.setPitchRanges((x, y, z), -90, -180, 0) 
        
        # Determine if there is a solutiton
        if target:
            success_buzz()
            servo_data = target[1]
            
            # Move the arm
            bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
            
            print('Arm movement successful')
            time.sleep(2)
    else:
        print('Please input arguments as floats')
        error_buzz()

# Move to initial position
def move_default():
    print('Moving to initial position')
    
    # Kinematics solution
    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0) 
    
    # Determine if there is a solution
    if target:
        success_buzz()
        servo_data = target[1]
        # Move the arm
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    
# Determines if the user's input is a float, returns a boolean
def input_function(user_input):
    try:
        float(user_input)
        return True
    except ValueError:
        print('Please input a float')
        error_buzz()
        return False

# Will repeat user input for x, y, z if not given a valid input
def get_float_input(prompt):
    while True:
        user_input = input(prompt)
        if input_function(user_input):
            return float(user_input)

# Starts the function for user inputs and moves the arm
def start_function():
    while True:
        start_input = input('Start program? (y/n): ').strip().lower()
        if start_input in ('y', 'yes'):
            
            # x, y, z user inputs
            x_input = get_float_input('Please input an x-value: ')
            y_input = get_float_input('Please input a y-value: ')
            z_input = get_float_input('Please input a z-value: ')
            
            # Moves the arm based on valid user inputs
            print('Starting...')
            move_arm(x_input, y_input, z_input)
            
            # Asks if the user wants to move the arm to another location
            repeat_input = input('Run again? (y/n): ').strip().lower()
            if repeat_input not in ('y', 'yes'):
                move_default()
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
    rospy.init_node('arm_test2', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Servo release
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2) # Delay
    
start_function()
       
        
