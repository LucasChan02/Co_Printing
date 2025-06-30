# !/usr/bin/python3
# coding=utf8
# Date:2024/07/12
import sys
import rospy
import signal
import time
import math
# import Board as Board
from chassis_control.msg import *
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
-------------------------------------------------------------
              "Printing" simulation v1.1
-------------------------------------------------------------
          
  - Will simulate how the robot would move while printing
  
  - The test "print" will be a square tube
  
  - Robot will "print" by moving the entire chassis in a
    square 
    
  - User can input how high the extruder should go up in 
    each layer
    
--------------------------------------------------------------
                        Changes
--------------------------------------------------------------

- Added DC fan support for the extruders
    * M2 is for the heatsink fan
    * M3 is for the extruder tip fan

--------------------------------------------------------------
  * Ctrl+C to stop the script
--------------------------------------------------------------
''')

ik = ik_transform.ArmIK()

start = True
fan_start = True

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

# ======================================================================================
# **Any "Board" modules only work if script is ran w/ sudo
# 
# Success buzzer has 1 long "beep"
# def success_buzz():
#     Board.setBuzzer(1)
#     time.sleep(0.75)
#     Board.setBuzzer(0)
#     
# Error buzzer has 3 short "beeps"
# def error_buzz():
#     Board.setBuzzer(1)
#     time.sleep(0.25)
#     Board.setBuzzer(0)
#     time.sleep(0.25)
#     Board.setBuzzer(1)
#     time.sleep(0.25)
#     Board.setBuzzer(0)
#     time.sleep(0.25)
#     Board.setBuzzer(1)
#     time.sleep(0.25)
#     Board.setBuzzer(0)
#
# ======================================================================================

# The following functions are for DC fans @ 5V:

# Starts both fans for the extruder assembly
# def fan_start_DC():
#     Board.setMotor(2,100)
#     Board.setMotor(3,100)
# 
# # Stops both fans for the extruder assembly
# def fan_stop_DC():
#     Board.setMotor(2,0)
#     Board.setMotor(3,0)
    

# ======================================================================================
# Alternatively, we can use PWM headers 6, 7; voltage is based on battery/external power
# Will use DC for simplicity for now, if 5V is not enough (which will probably the the
# case), use PWM code instead:
# 
# def fan_start_PWM():
#     while fan_start == True 
#         Board.setPWMServoPulse(6,1000,100)
#         Board.setPWMServoPulse(7,1000,100)
#         
# def fan_stop_PWM():
#     global fan_start
#     fan_start = False
# 
# ======================================================================================


# Arm movement function
def move_arm(x, y, z):
        
        # Parameters: ((x, y, z), pitch angle, min pitch angle, max pitch angle)
        # Default: ((0.0, 0.12, 0.15), -90, -180, 0) 
        # Kinematics solution
        
    target = ik.setPitchRanges((x, y, z), -90, -180, 0) 
    
    # Determine if there is a solution
    if target:
#         success_buzz()
        servo_data = target[1]

        # Move the arm
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        
        print('Arm movement successful')
        time.sleep(2)
#     else:
#         print('Please input arguments as floats')
#         error_buzz()

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
#         error_buzz()
        return False

# Will repeat user input for x, y, z if not given a valid input
def get_float_input(prompt):
    while True:
        user_input = input(prompt)
        if input_function(user_input):
            return float(user_input)

# "Printing" function
# Placeholder until we are able to parse a .g (G-Code) file from a 3D slicer
# Possibly will use "pygcode" library
def print_function(side_length, height, layer_height, speed):
    
#     success_buzz()
    num_layers = math.celling(height/layer_height)
    time_per_side = (1000 * side_length)/speed
    current_height = -0.05
    time.sleep(2)
    
    print('\n')
    print('num_layers = ' + str(num_layers))
    print('time_per_side = ' + str(time_per_side))
    print('\n')
    
    time.sleep(2)
    print('Starting print...\n')
#     success_buzzer()
    
    # Printing a square tube in CCW direction
    for i in range(int(num_layers)):
        print('Layer ' + str(i + 1))
        print('current_height = ' + str(current_height))
        time.sleep(2)
        
        set_velocity.publish(speed,90,0)  # North
        rospy.sleep(time_per_side)
        set_velocity.publish(speed,180,0) # West
        rospy.sleep(time_per_side)
        set_velocity.publish(speed,270,0) # South
        rospy.sleep(time_per_side)
        set_velocity.publish(speed,0,0)   # East
        rospy.sleep(time_per_side)
        set_velocity.publish(0,0,0)       # Stop
        rospy.sleep(2)
        
        print('Layer ' + str(i + 1) + ' complete\n')
        print('Moving to layer ' + str(i + 2) + '\n')
        move_arm(0, 0.20, float(current_height))
        current_height = current_height + layer_height
        time.sleep(2)
        print('current_height = ' + str(current_height))
    
    set_velocity.publish(0,0,0)           # Stops the car
    move_initial()                        # Returns arm to default position
    print('Print finished!')
#     success_buzz()
    time.sleep(2)
    
# Starts the function for user inputs and moves the arm
def start_function():
    while True:
#         fan_start_DC()
        start_input = input('Start program? (y/n): ').strip().lower()
        if start_input in ('y', 'yes'):
            
            # Move the arm foward and lower it to the ground
#             success_buzz()
            move_arm(0, 0.20, -0.05)

            side_length = get_float_input('Please input how long the sides should be (meters): ')
            height = get_float_input('Please input how tall the tube should be (meters): ')
            dz = get_float_input('Please input a layer height (0.01 m - 0.05 m): ')
            printing_speed = get_float_input('Please input a printing speed (mm/s): ')

            # Calls the print function
            print_function(side_length, height, dz, printing_speed)
            
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
    rospy.init_node('print_sim_no_buzz.py', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    
    # Chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    
    # Servo release
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2) # Delay
    
    # Set initial position
    move_initial()
    
    # Start function
    start_function()
    
    
    
    
    
    


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






