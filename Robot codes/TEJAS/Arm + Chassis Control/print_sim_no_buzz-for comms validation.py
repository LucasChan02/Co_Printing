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
                   "Printing" simulation
-------------------------------------------------------------
          
  - Will simulate how the robot would move while printing
  
  - The test "print" will be a square tube
  
  - Robot will "print" by moving the entire chassis in a
    square 
    
  - User can input how high the extruder should go up in 
    each layer
    
--------------------------------------------------------------
  * Ctrl+C to stop the script
--------------------------------------------------------------
''')

ik = ik_transform.ArmIK()

start = True
# Processing before closing
def stop():
    global start

    start = False
    print('Closing...')
    set_velocity.publish(0,0,0)  # Turn off all motors
    # Set initial position
    target = ik.setPitchRanges((0.0, 0.10, 0.2), -90, -180, 0) # Kinematics solution
    if target: # Determine if there is a solution
        servo_data = target[1]
        # Move the arm
        bus_servo_control.set_servos(joints_pub, 1800, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(2)
        
# Success buzzer has 1 long "beep"
# def success_buzz():
#     Board.setBuzzer(1)
#     time.sleep(0.75)
#     Board.setBuzzer(0)
    
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
    
# Arm movement function
def move_arm(x, y, z):
    
    # Determine if all arguments are floats
#     if isinstance(x, float) and isinstance(y, float) and isinstance(z, float):
        
        # Parameters: ((x, y, z), pitch angle, min pitch angle, max pitch angle)
        # Default: ((0.0, 0.12, 0.15), -90, -180, 0) 
        # Kinematics solution
        
    target = ik.setPitchRanges((x, y, z), -90, -180, 0) 
    
    # Determine if there is a solutiton
    if target:
#             success_buzz()
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
def move_default():
    print('Moving to initial position')
    
    # Kinematics solution
    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0) 
    
    # Determine if there is a solution
    if target:
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
#         error_buzz()
        return False

# Will repeat user input for x, y, z if not given a valid input
def get_float_input(prompt):
    while True:
        user_input = input(prompt)
        if input_function(user_input):
            return float(user_input)

# "Printing" function
def print_function(side_length, height, layer_height, speed):
    print('Starting print...')
#     success_buzz()
    num_layers = math.floor(height/layer_height)
    time_per_side = (1000 * side_length)/speed
    time.sleep(2)
    current_height = -0.05
    
    # Printing a square tube in CCW direction
    for i in range(int(num_layers)):
        print('Layer ' + str(i + 1))
        
        set_velocity.publish(speed,90,0)  # North
        rospy.sleep(time_per_side)
        set_velocity.publish(speed,180,0) # West
        rospy.sleep(time_per_side)
        set_velocity.publish(speed,270,0) # South
        rospy.sleep(time_per_side)
        set_velocity.publish(speed,0,0)   # East
        rospy.sleep(time_per_side)
        set_velocity.publish(0,0,0)       # Stop
        
        move_arm(0, 0.20, float(current_height))
        current_height = current_height + layer_height
    
    set_velocity.publish(0,0,0)           # Stops the car
    move_default()                        # Returns arm to default position
    print('Print finished!')
#     success_buzz()
    time.sleep(2)
    
# Starts the function for user inputs and moves the arm
def start_function():
    while True:
        start_input = input('Start program? (y/n): ').strip().lower()
        if start_input in ('y', 'yes'):
            
            # Move the arm foward and lower it to the ground
#             success_buzz()
            move_arm(0, 0.20, -0.05)

            side_length = get_float_input('Please input how long the sides should be (meters): ')
            height = get_float_input('Please input how tall the tube should be (meters): ')
            dz = get_float_input('Please input a layer height (0.01 m - 0.05 m): ')
            printing_speed = get_float_input('Please input a printing speed (mm/s): ')

            # Starts print function
            print_function(side_length, height, dz, printing_speed)
            
            # Asks if the user wants to print again
            repeat_input = input('Print again? (y/n): ').strip().lower()
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
    rospy.init_node('print_sim', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    # Servo release
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2) # Delay
    
    # Set initial position
    target = ik.setPitchRanges((0.0, 0.10, 0.2), -90, -180, 0) # Kinematics solution
    if target: # Determine if there is a solution
        servo_data = target[1]
        # Move the arm
        bus_servo_control.set_servos(joints_pub, 1800, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(2)

start_function()









