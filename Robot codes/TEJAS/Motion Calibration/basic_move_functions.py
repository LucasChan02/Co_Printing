# 07/19/24
# This script contains various functions for basic movement of the chassis
# When this script is ran, the robot should move by using the functions
# and then return to its original position once ran successfully.

import sys 
import rospy
import time
import chassis_control
from chassis_control.msg import * 

print('''

----------------------------------------------

 * Press Ctrl+C to close this program.
   If it fails, please try multiple times!

----------------------------------------------

''')


start = True

#Processes system before Closing
def stop():
    global start

    start = False
    print('Closing...')
    set_velocity.publish(0,0,0) #Stops all motors
    
# Stops all motors and pauses for 2 seconds
def pause():
    set_velocity.publish(0,0,0)
    rospy.sleep(2)

### Basic chassis movement functions

# move_function
# Arguments:
#    speed: Linear speed (mm/s?)
#    angle: Directional angle of travel (degrees, 90 is north, 180 east, etc.)
#    yaw: Yaw speed of car ( + is CC/left turn | - is CCW/right turn)
#    running_time: How long the car will move for

def move_function(speed, angle, yaw, running_time):
    set_velocity.publish(speed, angle, yaw)
    rospy.sleep(running_time)
    pause()

# --------------------------------------------------

# Turn functions
# 2 yaw/2.2 sec = 180 deg/2.2 sec 
# If 180 degree turn is 2.2, 90 degrees is 1.1 sec
# 2.2 sec multiplied by a ratio of input/180

def CC_turn(degree):
    move_function(0,90,2,float((degree/180)*2.2))

def CCW_turn(degree):
    move_function(0,90,-2,float((degree/180)*2.2))

# ---------------------------------------------------

if __name__== '__main__' :

    rospy.init_node('ArmPiTest_2', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # This Initialized the ROS node to run with the name 'Armpi...'

    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=2)
    # This calls the chassis_control script in order to send messages to run motors.

print('Starting...')

# Program start delay
time.sleep(2)

# 100 'speed', 90 deg, 0 yaw, 5 sec
move_function(100,90,0,5)

# Spin 180 degrees
CCW_turn(180)

# 100 'speed', 180 deg (Left), 0 yaw, 5 sec
move_function(100,180,0,2)

# Turn left 90 degrees
CC_turn(90)

### Will do the inverse
CCW_turn(90)
move_function(100,0,0,2)
CC_turn(180)
move_function(100,180,0,5)

print('\n')
print('Code run complete')


