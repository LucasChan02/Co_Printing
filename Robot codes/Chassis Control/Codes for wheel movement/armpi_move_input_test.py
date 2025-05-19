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

def input_function(user_input):
    try:
        float(user_input)
        return True
    except ValueError:
        print('Please input a float\n')
        return False

def get_float_input(prompt):
    while True:
        user_input = input(prompt)
        if input_function(user_input):
            return float(user_input)
    
def start_function():
    while True:
        lin_speed_input = get_float_input('Input a linear speed: ' )
        dir_angle_input = get_float_input('Input a directional angle: ')
        yaw_speed_input = get_float_input('Input a yaw speed: ')
        time_input = get_float_input('Input a running time (sec): ')
        
        print('Starting...\n')
        
        set_velocity.publish(lin_speed_input, dir_angle_input, yaw_speed_input)
        rospy.sleep(time_input)
        
        print('Stopping...\n')
        set_velocity.publish(0,0,0)
        
        repeat_input = input('Run again? (y/n): ').strip().lower()
            if repeat_input not in ('y', 'yes'):
                stop()
                break
    

if __name__== '__main__' :

    rospy.init_node('ArmPiTest_2', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # This Initialized the ROS node to run with the name 'Armpi...'

    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=2)
    # This calls the chassis_control script in order to send messages to run motors.

start_function()

print('\n')
print('Code run complete')


