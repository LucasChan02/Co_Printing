import sys 
import rospy
import time
import chassis_control
from chassis_control.msg import * 

#* Press Ctrl+C to close this program. If it fails, please try multiple times!

start = True
#Processes system before Closing

def stop():
    global start

    start = False
    print('Closing...')
    set_velocity.publish(0,0,0) #Stops all motors

if __name__== '__main__' :

    rospy.init_node('ArmPiTest_2', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # This Initialized the ROS node to run with the name 'Armpi...'

    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=2)
    # This calls the chassis_control script in order to send messages to run motors.

print('Starting')
time.sleep(2)

# Forward
set_velocity.publish(100, 90, 0)
rospy.sleep(28)

# Turn 180 Degrees
set_velocity.publish(0, 90, 2)
rospy.sleep(2.7)

# Forward (return)
set_velocity.publish(100, 90, 0)
rospy.sleep(28)

# Turn 180 Degrees
set_velocity.publish(0, 90, 2)
rospy.sleep(2.5)

set_velocity.publish(0,0,0) #Shuts off motors
print('\n')
print('Code fully ran')


