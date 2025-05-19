#!/usr/bin/python3
# coding=utf8
import sys
import rospy
from chassis_control.msg import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
Running code
----------------------------------------------------------
Tips:
 * Ctl + c to stop program
----------------------------------------------------------
''')


start = True
#Process before Closing
def stop():
    global start

    start = False
    print('Closing...')
    set_velocity.publish(0,0,0)  # Closes all motors
    
if __name__ == '__main__':
    # Iniitalize node
    rospy.init_node('movetest2', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Mecanum chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    
    while start:
        # Move forward for 2 sec
        set_velocity.publish(60,90,0)
        rospy.sleep(2)
        
        set_velocity.publish(0,90,0.3)
        rospy.sleep(5)
        
        set_velocity.publish(0,90,-0.3)
        rospy.sleep(5)

        set_velocity.publish(-60,90,0) 
        rospy.sleep(2)
        
    set_velocity.publish(0,0,0)  # Close all motors
    print('Stopping')
        
