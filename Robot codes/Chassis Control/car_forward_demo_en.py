
#!/usr/bin/python3
# coding=utf8
import sys
import rospy
from chassis_control.msg import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
********************功能:小车前进例程********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')


start = True
#Process before Closing
def stop():
    global start

    start = False
    print('Closing...')
    set_velocity.publish(0,0,0)  # Issues chassis control message to stop movement
    
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('car_forward_demo', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    
    while start:
        # Publish chassis control message, linear speed 60, direction angle 90, yaw angular speed 0 (less than 0, clockwise direction)
        set_velocity.publish(60,90,0) # Move forward
        rospy.sleep(1)
        
    set_velocity.publish(0,0,0)  # Issues chassis control message to stop movement
    print('已关闭')
        
