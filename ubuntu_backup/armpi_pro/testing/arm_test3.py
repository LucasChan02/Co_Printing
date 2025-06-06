#!/usr/bin/python3
# coding=utf8
# Date:2025/01/24
import sys
import time
import rospy
#import signal
#import Board as Board
from kinematics import ik_transform_rev
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

print('''
**********************************************************
****功能:幻尔科技树莓派扩展板，运动学XYZ轴移动例程*****
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Ctrl+C to stop
----------------------------------------------------------
''')

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

ik = ik_transform_rev.ArmIK()

# Processing before closing
def stop():
    # Initial position
    target = ik.setPitchRanges((0.06, 0.0, 0.04), -150, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('arm_test2', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Servo release
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2) # Delay
    
    
    # Parameters: ((x, y, z), pitch angle, min pitch angle, max pitch angle)
    # Default: ((0.0, 0.12, 0.15), -90, -180, 0) 
    
    # Set initial position
    print('Pos_1')
    target = ik.setPitchRanges((0.04, 0.04, 0.00), -180, -180, 0) # Kinematics solution
    if target: # Determine if there is a solutiton
        servo_data = target[1]
        # Move the arm
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    
    time.sleep(1) # Delay for 1 sec
    
#     Board.setBuzzer(1)	# Buzzer on
#     time.sleep(0.25)	# Delay 0.25 sec
#     Board.setBuzzer(0)	# Buzzer off
    
    time.sleep(2) # Wait 2 sec?
    
    
    # move 0.15 m in the x-axis
    print('Pos_2')
    target = ik.setPitchRanges((0.06, 0.04, 0.00), -180, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    
    time.sleep(1)
    
#     Board.setBuzzer(1)
#     time.sleep(0.25)
#     Board.setBuzzer(0)
    
    time.sleep(2)
    
    # Move 0.20 m in the z-axis
    print('Pos_3')
    target = ik.setPitchRanges((0.05, 0.02, 0.00), -180, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    
    time.sleep(1)
    
#     Board.setBuzzer(1)
#     time.sleep(0.25)
#     Board.setBuzzer(0)
    
    time.sleep(2)
    
    # Move somewhere
    print('Pos_4')
    target = ik.setPitchRanges((0.0, -0.0, 0.00), -180, -180, 0)
    print(target)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    
    time.sleep(1)
    
#     Board.setBuzzer(1)
#     time.sleep(0.25)
#     Board.setBuzzer(0)
    
    time.sleep(2)
    
    # Return to initial position
    print('Pos_5')
    target = ik.setPitchRanges((0.0, 0.03, 0.00), -180, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1)
    
#     Board.setBuzzer(1)
#     time.sleep(0.25)
#     Board.setBuzzer(0)
    
    time.sleep(3)
    
    print('Program run complete')
    
   
    
