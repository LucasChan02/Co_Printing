import sys
import time
import rospy
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
ik = ik_transform.ArmIK()


def stop():

 
     target = ik.setPitchRanges((0.00, 0.0, 0.0416), -120, -180, 0)
     if target:
         servo_data = target[1]
         bus_servo_control.set_servos(joints_pub, 3500, ((1, 10), (2, 50), (3, servo_data['servo3']),
                         (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
if __name__ == '__main__':

     rospy.init_node('kinematics_demo', log_level=rospy.DEBUG)
     rospy.on_shutdown(stop)

     joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
     rospy.sleep(0.2)
     target = ik.setPitchRanges((0.0, 0.15, 0.0416), -120, -180, 0) # è¿å¨å­¦æ±è§£
     if target: # å¤æ­æ¯å¦æè§£
          servo_data = target[1]
          # é©±å¨æºæ¢°èç§»å¨
          bus_servo_control.set_servos(joints_pub, 3500, ((1,10), (2, 50), (3, servo_data['servo3']),
                          (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
     time.sleep(2.5)
     target = ik.setPitchRanges((0.0, 0.25, 0.0416), -120, -180, 0) # è¿å¨å­¦æ±è§£
     if target: # å¤æ­æ¯å¦æè§£
          servo_data = target[1]
          # é©±å¨æºæ¢°èç§»å¨
          bus_servo_control.set_servos(joints_pub, 3500, ((1,10), (2, 50), (3, servo_data['servo3']),
                          (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
     time.sleep(2.5)
     target = ik.setPitchRanges((0.1, 0.25, 0.0416), -120, -180, 0) # è¿å¨å­¦æ±è§£
     if target: # å¤æ­æ¯å¦æè§£
          servo_data = target[1]
          # é©±å¨æºæ¢°èç§»å¨
          bus_servo_control.set_servos(joints_pub, 3500, ((1,10), (2, 50), (3, servo_data['servo3']),
                          (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
     time.sleep(2.5)
     target = ik.setPitchRanges((0.1, 0.15, 0.0416), -120, -180, 0) # è¿å¨å­¦æ±è§£
     if target: # å¤æ­æ¯å¦æè§£
          servo_data = target[1]
          # é©±å¨æºæ¢°èç§»å¨
          bus_servo_control.set_servos(joints_pub, 3500, ((1,10), (2, 50), (3, servo_data['servo3']),
                          (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
     time.sleep(2.5)
     target = ik.setPitchRanges((0.0, 0.15, 0.0416), -120, -180, 0) # è¿<90>å
     if target: # å<88>¤æ<96>­æ<98>¯å<90>¦æ<9c><89>è§£
          servo_data = target[1]
          # é©±å<8a>¨æ<9c>ºæ¢°è<87><82>ç§»å<8a>¨
          bus_servo_control.set_servos(joints_pub, 3500, ((1,10), (2, 50), (    3, servo_data['servo3']),
                          (4, servo_data['servo4']),(5, servo_data['servo5']    ),(6, servo_data['servo6'])))
     time.sleep(2.5)

