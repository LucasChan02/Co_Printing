import sys
import time
import rospy
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
ik = ik_transform.ArmIK()
 

def stop():

     # 回到开机初始位置
     target = ik.setPitchRanges((0.00, 0.14, 0.15), -120, -180, 0)
     if target:
         servo_data = target[1]
         bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                         (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
if __name__ == '__main__':

     rospy.init_node('kinematics_demo', log_level=rospy.DEBUG)
     rospy.on_shutdown(stop)

     joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
     rospy.sleep(0.2) 
     target = ik.setPitchRanges((0.0, 0.08, 0.15), -120, -180, 0) # 运动学求解
     if target: # 判断是否有解
          servo_data = target[1]
          # 驱动机械臂移动
          bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                          (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
     time.sleep(1.5)
     target = ik.setPitchRanges((0.0, 0.3, 0.15),-120, -180, 0)
     if target:
          servo_data = target[1]
          bus_servo_control.set_servos(joints_pub, 1500, ((1, 200)    , (2, 500), (3, servo_data['servo3']),
                          (4, servo_data['servo4']),(5, servo_data    ['servo5']),(6, servo_data['servo6'])))
     time.sleep(1.5)
     target = ik.setPitchRanges((0.0, 0.05, 0.15),-120, -180, 0)
     if target:
          servo_data = target[1]
          bus_servo_control.set_servos(joints_pub, 1500, ((1, 200)    , (2, 500), (3, servo_data['servo3']),
                          (4, servo_data['servo4']),(5, servo_data    ['servo5']),(6, servo_data['servo6'])))
     time.sleep(1.5)

