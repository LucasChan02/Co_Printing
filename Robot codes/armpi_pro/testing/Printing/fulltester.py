 #!/usr/bin/python3
 # coding=utf8
 # Date:2022/06/30
import sys
import time
import rospy
from kinematics import ik_transform
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
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

ik = ik_transform.ArmIK()

#关闭前处理
def stop():
    # 回到开机初始位置
    target = ik.setPitchRanges((0.00, 0.0, 0.06), -145, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 20), (2, 50), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))


if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('kinematics_demo', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # 舵机发布
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2) # 延时等生效
      # 设置初始位置
    target = ik.setPitchRanges((0.0, 0.0, 0.05), -120, -180, 0) # 运动学求解
    if target: # 判断是否有解
        servo_data = target[1]
        # 驱动机械臂移动
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 20), (2, 50), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1.5)
    target = ik.setPitchRanges((0.0, 0., 0.1), -120, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 20), (2, 50), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1.5)
    target = ik.setPitchRanges((0.0, 0.0, 0.15), -120, -180, 0)
    if target:
         servo_data = target[1]
         bus_servo_control.set_servos(joints_pub, 1500, ((1, 20), (2, 50), (3, servo_data['servo3']),
                         (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1.5)
    target = ik.setPitchRanges((0.0, 0.0, 0.2), -120, -180, 0)
    if target:
         servo_data = target[1]
         bus_servo_control.set_servos(joints_pub, 1500, ((1, 20), (2, 50), (3, servo_data['servo3']),
                         (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1.5)

