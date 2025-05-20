#!/usr/bin/env python3
# encoding:utf-8
# 2020/01/25 aiden
import time
import numpy as np
from math import sqrt
if __name__ == "__main__":
    import inverse_kinematics
else:
    from . import inverse_kinematics

#机械臂根据逆运动学算出的角度进行移动
ik = inverse_kinematics.IK()

class ArmIK:
    servo3Range = (0, 1000, 0, 240.0) #脉宽， 角度
    servo4Range = (0, 1000, 0, 240.0)
    servo5Range = (0, 1000, 0, 240.0)
    servo6Range = (0, 1000, 0, 240.0)
    end_effector_link_length = 0.111
    end_effector_x_offset = 0.016
    end_effector_length = np.sqrt (end_effector_link_length ** 2 + end_effector_x_offset ** 2)


    forearm_link_length = 0.094
    upper_arm_length = 0.101
    
    shouder_height = 0.065
    robot_arm_platform_height = 0.092


    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        # 适配不同的舵机
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range

        self.servo3halfrange = (self.servo3Range[3] - self.servo3Range[2])/2
        self.servo4halfrange = (self.servo4Range[3] - self.servo4Range[2])/2
        self.servo5halfrange = (self.servo5Range[3] - self.servo5Range[2])/2
        self.servo6halfrange = (self.servo6Range[3] - self.servo6Range[2])/2

        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        #将逆运动学算出的角度转换为舵机对应的脉宽值
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        servo4 = int(round(-theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 + theta5 * self.servo5Param))
        servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + theta6)) * self.servo6Param)
        
        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def inverse_kinematics(self, coordinate_data, alpha):
        x, y, z = coordinate_data
        pitch_angle = np.deg2rad(alpha)

        theta6 = np.rad2deg(np.arctan(y / x)) # Hip angle

        l_x = np.sqrt(x**2 + y**2)   # distance in vertical plane
        l_z = z + self.shouder_height    # height in vertical plane

        # angle of end effector equivalent line to verticle line
        gamma = (np.pi + pitch_angle - np.arctan(end_effector_x_offset / end_effector_link_length))  

        l_cg = end_effector_length * np.cos(gamma) - l_z
        l_eg = l_x - end_effector_x_offset * np.sin(gamma)
        beta = np.arctan(l_cg / l_eg)

        l_ce = np.sqrt(l_cg**2 + l_eg**2)
        l_cd = forearm_link_length
        l_de = upper_arm_length

        angle_d = np.arccos((l_cd**2 + l_de**2 - l_ce**2) / (2 * l_cd * l_de))
        theta4 = - np.rad2deg(angle_d - np.pi)    # elbow angle

        angle_e = np.arccos((l_de**2 + l_ce**2 - l_cd**2) / (2 * l_de * l_ce))
        theta5 = np.rad2deg( -(np.pi/2 -angle_e - beta))  # shoulder angle

        angle_c = np.pi - angle_d - angle_e
        theta3 = np.rad2deg( -(np.pi - angle_c - (np.pi/2 - beta) - (np.pi - pitch_angle)))  # wirst angle

        is_theta3_valid = -self.servo3halfrange <= theta3 <= self.servo3halfrange
        is_theta4_valid = -self.servo4halfrange <= theta4 <= self.servo4halfrange
        is_theta5_valid = -self.servo5halfrange <= theta5 <= self.servo5halfrange
        is_theta6_valid = -self.servo6halfrange <= theta6 <= self.servo6halfrange

        if is_theta3_valid and is_theta4_valid and is_theta5_valid and is_theta6_valid:
            new_angles = {}
            new_angles['theta3'] = theta3
            new_angles['theta4'] = theta4
            new_angles['theta5'] = theta5
            new_angles['theta6'] = theta6
            return new_angles
        else:
            # debug message
            if not is_theta3_valid: print(f"Theta3 {theta3} is out of range {self.servo3Range}")
            if not is_theta4_valid: print(f"Theta4 {theta4} is out of range {self.servo4Range}")
            # ... etc.
            return False

    
    def setPitchRanges(self, coordinate_data, alpha, alpha1, alpha2, d = 0.01):
        #给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解
        #如果无解返回False,否则返回舵机角度、俯仰角
        #坐标单位m， 以元组形式传入，例如(0, 0.5, 0.1)
        #alpha为给定俯仰角, 单位度
        #alpha1和alpha2为俯仰角的取值范围
        x, y, z = coordinate_data
        # a_range = abs(int(abs(alpha1 - alpha2)/d)) + 1
        # for i in range(a_range):
        #     if i % 2:
        #         alpha_ = alpha + (i + 1)/2*d
        #     else:                
        #         alpha_ = alpha - i/2*d
        #         if alpha_ < alpha1:
        #             alpha_ = alpha2 - i/2*d
        #     result = ik.getRotationAngle((x, y, z), alpha_)
        #     if result:
        #         theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
        #         servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
        #         return result, servos, alpha_

        result = self.inverse_kinematics(coordinate_data, alpha)
        if result:
            theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
            servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
            return result, servos, alpha_
        
        return False

if __name__ == "__main__":
    import rospy
    import armpi_fpv.bus_servo_control as bus_servo_control
    from hiwonder_servo_msgs.msg import MultiRawIdPosDur
    
    # 初始化节点
    rospy.init_node('ik_test', log_level=rospy.DEBUG)
    # 舵机发布
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2)
    
    AK = ArmIK()
    print(ik.getLinkLength())
    target = AK.setPitchRanges((0.0, 0.12, 0.08), -145, -180, 0)
    
    if target:
        print(target)
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1000, ((1, 450), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
