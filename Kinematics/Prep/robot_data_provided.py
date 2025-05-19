import numpy as np
import utility as ram

class Robot:
    class Body:
        """Defines the body (links) of the robot."""
        def __init__(self, parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range):
            self.parent = parent
            self.name = name
            self.pos = np.array(pos)
            self.quat = np.array(quat)
            self.ipos = np.array(ipos)
            self.iquat = np.array(iquat)
            self.mass = mass
            self.inertia = np.array(inertia)
            self.joint_axis = np.array(joint_axis)
            self.joint_range = np.array(joint_range)

    class Params:
        """Class to hold heterogeneous robot-level parameters."""
        def __init__(self):
            base_quat = np.array([1, 0, 0, 0])
            self.base_quat = ram.quat_normalize(base_quat)

            self.end_eff_pos_local = np.array([0.016, 0, 0.111])
            end_eff_quat_local = np.array([1, 0, 0, 0])
            self.end_eff_quat_local = ram.quat_normalize(end_eff_quat_local)

    def __init__(self):
        self.body = {}
        self.params = Robot.Params()  # Initialize robot parameters

    def add_body(self, body_id, parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range):
        self.body[body_id] = Robot.Body(parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range)


# Initialize the robot
robot = Robot()

# Add bodies
robot.add_body(
    1, parent='ground', name='shoulder_link', pos=[0, 0, 0.095],
    quat=[1, 0, 0, 0], ipos=[2.23482e-05, 4.14609e-05, 0.0066287], iquat=[0.0130352, 0.706387, 0.012996, 0.707586],
    mass=0.480879, inertia=[0.000588946, 0.000555655, 0.000378999],
    joint_axis=[0, 0, 1], joint_range=[-1.5, 1.5]
)


robot.add_body(
    2, parent='shoulder_link', name='upper_arm_link', pos=[0, 0, 0.065],
    quat=[1, 0, 0, 0], ipos=[0.0171605, 2.725e-07, 0.191323], iquat=[0.705539, 0.0470667, -0.0470667, 0.705539],
    mass=0.430811, inertia=[0.00364425, 0.003463, 0.000399348],
    joint_axis=[0, 1, 0], joint_range=[-1.52, 1.52]
)


robot.add_body(
    3, parent='upper_arm_link', name='upper_forearm_link', pos=[0, 0, 0.101],
    quat=[1, 0, 0, 0], ipos=[0.107963, 0.000115876, 0], iquat=[0.000980829, 0.707106, -0.000980829, 0.707106],
    mass=0.234589, inertia=[0.000888, 0.000887807, 3.97035e-05],
    joint_axis=[0, 1, 0], joint_range=[-0, 1.832]
)

robot.add_body(
    4, parent='upper_forearm_link', name='lower_forearm_link', pos=[0, 0, 0.094],
    quat=[1, 0, 0, 0], ipos=[0.0374395, 0.00522252, 0], iquat=[-0.0732511, 0.703302, 0.0732511, 0.703302],
    mass=0.220991, inertia=[0.0001834, 0.000172527, 5.88633e-05],
    joint_axis=[0, 1, 0], joint_range=[-2.006, 2.006]
)

# robot.add_body(
#     5, parent='lower_forearm_link', name='end_effoctor', pos=[0.016, 0, 0.111],
#     quat=[1, 0, 0, 0], ipos=[0.0374395, 0.00522252, 0], iquat=[-0.0732511, 0.703302, 0.0732511, 0.703302],
#     mass=0.220991, inertia=[0.0001834, 0.000172527, 5.88633e-05]
# )



# Normalize quaternions using the rotation library
for body_id, body in robot.body.items():
    body.quat = ram.quat_normalize(body.quat)
    body.iquat = ram.quat_normalize(body.iquat)

# Example of parameter usage
#print(robot.params.end_eff_pos_local)  # Access robot-level parameter
#print(robot[1])  # Access body
