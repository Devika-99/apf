import pybullet as p
import pybullet_data
import math
import numpy as np
import time
from ur_ikfast import ur_kinematics
import sympy as sp


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
ur5_orientation_euler = [0, 0, 3.14] 
ur5_orientation_quaternion = p.getQuaternionFromEuler(ur5_orientation_euler)
ur5e_arm = ur_kinematics.URKinematics('ur5e')
robotId = p.loadURDF("/home/hp/catkin_ws/src/UR5_Jacobian/UR5.urdf", basePosition=[0, 0, 0], baseOrientation=ur5_orientation_quaternion)

num_joints = p.getNumJoints(robotId)
joint_states_1 = p.getJointStates(robotId, list(range(num_joints)))
joint_positions_1 = [state[0] for state in joint_states_1]
theta1, theta2, theta3, theta4, theta5, theta6 = joint_positions_1[1:7]
theta1+=3.14
pose_squat=ur5e_arm.forward((theta1,theta2,theta3,theta4,theta5,theta6))
current_position=pose_squat[:3]
print(current_position)
while p.isConnected():
    p.stepSimulation()
    time.sleep(0.01)