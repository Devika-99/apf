import pybullet as p
import pybullet_data
import math
import numpy as np
import time
from ur_ikfast import ur_kinematics

ur5e=ur_kinematics.URKinematics('ur5e')
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
ur5_orientation_euler = [0, 0, 3.14] 
ur5_orientation_quaternion = p.getQuaternionFromEuler(ur5_orientation_euler)
robotId = p.loadURDF("/home/hp/catkin_ws/src/UR5_Jacobian/UR5.urdf", basePosition=[0, 0, 0], baseOrientation=ur5_orientation_quaternion)
p.setJointMotorControl2(robotId, 5, p.POSITION_CONTROL, targetPosition=np.pi)  
while p.isConnected():
    p.stepSimulation()
    time.sleep(0.01)