import pybullet as p
import pybullet_data
import math
import numpy as np
import time
from ur_ikfast import ur_kinematics
import sympy as sp

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

x, y, z = sp.symbols('x y z')
ur5e=ur_kinematics.URKinematics('ur5e')

planeId = p.loadURDF("plane.urdf")
ur5_orientation_euler = [0, 0, 3.14] 
ur5_orientation_quaternion = p.getQuaternionFromEuler(ur5_orientation_euler)
ur5e_arm = ur_kinematics.URKinematics('ur5e')
robotId = p.loadURDF("/home/hp/catkin_ws/src/UR5_Jacobian/UR5.urdf", basePosition=[0, 0, 0], baseOrientation=ur5_orientation_quaternion)
initial_joint_positions = [0, -2.24, -1.65, -0.87, -0.07, -0.29]
p.resetJointState(robotId, 1, 0, targetVelocity=0)
p.resetJointState(robotId, 2, -2.24, targetVelocity=0)
p.resetJointState(robotId, 3, -1.65, targetVelocity=0)
p.resetJointState(robotId, 4, -0.87, targetVelocity=0)
p.resetJointState(robotId, 5, -0.07, targetVelocity=0)
p.resetJointState(robotId, 6, -0.29, targetVelocity=0)

num_joints = p.getNumJoints(robotId)
goal=(0,0,0.6)

sphere_radius = 0.05
sphere_visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[0, 1, 0, 1])
sphere_collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
sphere_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_visual_id, baseCollisionShapeIndex=sphere_collision_id, basePosition=goal)

def distance(coord, point):
    dist = sp.sqrt((coord[0] - point[0])**2 + (coord[1] - point[1])**2 + (coord[2] - point[2])**2)
    return dist

x_val, y_val, z_val = (100, 1.9, 0.2)
U_attract = distance((x, y, z), goal)
U_tot = U_attract

# Compute the symbolic gradients
U_x = -sp.diff(U_tot, x)
U_y = -sp.diff(U_tot, y)
U_z = -sp.diff(U_tot, z)
print(U_y)

# Substitute values into the gradients
U_x_sub = U_x.subs({x: x_val, y: y_val, z: z_val})
U_y_sub = U_y.subs({x: x_val, y: y_val, z: z_val})
U_z_sub = U_z.subs({x: x_val, y: y_val, z: z_val})

# Round after substitution
U_x_rounded = round(float(U_x_sub), 2)
U_y_rounded = round(float(U_y_sub), 2)
U_z_rounded = round(float(U_z_sub), 2)

# Calculate magnitude using unrounded values
magnitude = sp.sqrt(U_x_sub**2 + U_y_sub**2 + U_z_sub**2)
rounded_magnitude = np.sqrt(U_x_rounded**2 + U_y_rounded**2 + U_z_rounded**2)
print(f"pot_x: {U_x_rounded}")
print(f"pot_y: {U_y_rounded}")
print(f"pot_z: {U_z_rounded}")
print(f"Rounded Mag_pot: {rounded_magnitude}")


# print(f"Mag_pot: {magnitude.evalf()}")


def mag_potential(coordinate):
    x_val,y_val,z_val=coordinate
    U_attract=distance((x,y,z),goal)
    U_tot=U_attract
    U_x = -sp.diff(U_tot, x)
    U_y = -sp.diff(U_tot, y)
    U_z = -sp.diff(U_tot, z)
      # Substitute numerical values
    pot_x = sp.N(U_x.subs({x: x_val, y: y_val, z: z_val}),2)
    pot_y = sp.N(U_y.subs({x: x_val, y: y_val, z: z_val}),2)
    pot_z = sp.N(U_z.subs({x: x_val, y: y_val, z: z_val}),2)
    
    # Compute magnitude
    mag_neg_U_grad_tot = -np.sqrt(pot_x**2 + pot_y**2 + pot_z**2)
    
    return mag_neg_U_grad_tot
print(mag_potential((0.2,0.5,1.3)))


