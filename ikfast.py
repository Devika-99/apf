# from ur_ikfast import ur_kinematics

# ur3e_arm = ur_kinematics.URKinematics('ur3e')

# joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians
# print("joint angles", joint_angles)

# pose_quat = ur3e_arm.forward(joint_angles)
# pose_matrix = ur3e_arm.forward(joint_angles, 'matrix')

# print("forward() quaternion \n", pose_quat)
# print("forward() matrix \n", pose_matrix)

# # print("inverse() all", ur3e_arm.inverse(pose_quat, True))
# print("inverse() one from quat", ur3e_arm.inverse(pose_quat, False, q_guess=joint_angles))

# print("inverse() one from matrix", ur3e_arm.inverse(pose_matrix, False, q_guess=joint_angles))

import sympy as sp
import numpy as np

# Define symbolic variables
x, y, z = sp.symbols('x y z')

# Define the obstacle and goal positions
obstacle = (0.5, 0.5, 0.5)
goal = (0.3, 0.5, 0.9)

# Symbolic distance function
def distance(coord, point):
    dist = sp.sqrt((coord[0] - point[0])**2 + (coord[1] - point[1])**2 + (coord[2] - point[2])**2)
    return dist

# Function to compute velocities from potential fields
def velocities(coordinate):
    # Extract x, y, z values from the current position
    x_val, y_val, z_val = coordinate
    
    # Compute the attraction and repulsion potentials symbolically
    U_attract = distance((x, y, z), goal)
    U_rep = 1 / distance((x, y, z), obstacle)
    
    # Total potential
    U_tot = U_attract + U_rep
    
    # Scaling factor for velocities
    k = 0.1
    
    # Compute the symbolic velocity components
    V_x = -k * sp.diff(U_tot, x)
    V_y = -k * sp.diff(U_tot, y)
    V_z = -k * sp.diff(U_tot, z)
    
    # Substitute the actual coordinate values into the symbolic expressions
    vel_x = V_x.subs({x: x_val, y: y_val, z: z_val})
    vel_y = V_y.subs({x: x_val, y: y_val, z: z_val})
    vel_z = V_z.subs({x: x_val, y: y_val, z: z_val})
    
    return vel_x, vel_y, vel_z

d_thres=0.1
def mag_potential(coordinate):
    x_val,y_val,z_val=coordinate
    U_attract=distance((x,y,z),goal)
    if distance(coordinate,goal)>d_thres:
        U_rep=1/(distance((x,y,z),obstacle))
    else:
        U_rep=0
    U_tot=U_attract+U_rep  
    U_x = -sp.diff(U_tot, x)
    U_y = -sp.diff(U_tot, y)
    U_z = -sp.diff(U_tot, z)
      # Substitute numerical values
    pot_x = float(U_x.subs({x: x_val, y: y_val, z: z_val}))
    pot_y = float(U_y.subs({x: x_val, y: y_val, z: z_val}))
    pot_z = float(U_z.subs({x: x_val, y: y_val, z: z_val}))
    
    # Compute magnitude
    mag_neg_U_grad_tot = np.sqrt(pot_x**2 + pot_y**2 + pot_z**2)
    
    return mag_neg_U_grad_tot

coordinate1=[0.2,0.4,0.9]
coordinate2=[1.2,1.5,0.6]
print(f"Mag1:{mag_potential(coordinate1)},distance:{distance(coordinate1,goal)}")

print(f"Mag2:{mag_potential(coordinate2)},distance :{distance(coordinate2,goal)}")