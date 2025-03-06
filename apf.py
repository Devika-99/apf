import pybullet as p
import pybullet_data 
import numpy as np
import math
import time
from ur_ikfast import ur_kinematics
import sympy as sp
x, y, z = sp.symbols('x y z')
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

ur5e_arm = ur_kinematics.URKinematics('ur5e')

planeId = p.loadURDF("plane.urdf")
ur5_orientation_euler = [0, 0, 3.14] 
ur5_orientation_quaternion = p.getQuaternionFromEuler(ur5_orientation_euler)
robotId = p.loadURDF("/home/hp/catkin_ws/src/UR5_Jacobian/UR5.urdf", basePosition=[0, 0, 0], baseOrientation=ur5_orientation_quaternion)

obstacle=(0.5,0.5,0.5)
goal=(0.3,0.5,0.9)

sphere_radius = 0.05
sphere_visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[1, 0, 0, 1])
sphere_collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
sphere_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_visual_id, baseCollisionShapeIndex=sphere_collision_id, basePosition=obstacle)

sphere_radius = 0.05
sphere_visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[0, 1, 0, 1])
sphere_collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
sphere_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_visual_id, baseCollisionShapeIndex=sphere_collision_id, basePosition=goal)
d_thres=0.1
def distance(coordinate,point):
    dist=np.linalg.norm(np.array(coordinate)-np.array(point))
    return dist


def grad_U_attract(coordinate):
    grad_U_att=(np.array(coordinate)-np.array(goal))/(distance(coordinate,goal))

    return grad_U_att

def grad_U_rep(coordinate):
    if distance(coordinate,obstacle)>d_thres:
        grad_U_rep=(np.array(coordinate)-np.array(obstacle))/(distance(coordinate,obstacle))**3
    else:
        grad_U_rep=0
    return grad_U_rep

def tot_neg_grad(coordinate):
    tot=grad_U_attract(coordinate)+grad_U_rep(coordinate)
    return tot

step=0.1
def neighbours(coordinate):
    neighbours = [
            (round(coordinate[0] + step, 2), round(coordinate[1], 2), round(coordinate[2], 2)),
            (round(coordinate[0] - step, 2), round(coordinate[1], 2), round(coordinate[2], 2)),
            (round(coordinate[0], 2), round(coordinate[1] + step, 2), round(coordinate[2], 2)),
            (round(coordinate[0], 2), round(coordinate[1] - step, 2), round(coordinate[2], 2)),
            (round(coordinate[0], 2), round(coordinate[1], 2), round(coordinate[2] + step, 2)),
            (round(coordinate[0], 2), round(coordinate[1], 2), round(coordinate[2] - step, 2))
        ]
    return neighbours
def velocities(coordinate):
     x_val,y_val,z_val=coordinate
     U_attract=distance((x,y,z),goal)
     U_rep=1/(distance((x,y,z),obstacle))
     U_tot=U_attract+U_rep 
     k=0.1 
     V_x = -k*sp.diff(U_tot, x)
     V_y = -k*sp.diff(U_tot, y)
     V_z = -k*sp.diff(U_tot, z)
     vel_x=V_x.subs({x:x_val,y:y_val,z:z_val})
     vel_y=V_y.subs({x:x_val,y:y_val,z:z_val})
     vel_z=V_z.subs({x:x_val,y:y_val,z:z_val})
     
     return (vel_x,vel_y,vel_z)


def Jacobian(theta1,theta2,theta3,theta4,theta5,theta6):

                    J11 =(math.cos(theta1) * (0.1333 + 0.0996 * math.cos(theta5)) + 
            math.sin(theta1) * (-0.0997 * math.cos(theta4) * math.sin(theta2 + theta3) + 
            math.cos(theta2) * (0.425 + math.cos(theta3) * (0.3922 - 0.0997 * math.sin(theta4))) + 
            math.sin(theta2) * math.sin(theta3) * (-0.3922 + 0.0997 * math.sin(theta4)) + 
            0.0996 * math.cos(theta2 + theta3 + theta4) * math.sin(theta5)))



                    J12 = (0.0997 * math.cos(theta1) * 
            (1.0 * math.cos(theta2 + theta3 + theta4) + 
            0.499498 * math.cos(theta2 + theta3 + theta4 - theta5) - 
            0.499498 * math.cos(theta2 + theta3 + theta4 + theta5) + 
            4.26279 * math.sin(theta2) + 
            3.9338 * math.sin(theta2 + theta3)))

                    J13 = (0.0997 * math.cos(theta1) * 
            (1.0 * math.cos(theta2 + theta3 + theta4) + 
            0.499498 * math.cos(theta2 + theta3 + theta4 - theta5) - 
            0.499498 * math.cos(theta2 + theta3 + theta4 + theta5) + 
            3.9338 * math.sin(theta2 + theta3)))




                    J14 = (math.cos(theta1) * 
            (0.0997 * math.cos(theta2 + theta3 + theta4) + 
            0.0996 * math.sin(theta2 + theta3 + theta4) * math.sin(theta5)))
                    
                    J15 = (-0.0996 * math.cos(theta1) * math.cos(theta2 + theta3 + theta4) * math.cos(theta5) - 
            0.0996 * math.sin(theta1) * math.sin(theta5))




                    J16 = 0

                    J21 =  ((0.1333 + 0.0996 * math.cos(theta5)) * math.sin(theta1) + 
            math.cos(theta1) * (-0.425 * math.cos(theta2) - 
            0.3922 * math.cos(theta2 + theta3) + 
            0.0997 * math.sin(theta2 + theta3 + theta4) + 
            0.0498 * math.sin(theta2 + theta3 + theta4 - theta5) - 
            0.0498 * math.sin(theta2 + theta3 + theta4 + theta5)))



                    J22 =(0.0997 * math.sin(theta1) * 
            (1.0 * math.cos(theta2 + theta3 + theta4) + 
            0.499498 * math.cos(theta2 + theta3 + theta4 - theta5) - 
            0.499498 * math.cos(theta2 + theta3 + theta4 + theta5) + 
            4.26279 * math.sin(theta2) + 
            3.9338 * math.sin(theta2 + theta3)))



                    J23 = (math.sin(theta1) * 
            (0.0997 * math.cos(theta2 + theta3 + theta4) + 
            0.3922 * math.cos(theta3) * math.sin(theta2) + 
            0.3922 * math.cos(theta2) * math.sin(theta3) + 
            0.0996 * math.sin(theta2 + theta3 + theta4) * math.sin(theta5)))


                    J24 = (math.sin(theta1) * 
            (0.0997 * math.cos(theta2 + theta3 + theta4) + 
            0.0996 * math.sin(theta2 + theta3 + theta4) * math.sin(theta5)))

                    J25 =(-0.0996 * math.cos(theta2 + theta3 + theta4) * math.cos(theta5) * math.sin(theta1) + 
            0.0996 * math.cos(theta1) * math.sin(theta5))

                    J26 = 0

                    J31 = 0

                    J32 = (-0.425 * math.cos(theta2) - 
            0.3922 * math.cos(theta2 + theta3) + 
            0.0997 * math.sin(theta2 + theta3 + theta4) + 
            0.0498 * math.sin(theta2 + theta3 + theta4 - theta5) - 
            0.0498 * math.sin(theta2 + theta3 + theta4 + theta5))

                    J33 = (-0.3922 * math.cos(theta2 + theta3) + 
            0.0997 * math.sin(theta2 + theta3 + theta4) + 
            0.0498 * math.sin(theta2 + theta3 + theta4 - theta5) - 
            0.0498 * math.sin(theta2 + theta3 + theta4 + theta5))

                    J34 = (0.0997 * math.sin(theta2 + theta3 + theta4) + 
            0.0498 * math.sin(theta2 + theta3 + theta4 - theta5) - 
            0.0498 * math.sin(theta2 + theta3 + theta4 + theta5))

                    J35 =-0.0996 * math.cos(theta5) * math.sin(theta2 + theta3 + theta4)


                    J36 = 0
                    J41=0
                    J42=math.sin(theta2)
                    J43=math.sin(theta3)
                    J44=math.sin(theta4)
                    J45=0
                    J46=math.sin(theta6)
                    J51=0
                    J52=0
                    J53=0
                    J54=0
                    J55=0
                    J56=0
                    J61=1
                    J62=math.cos(theta2)
                    J63=math.cos(theta3)
                    J64=math.cos(theta4)
                    J65=1
                    J66=math.cos(theta6)

                    J = np.array([
                    [J11, J12, J13, J14, J15, J16],
                    [J21, J22, J23, J24, J25, J26],
                    [J31, J32, J33, J34, J35, J36],
                    [J41, J42, J43, J44, J45, J46],
                    [J51, J52, J53, J54, J55, J56],
                    [J61, J62, J63, J64, J65, J66]
                    ])
                    return J

joint_states = p.getJointStates(robotId, [1, 2, 3])
joint_positions = [state[0] for state in joint_states]
theta_1, theta_2, theta_3,theta_4,theta_5,theta_6 = joint_positions
pose_squat=ur5e_arm.forward((theta_1,theta_2,theta_3,theta_4,theta_5,theta_6))
current_node=pose_squat[:3]
print(current_node)
default_orientation = [1.0, 0.0, 0.0, 0.0]
min_field=np.inf
best_neighbour=None
while distance(current_node,goal)>0.5:
    nbs=neighbours(current_node)
    for nb in nbs:
        field=tot_neg_grad(nb)
        if field<min_field:
            min_field=field
            best_neighbour=nb
    angles=ur5e_arm.inverse(best_neighbour+default_orientation,q_guess=[0, 0, 0, 0, 0, 0])

    p.setJointMotorControl2(robotId, 1, p.POSITION_CONTROL, targetPosition=angles[0])
    p.setJointMotorControl2(robotId, 2, p.POSITION_CONTROL, targetPosition=angles[1])
    p.setJointMotorControl2(robotId, 3, p.POSITION_CONTROL, targetPosition=angles[2])
    current_node=best_neighbour
    min_field=np.inf
    best_neighbour=None

while p.isConnected():
    p.stepSimulation()
    time.sleep(0.01)

