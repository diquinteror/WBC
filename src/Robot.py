import numpy as np
import os
import sys
import time

src_dir = os.path.dirname(os.path.abspath(__file__))
api_dir = os.path.abspath(os.path.join(src_dir, "..", "api"))
if api_dir not in sys.path:
    sys.path.append(api_dir)
    
import sim
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from scipy.spatial.transform import Rotation as R
import RealTimePlotter as RTP

class WBC13ddl:

    def __init__(self, client_id , name : str = 'LBR_iiwa_7_R800_connection', number : int = 1):
        self.number = number
        self.name = name
        self.client_id = client_id
        self.set_ZeroMQ_api()
        
        self.joint_handles = self.get_joint_handles()
        self.joint_values = self.get_joint_values()
        self.joint_types = ['P','P','P','R','R','R','R','R','R','R','R','R','R']
        self.jointValueActualization()

        
        
    def set_ZeroMQ_api(self):
        client = RemoteAPIClient()
        self.simZMQ = client.require('sim')

    def jointValueActualization(self):
        # mettre a jour la configuration de robot
        self.joint_values = self.get_joint_values()
        # [tht, d, a, alpha]
        self.dh_params = [
            (np.pi/2, 0, 0, 0), #Adjusting to world coordinates
            (0, self.joint_values[0],0, np.pi/2),  # Joint 1 prismatic
            (np.pi/2, 1.85+self.joint_values[1], 0, -np.pi/2),  # Joint 2 prismatic
            (np.pi/2, 0.7+self.joint_values[2], 0, np.pi / 2),  # Joint 3 prismatic
            
            (self.joint_values[3], 0, -0.14, -np.pi / 2),  # Joint 1 spheric
            (np.pi/2 - self.joint_values[4], 0, 0, np.pi / 2),  # Joint 2 spheric
            (-self.joint_values[5], 0, 0, -np.pi/2),  # Joint 3 spheric
            
            (self.joint_values[6], 0.26125, 0, 0),  # Joint 1 kuka
            (self.joint_values[7], 0, 0, -np.pi / 2),  # Joint 2 kuka
            (self.joint_values[8], 0.4, 0, np.pi / 2),  # Joint 3 kuka
            (self.joint_values[9], 0., 0, np.pi / 2),  # Joint 4 kuka
            (self.joint_values[10], 0.4, 0, -np.pi / 2),  # Joint 5 kuka
            (self.joint_values[11], 0, 0, -np.pi / 2),  # Joint 6 kuka
            (self.joint_values[12], 0.19875, 0, np.pi / 2)  # Joint 7 kuka
        ]

    def dh_transform(self, a, alpha, d, theta):
        """transform matrix"""
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self):
        # LBR iiwa 7 R800 D-H table
        self.jointValueActualization()
        dh_params = self.dh_params

        T = np.eye(4)
        transforms = []  # transform matrix for every coude
        for i in range(len(dh_params)):
            theta, d, a, alpha = dh_params[i]
            T_next = self.dh_transform(a, alpha, d, theta)
            T = T @ T_next 
            transforms.append(T)
        
        transforms.pop(0) #Deletes the first transformation who works for adjusting to World Cordinators
        return transforms

    def compute_jacobian(self):
        """Jacobien matrix"""
        transforms = self.forward_kinematics()
        T_end = transforms[-1]          # Matriz de transformación total
        p_end = T_end[:3, 3]            # Vector de posición del efector final

        J = np.zeros((6, 13))
        jacobians = []

        for i in range(13):
            T_i = transforms[i]
            z_i = T_i[:3, 2]          # Eje z de la i-ésima articulación
            p_i = T_i[:3, 3]          # Posición de la i-ésima articulación

            if self.joint_types[i] == 'R':  # Articulación rotoide
                J_v = np.cross(z_i, p_end - p_i)
                J_w = z_i
            elif self.joint_types[i] == 'P':  # Articulación prismática
                J_v = z_i
                J_w = np.zeros(3)
            else:
                raise ValueError(f"Tipo de articulación desconocido en el índice {i}")

            J[:3, i] = J_v
            J[3:, i] = J_w

            # Se agrega una copia del Jacobiano actualizado hasta esta articulación
            jacobians.append(J.copy())

        return jacobians
    
    def get_joint_values(self):
        joint_handles = self.joint_handles
        posValue = []
        # Get values
        for joint_handle in joint_handles:
            value = self.simZMQ.getJointPosition(joint_handle)
            posValue.append(value)
        
        return posValue
    
    # """Get the handles of the n-robot arm joint(two series)"""
    def get_joint_handles(self):
        joint_handles = []
        #3PS Handles
        #Prismatic joints
        err_code1, jointHandle1 = sim.simxGetObjectHandle(self.client_id, 'Prismatic_joint0', sim.simx_opmode_blocking)
        err_code2, jointHandle2 = sim.simxGetObjectHandle(self.client_id, 'Prismatic_joint', sim.simx_opmode_blocking)
        err_code3, jointHandle3 = sim.simxGetObjectHandle(self.client_id, 'Prismatic_joint1', sim.simx_opmode_blocking)

        if err_code1 == 0 & err_code2 == 0 & err_code3 == 0:
            joint_handles.append(jointHandle1)
            joint_handles.append(jointHandle2)
            joint_handles.append(jointHandle3)

        else:
            print("Failed to get joint handle")
        
        #Spheric joint
        err_code1, jointHandle1 = sim.simxGetObjectHandle(self.client_id, 'Spheric1', sim.simx_opmode_blocking)
        err_code2, jointHandle2 = sim.simxGetObjectHandle(self.client_id, 'Spheric2', sim.simx_opmode_blocking)
        err_code3, jointHandle3 = sim.simxGetObjectHandle(self.client_id, 'Spheric3', sim.simx_opmode_blocking)

        if err_code1 == 0 & err_code2 == 0 & err_code3 == 0:
            joint_handles.append(jointHandle1)
            joint_handles.append(jointHandle2)
            joint_handles.append(jointHandle3)

        else:
            print("Failed to get joint handle")
        
        #KUKA Handles
        n = self.number
        if n < 0:
            raise Exception("Robot number (n) not valid")

        for i in range(1, 8):  # every LBR iiwa 7R800 has 7 joints
            _, handle = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_joint{i}', sim.simx_opmode_blocking)
            joint_handles.append(handle)

        return joint_handles

    # get the positions of end(two)
    def get_effector_position(self):

        res, end_effector = sim.simxGetObjectHandle(self.client_id, 'Effector', sim.simx_opmode_blocking)

        if res == sim.simx_return_ok:
            res, position = sim.simxGetObjectPosition(self.client_id, end_effector, -1, sim.simx_opmode_blocking)
            if res != sim.simx_return_ok:
                position = 0
        else:
            position = 0
        if position == 0:
            print("Fail to get effector's position")
        
        return position

    # """set the joint target position"""
    def set_joint_positions(self, target_positions):
        joint_handles = self.joint_handles

        # Set Prismatic values
        for joint_handle,taget_pose in zip(joint_handles,target_positions):
            sim.simxSetJointTargetPosition(self.client_id, joint_handle, taget_pose, sim.simx_opmode_oneshot)

    # """set the velocity""
    def set_joint_velocity(self, target_velocity):
        joint_handles = self.joint_handles
        
        # Set Prismatic values
        for joint_handle,taget_vel in zip(joint_handles,target_velocity):
            sim.simxSetJointTargetVelocity(self.client_id, joint_handle, taget_vel, sim.simx_opmode_oneshot)