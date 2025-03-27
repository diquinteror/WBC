import sys
import os

src_dir = os.path.dirname(os.path.abspath(__file__))
api_dir = os.path.abspath(os.path.join(src_dir, "..", "api"))
if api_dir not in sys.path:
    sys.path.append(api_dir)
    
import sim
import Task
import Robot
import cvxpy as cp
import numpy as np
import time
import Enviroment
from scipy.spatial.transform import Rotation as R
import RealTimePlotter as RTP

class Control:

    def __init__(self, robot: Robot.WBC13ddl, world:Enviroment.Enviroment, task:Task.Task):
        self.robot:Robot.WBC13ddl = robot
        self.task:Task.Task = task
        self.spheres = world.getSpheres()
        self.Kp = np.eye(6)
        self.K_distance = 0.5
        self.v_max_joint = 0.01
        self.robot_point_radius = 0.20
        self.security_distance = 0.0001
        self.warning_distance = 0.15
        self.dq_ans = None

    def quadOptimisation(self, dX, T_list, J_list):
        J = J_list[-1] #Last Jacobian

        # Variable
        dq = cp.Variable(13)

        H = J.T @ J

        if np.linalg.matrix_rank(J) == min(J.shape):
            f = J.T @ self.Kp @ (-dX)
        else:
            Jpinv = np.linalg.pinv(J)
            f = J.T @ J @ Jpinv @ self.Kp @ (-dX)

        alpha = 0.2;
        qp_function = cp.Minimize((1/2) * cp.quad_form(dq, H) + alpha*f.T @ (dq-self.dq_ans))
        
        ############ Restrictions ############
        restrictions = [] 

        ## Derivative control (Avoid vibrations)
        k_derivative = 100
        acc_max = 0.005
        if self.dq_ans is not None:
            restrictions.append(-k_derivative*acc_max+np.abs(self.dq_ans)<=dq)
            restrictions.append(dq<=k_derivative*acc_max-np.abs(self.dq_ans))

        ## Obstacle avoidance
        # Spheres
        self.dist_obs = []
        for articulation in range(12):
            J_i = J_list[articulation][:3,:13]
            X_i = T_list[articulation][:3, 3]
            if articulation == 8 or articulation == 10:
                X_i = (T_list[articulation][:3, 3]+T_list[articulation+1][:3, 3])/2

            # For every obstacle
            for sphere in self.spheres:
                sphere_pose = sphere.getPose()
                d_from_centers = np.linalg.norm(X_i - sphere_pose)
                sphere_radius = sphere.getRadius()

                distance_to_surface = d_from_centers - sphere_radius - self.robot_point_radius
                self.dist_obs.append(distance_to_surface)
                
                if d_from_centers < sphere_radius + self.robot_point_radius + self.warning_distance :
                    self.K_distance = 0.5*max(0.0, min(1.0, distance_to_surface / self.warning_distance))

                    A = -2*(X_i-sphere_pose).T @ J_i
                    
                    d_min = self.robot_point_radius + self.security_distance
                    b = -self.K_distance*(d_min-d_from_centers)
                    restrictions.append(A @ dq <= b)

        ## Joint restrictions
        #Gimball lock (Max angle = +- 60 degrees)

        joint_values = self.robot.get_joint_values()
        PrismX = joint_values[0]
        PrismY = joint_values[1]
        PrismZ = joint_values[2]
        roll = joint_values[3]
        pitch = joint_values[4]
        yaw = joint_values[5]
        kuka = joint_values[5:]
        inv_speed = 0.0001

        out_of_limits = [np.abs(PrismX) >= 0.4,
                           np.abs(PrismY) >= 0.4,
                           np.abs(PrismZ) >= 0.4,
                           np.abs(roll) >= np.pi/4,
                           np.abs(pitch) >= np.pi/4,
                           np.abs(yaw) >= np.pi/4,
                           np.abs(kuka[0]) >= 170*np.pi/180,
                           np.abs(kuka[1]) >= 120*np.pi/180,
                           np.abs(kuka[2]) >= 170*np.pi/180,
                           np.abs(kuka[3]) >= 120*np.pi/180,
                           np.abs(kuka[4]) >= 170*np.pi/180,
                           np.abs(kuka[5]) >= 120*np.pi/180,
                           np.abs(kuka[6]) >= 175*np.pi/180]

        Aeq = [[out_of_limits[0], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
               [0, out_of_limits[1], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
               [0, 0, out_of_limits[2], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
               [0, 0, 0, out_of_limits[3], 0, 0, 0, 0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, out_of_limits[4], 0, 0, 0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, out_of_limits[5], 0, 0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, out_of_limits[6], 0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, out_of_limits[7], 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, out_of_limits[8], 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, out_of_limits[9], 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, out_of_limits[10], 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, out_of_limits[11], 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, out_of_limits[12]]]
        

        beq = [-self.__sign(PrismX)*inv_speed*out_of_limits[0],
               -self.__sign(PrismY)*inv_speed*out_of_limits[1],
               -self.__sign(PrismZ)*inv_speed*out_of_limits[2],
               -self.__sign(roll)*inv_speed*out_of_limits[3],
               -self.__sign(pitch)*inv_speed*out_of_limits[4],
               -self.__sign(yaw)*inv_speed*out_of_limits[5],
               -self.__sign(kuka[0])*inv_speed*out_of_limits[6],
               -self.__sign(kuka[1])*inv_speed*out_of_limits[7],
               -self.__sign(kuka[2])*inv_speed*out_of_limits[8],
               -self.__sign(kuka[3])*inv_speed*out_of_limits[9],
               -self.__sign(kuka[4])*inv_speed*out_of_limits[10],
               -self.__sign(kuka[5])*inv_speed*out_of_limits[11],
               -self.__sign(kuka[6])*inv_speed*out_of_limits[12]]
        
        # Aeq = [[0,0,0,1,0,0,0,0,0,0,0,0,0],
        #        [0,0,0,0,1,0,0,0,0,0,0,0,0],
        #        [0,0,0,0,0,1,0,0,0,0,0,0,0]]
        # beq = [0,0,0]
        for Aeq_i,beq_i in zip(Aeq,beq):
            restrictions.append(Aeq_i @ dq == beq_i)

        # Max speed restrictions
        lb= self.v_max_joint * np.ones_like(13)
        ub=-lb;
        restrictions.append(ub<=dq)
        restrictions.append(dq<=lb)

        ############# Solution ############
        prob = cp.Problem(qp_function, restrictions)

        try:
            result = prob.solve(cp.MOSEK)
        except cp.error.SolverError:
            print("Error: Cannot solve QP Problem")
            result = None  # Return None if error
        
        return dq.value
        
    def getControlJointSpeedValue(self, atime):
        T_list = self.robot.forward_kinematics()
        J_list = self.robot.compute_jacobian()

        X = (T_list[-1])[:3, 3] #Last position

        dX = self.task.getDesiredPosition(atime, X) - np.concatenate((X, np.array([0, 0, 0])))
        dX[3] = dX[4] = dX[5] = 0;


        norm_error = np.linalg.norm(dX)

        if self.task.get_type_of_task() == 'WAYPOINTS':
            if norm_error < 0.05:
                self.v_max_joint = 0.001
            elif norm_error < 0.15:
                m = (0.001-0.01)/(0.05-0.15)
                self.v_max_joint = (norm_error-0.15)*m+0.01
            else:
                self.v_max_joint = 0.01
        
        #dq = np.linalg.pinv(J_list[-1]) @ dX

        dq = self.quadOptimisation(dX, T_list, J_list)

        #Filtering dq
        beta = 0.6
        if self.dq_ans is not None :
            dq =  beta * self.dq_ans + (1 - beta) * dq
        
        #Saving dq
        self.dq_ans = dq
        
        return dq
    
    def getControlJointPosValue(self, time, q0 = None):
        if q0 is None:
            q = self.robot.get_joint_values()
        else :
            q = q0
        q = q + self.getControlJointSpeedValue(time)
        return q
    
    def __sign(self,x):
        return 1 if x >= 0 else -1