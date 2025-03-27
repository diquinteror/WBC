# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import sys
import os

src_dir = os.path.dirname(os.path.abspath(__file__))
api_dir = os.path.abspath(os.path.join(src_dir, "..", "api"))
if api_dir not in sys.path:
    sys.path.append(api_dir)

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import Control
import Robot
import time
import numpy as np
import Task
import Enviroment
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def connect_to_sim():
    """connect to CoppeliaSim"""
    sim.simxFinish(-1)  # close all connections before
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:
        print("successfully connect to CoppeliaSim")
    else:
        raise Exception("false，pls check CoppeliaSim and remote API are active")
    return client_id


def main():
    client_id = connect_to_sim()

    interval=0.1;
    last_execution_time = 0.0;

    # Object definition
    world = Enviroment.Enviroment(client_id)
    robot = Robot.WBC13ddl(client_id)
    track = world.getTack()
    task = Task.Task(task='WAYPOINTS', waypoints=track)
    control = Control.Control(robot, world, task)
    
    #initial position
    q_0 = [0, 0, 0, 
           0, 0, 0, 
           -np.pi/2, np.pi/4, 0, np.pi/4, 0, np.pi/2, 0]

    T = 10
    dt = 0.05
    robot.set_joint_positions(q_0)
    
    time.sleep(1)
    sim_time = 0;

    while(True):

        # ------ Speed Control
        dq = control.getControlJointSpeedValue(sim_time)
        robot.set_joint_velocity(dq)
        
        sim_time = sim_time + dt

        if(task.task_finished()):
            break
        

    sim.simxFinish(client_id)
    print(" disconnection to CoppeliaSim ")

if __name__ == '__main__':
    main()
