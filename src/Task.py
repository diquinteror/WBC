import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib.pyplot as plt
import RealTimePlotter as RTP

class Task:
    def __init__(self, type = 'POSE', task = 'SIN', waypoints = []):
        self.type = type
        self.task = task
        self.time_save = None
        self.task_finished_flag = False

        if task == 'WAYPOINTS':
            self.waypoints = waypoints
            self.waypoints_index = 0

    def get_type_of_task(self):
        return self.task
    
    def task_finished(self):
        return self.task_finished_flag
    
    def getDesiredPosition(self, atime, actual_pose = 0):
        match self.task:
            case 'SIN':
                position = np.array([
                    0.4* np.sin(np.pi * atime / 4),
                    1.05,
                    0.75
                ])
                orientation = np.array([np.pi, 0, np.pi])
                vec_error = actual_pose - position

                self.error_plotter.update(atime, vec_error)
                self.pose_plotter.update(atime, np.concatenate((actual_pose,position)))
                return np.concatenate((position, orientation))
            case 'WAYPOINTS':
                norm_error = np.linalg.norm(np.array(actual_pose) - np.array(self.waypoints[self.waypoints_index]))
                vec_error = np.array(actual_pose) - np.array(self.waypoints[self.waypoints_index])

                if  norm_error < 0.01 :
                    if self.time_save is None:
                        self.time_save = time.time()
                    
                    if time.time() - self.time_save > 3:
                        
                        self.time_save = None

                        if len(self.waypoints) - 1 == self.waypoints_index:
                            print('Track finished !!!')
                            #self.task_finished_flag = True
                            self.waypoints_index=0

                        else:
                            print(f'Arrived at point {self.waypoints_index}')
                            self.waypoints_index += 1
                else:
                    self.time_save = None

                orientation = np.array([np.pi, 0, np.pi])

                return np.concatenate((self.waypoints[self.waypoints_index], orientation))
            case _:
                return np.zeros(6)

