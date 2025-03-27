import numpy as np
import os
import sys

src_dir = os.path.dirname(os.path.abspath(__file__))
api_dir = os.path.abspath(os.path.join(src_dir, "..", "api"))
if api_dir not in sys.path:
    sys.path.append(api_dir)
    
import sim


class Sphere:

    def __init__(self, pose = [], radius = 0):
        self.pose = pose
        self.radius = radius

    def getPose(self):
        return self.pose
    
    def getRadius(self):
        return self.radius
    

class Enviroment:

    def __init__(self, client_id):
        self.client_id = client_id
        self.track = []
        self.spheres = []
        self.planes = []
        self.setSpheres()
        self.setTrack()

    def setEnviroment(self):
        return 0
    
    def setSpheres(self):
        sphereNum = 0
        sphereHandles = []
        
        error,sphereHandle=sim.simxGetObjectHandle(self.client_id,f'Sphere',sim.simx_opmode_blocking) #First sphere
        if(error == 0):
                sphereHandles.append(sphereHandle)

        while sphereNum < 20 :
            error, sphereHandle = sim.simxGetObjectHandle(self.client_id,f'Sphere{sphereNum}',sim.simx_opmode_blocking)
            if(error == 0):
                sphereHandles.append(sphereHandle)

            sphereNum+=1
        for sphereId in sphereHandles:
            _,pose = sim.simxGetObjectPosition(self.client_id,sphereId,-1,sim.simx_opmode_blocking)
            _,radius = sim.simxGetObjectFloatParameter(self.client_id, sphereId, sim.sim_objfloatparam_objbbox_max_x, sim.simx_opmode_blocking)
            self.spheres.append(Sphere(pose,radius))
    
    def setTrack(self):
        pointNum = 0
        pointHandles = []

        while pointNum < 20 :
            error, pointHandle = sim.simxGetObjectHandle(self.client_id,f'Track{pointNum}',sim.simx_opmode_blocking)
            if(error == 0):
                pointHandles.append(pointHandle)

            pointNum+=1
        for pointId in pointHandles:
            _,pose = sim.simxGetObjectPosition(self.client_id,pointId,-1,sim.simx_opmode_blocking)
            self.track.append(pose)

    def getSpheres(self):
        return self.spheres
    
    def getTack(self):
        return self.track
    