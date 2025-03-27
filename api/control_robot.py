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

import time
import sys

print("Program started")
sim.simxFinish(-1)

clientID=sim.simxStart('127.0.0.1',15000,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
    sys.exit("Failed to connect")

time.sleep(1)    

errorcode, left_motor_handle = sim.simxGetObjectHandle(clientID,'./PioneerP3DX/leftMotor',sim.simx_opmode_oneshot_wait)
errorcode, right_motor_handle = sim.simxGetObjectHandle(clientID,'./PioneerP3DX/rightMotor',sim.simx_opmode_oneshot_wait)

errorcode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.2, sim.simx_opmode_oneshot_wait)
errorcode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.2, sim.simx_opmode_oneshot_wait)
   
print ('Program ended')