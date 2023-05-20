import pybullet as p
import time
import pybullet_data
import numpy as np
import math


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
last_angle_array = [ 0,0,0, 0,0,0, 0,0,0, 0,0,0 ]
timestep = 1./240.


def loadRobot():    
    planeId = p.loadURDF("plane.urdf")
    cubeStartPos = [0,0,0.030328007090058885]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    urdfFlags = p.URDF_USE_SELF_COLLISION
    robotId = p.loadURDF("quadru_v1.urdf",cubeStartPos, cubeStartOrientation, 
                        # useMaximalCoordinates=1, ## New feature in Pybullet
                        # flags=p.URDF_USE_INERTIA_FROM_FILE
                        )
    return robotId

def reset():    
    p.resetSimulation()
    robotId = loadRobot()

def pwmWriteArray(targetPositions, robotId):
    targetPositions=np.radians(targetPositions)
    for i in [3,4,5,9,10,11]: targetPositions[i] = targetPositions[i]*(-1)

    p.setJointMotorControlArray(robotId, [0,1,2,3,4,5,6,7,8,9,10,11], controlMode=p.POSITION_CONTROL, targetPositions=targetPositions, forces=[1.6,1.6,1.6, 1.6,1.6,1.6, 1.6,1.6,1.6, 1.6,1.6,1.6])

def pwmWriteArray_interpolated(targetPositions, robotId, speed=400):  
    sendPositions = [ 0,0,0, 0,0,0, 0,0,0, 0,0,0 ]  
    max_angle = 0
    for i in range(len(targetPositions)):
        max_angle = max(max_angle, abs(last_angle_array[i]-targetPositions[i]))
    max_time = max_angle / speed
    num_iter = int(max_time/timestep)
    for t in range(num_iter):
        for i in range(len(targetPositions)): sendPositions[i] = int(last_angle_array[i] + (targetPositions[i]-last_angle_array[i])*t/num_iter)
        pwmWriteArray(sendPositions, robotId)
        p.stepSimulation()
        time.sleep(timestep)
        cameraFollow(robotId)
    for i in range(len(last_angle_array)): last_angle_array[i] = targetPositions[i]

def step(t=timestep):
    for i in range(int(t/timestep)):
        p.stepSimulation()
    time.sleep(t)

def cameraFollow(robotID):
    basePos, baseOrn = p.getBasePositionAndOrientation(robotID) # Get model position
    p.resetDebugVisualizerCamera( cameraDistance=1, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=basePos) # fix camera onto model

def inverse_angles(pt_x,pt_y,legID=0):
    if legID in [1,2]:
        pt_x = pt_x*(-1)

    if pt_x != 0:
        pt_angle = math.degrees(math.atan(pt_y/pt_x))
    else:
        pt_angle = 90
    iso_angle = math.degrees(math.acos(math.sqrt(pt_x**2 + pt_y**2)/20))
    theta_1 = pt_angle - iso_angle
    theta_2 = 180 - pt_angle - iso_angle
    return [0, theta_1, theta_2]

print("MMM", inverse_angles(0,14.14))   
    

    

# ##########

robotId = loadRobot()                  
# for i in range (1000):
#     p.stepSimulation()
#     time.sleep(timestep)
#     if i > 100 and i < 500:
#         pwmWriteArray_interpolated([0,45,90, 0,-45,-90, 0,45,90, 0,-45,-90], robotId, speed=10)
#     if i==500:
#         print('Poyoyoyoyo')
#         # p.resetSimulation()
#         # robotId = loadRobot()

pwmWriteArray_interpolated([0,45,90, 0,45,90, 0,45,90, 0,45,90], robotId)
step(1)

# while True:
    # On-Spot Trot
    # pwmWriteArray_interpolated([0,30,60, 0,45,90, 0,30,60, 0,45,90], robotId)
    # pwmWriteArray_interpolated([0,45,90, 0,30,60, 0,45,90, 0,30,60], robotId)

    # Push-Up
    # pwmWriteArray_interpolated([0,90,180, 0,0,180, 0,0,180, 0,90,180], robotId)
    # pwmWriteArray_interpolated([0,10,20, 0,0,180, 0,0,180, 0,10,20], robotId)

    # pwmWriteArray_interpolated([0,45,90, 0,45,90, 0,45,90, 0,45,90], robotId)




# ##########

p.disconnect()

