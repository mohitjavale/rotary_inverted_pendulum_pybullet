import pybullet as p
import time
import pybullet_data
import numpy as np
import math


class bulletSim:

    def __init__(self):
        # or p.DIRECT for non-graphical version
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -10)
        self.timestep = 1./240.
        # p.setRealTimeSimulation(1)

    def loadGroundPlane(self):
        self.planeId = p.loadURDF("plane.urdf")

    def loadRobot(self):
        cubeStartPos = [0, 0, 0.030328007090058885]
        cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        urdfFlags = p.URDF_USE_SELF_COLLISION
        self.robotId = p.loadURDF("quadru_v1.urdf", cubeStartPos, cubeStartOrientation,
                                  # useMaximalCoordinates=1, ## New feature in Pybullet
                                  # flags=p.URDF_USE_INERTIA_FROM_FILE
                                  )

    def reset(self):
        p.resetSimulation()
        self.loadGroundPlane()
        self.loadRobot()

    def step(self, t):
        for i in range(int(t/self.timestep)):
            p.stepSimulation()
        time.sleep(t)

    def cameraFollow(self):
        basePos, baseOrn = p.getBasePositionAndOrientation(
            self.robotId)  # Get model position
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=75, cameraPitch=-20,
                                     cameraTargetPosition=basePos)  # fix camera onto model
        

# testSim = bulletSim()
# testSim.loadGroundPlane()
# testSim.loadRobot()
