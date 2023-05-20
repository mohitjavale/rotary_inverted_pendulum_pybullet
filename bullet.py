import pybullet as p
import time
import pybullet_data
import math

class RILSim():
    def __init__(self):
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        self.timestep = 1./240.
        p.setRealTimeSimulation(1)

    # def loadGroundPlane(self):
    #     self.planeId = p.loadURDF("plane.urdf")

    def loadRobot(self):
        cubeStartPos = [0,0,0]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        # urdfFlags = p.URDF_USE_SELF_COLLISION
        self.robotId = p.loadURDF("description/rotary_pendulum_v5.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1,
                                # useMaximalCoordinates=1, ## New feature in Pybullet
                                flags=p.URDF_USE_INERTIA_FROM_FILE)
        p.setJointMotorControl2(self.robotId, 1, controlMode=p.VELOCITY_CONTROL, force=0)
    
    def reset(self):
        p.resetSimulation()
        # self.loadGroundPlane()
        self.loadRobot()
        p.setRealTimeSimulation(1)

    def step(self, t=1./240.):
        for i in range(int(t/self.timestep)):
            p.stepSimulation()
        time.sleep(t)

    def cameraFollow(self):
        basePos, baseOrn = p.getBasePositionAndOrientation(
            self.robotId)  # Get model position
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=75, cameraPitch=-20,
                                     cameraTargetPosition=basePos)  # fix camera onto model
    
    def getPendulumAngle(self):
        pendulum_angle = math.degrees(p.getJointState(self.robotId, 1)[0])%360
        if pendulum_angle>180: pendulum_angle = pendulum_angle - 360
        return pendulum_angle

    def getPendulumVelocity(self):
        pendulum_velocity = math.degrees(p.getJointState(self.robotId, 1)[1])
        return pendulum_velocity

    def setRotorAngle(self, target_angle):
        p.setJointMotorControl2(self.robotId, 0, controlMode=p.POSITION_CONTROL, targetPosition=target_angle)

    def setRotorVelocity(self, target_vel):
        p.setJointMotorControl2(self.robotId, 0, controlMode=p.VELOCITY_CONTROL, targetVelocity=target_vel)

    # Doesn't work
    def setRotorTorque(self, target_torque):
        p.setJointMotorControl2(self.robotId, 0, controlMode=p.TORQUE_CONTROL, force=target_torque)



# p.setJointMotorControl2(robotId, 1, controlMode=p.VELOCITY_CONTROL, force=0)
# for i in range (10000):
#     time.sleep(1./240.)
#     pendulum_angle = math.degrees(p.getJointState(robotId, 1)[0])%360
#     if pendulum_angle>180: pendulum_angle = pendulum_angle - 360
#     print(pendulum_angle)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
# print(cubePos,cubeOrn)
# p.disconnect()

