import pybullet as p
import time
import pybullet_data
import numpy as np
import math


class quadru_control:
    def __init__(self, bullet):
        self.bullet = bullet
        self.last_angle_array = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.robotId = self.bullet.robotId
        self.timestep = self.bullet.timestep

    def pwmWriteArray(self, targetPositions):
        targetPositions = np.radians(targetPositions)
        for i in [3, 4, 5, 9, 10, 11]:
            targetPositions[i] = targetPositions[i]*(-1)
        p.setJointMotorControlArray(self.robotId, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], controlMode=p.POSITION_CONTROL,
                                    targetPositions=targetPositions, forces=[1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6])

    def pwmWriteArray_interpolated(self, targetPositions, speed=400, camera_follow=True):
        sendPositions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        max_angle = 0
        for i in range(len(targetPositions)):
            max_angle = max(max_angle, abs(
                self.last_angle_array[i]-targetPositions[i]))
        max_time = max_angle / speed
        num_iter = int(max_time/self.timestep)
        for t in range(num_iter):
            for i in range(len(targetPositions)):
                sendPositions[i] = int(
                    self.last_angle_array[i] + (targetPositions[i]-self.last_angle_array[i])*t/num_iter)
            self.pwmWriteArray(sendPositions)
            p.stepSimulation()
            time.sleep(self.timestep)
            if camera_follow==True:
                self.bullet.cameraFollow()
        for i in range(len(self.last_angle_array)):
            self.last_angle_array[i] = targetPositions[i]

    def stand(self):
        self.pwmWriteArray_interpolated([0, 45, 90, 0, 45, 90, 0, 45, 90, 0, 45, 90])

    def fold(self):
        self.pwmWriteArray_interpolated([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def inverse_angles(self, pt_x, pt_y, legId=0):
        if legId in [1, 2]:
            pt_x = pt_x*(-1)

        if pt_x != 0:
            pt_angle = math.degrees(math.atan(pt_y/pt_x))
            if pt_angle<0:
                pt_angle = pt_angle+180
        else:
            pt_angle = 90
        iso_angle = math.degrees(math.acos(math.sqrt(pt_x**2 + pt_y**2)/20))
        theta_1 = pt_angle - iso_angle
        theta_2 = 180 - 2*iso_angle
        return [0, theta_1, theta_2]

