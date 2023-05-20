import pybullet as p
import time
import pybullet_data
import numpy as np
import math

from bullet import bulletSim
from quadru_control import quadru_control

bullet = bulletSim()
bullet.loadGroundPlane()
bullet.loadRobot()

c = quadru_control(bullet)
angles = [0,45,90, 0,45,90, 0,45,90, 0,45,90]
c.pwmWriteArray_interpolated(angles)

step_half_length_id = p.addUserDebugParameter('step_half_length', 0, 14.14, 3.5)
step_height_id = p.addUserDebugParameter('step_height', 0, 14.14, 1.5)

while True:
    step_half_length = p.readUserDebugParameter(step_half_length_id)
    base_height = 14.14
    step_height = p.readUserDebugParameter(step_height_id)
    step_y = base_height-step_height

    angles[0:3] = c.inverse_angles(step_half_length,step_y,0)
    angles[3:6] = c.inverse_angles(-step_half_length,step_y,1)
    angles[6:9] = c.inverse_angles(step_half_length,base_height,2)
    angles[9:12] = c.inverse_angles(-step_half_length,base_height,3)    
    c.pwmWriteArray_interpolated(angles)
    print("1)", angles)

    angles[0:3] = c.inverse_angles(step_half_length,base_height,0)
    angles[3:6] = c.inverse_angles(step_half_length,step_y,1)
    angles[6:9] = c.inverse_angles(-step_half_length,base_height,2)
    angles[9:12] = c.inverse_angles(-step_half_length,step_y,3)    
    c.pwmWriteArray_interpolated(angles)
    print("2)", angles)

    angles[0:3] = c.inverse_angles(-step_half_length,base_height,0)
    angles[3:6] = c.inverse_angles(step_half_length,base_height,1)
    angles[6:9] = c.inverse_angles(-step_half_length,step_y,2)
    angles[9:12] = c.inverse_angles(step_half_length,step_y,3)    
    c.pwmWriteArray_interpolated(angles)
    print("3)", angles)

    angles[0:3] = c.inverse_angles(-step_half_length,step_y,0)
    angles[3:6] = c.inverse_angles(-step_half_length,base_height,1)
    angles[6:9] = c.inverse_angles(step_half_length,step_y,2)
    angles[9:12] = c.inverse_angles(step_half_length,base_height,3)    
    c.pwmWriteArray_interpolated(angles)
    print("4)", angles)

    


