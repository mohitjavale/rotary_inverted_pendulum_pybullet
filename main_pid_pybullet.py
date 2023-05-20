import pybullet as p
import time
import pybullet_data
import math
from matplotlib import pyplot as plt
import tkinter as tk
from tkinter import *

from bullet import RILSim

bullet = RILSim()
bullet.loadRobot()

kp = 0.013
ki = 0.0
kd = 0.0
i = 0
e = 0
time.sleep(1)
plt.ion()
fig, ax = plt.subplots()


t = 0
t_arr = []
e_arr = []

kp_id = p.addUserDebugParameter('kp', 0, 0.02, 0)
ki_id = p.addUserDebugParameter('ki', 0, 0.02, 0)
kd_id = p.addUserDebugParameter('kd', 0, 0.02, 0)


while True:
    kp = p.readUserDebugParameter(kp_id)
    ki = p.readUserDebugParameter(ki_id)
    kd = p.readUserDebugParameter(kd_id)
    t += 1
    # print(bullet.getPendulumAngle())
    last_e = e
    e = bullet.getPendulumAngle()
    i += e
    d = e - last_e    
    target_pid = kp*e + ki*i + kd*d
    print(target_pid)
    # p.setJointMotorControl2(bullet.robotId, 0, controlMode=p.POSITION_CONTROL, targetPosition=target_pid)target_pid
    bullet.setRotorAngle(target_pid)
    t_arr.append(t)
    e_arr.append(e)
    ax.clear()
    ax.plot(t_arr[-20:], e_arr[-20:], color='k')
    fig.canvas.flush_events()
    continue