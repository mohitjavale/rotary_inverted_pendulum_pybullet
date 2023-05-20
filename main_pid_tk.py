import time
from matplotlib import pyplot as plt
from bullet import RILSim
from tkApp import TkApp
import threading

bullet = RILSim()
bullet.loadRobot()


time.sleep(1)
plt.ion()
fig, ax = plt.subplots()

tk_my = TkApp()
threading.Thread(target=tk_my.startApp).start()

e = 0
t = 0
t_arr = []
e_arr = []



while True:    
    start_time = time.perf_counter()
    last_e = e
    e = bullet.getPendulumAngle()
    tk_my.i += e
    d = e - last_e    
    target_pid = tk_my.kp*e + tk_my.ki*tk_my.i + tk_my.kd*d
    print(f'p={e:.2f}, i={tk_my.i:.2f}, d={d:.2f}, target_pid={target_pid:.2f}')   

    bullet.setRotorVelocity(-target_pid)

    end_time = time.perf_counter()
    t += end_time - start_time

    t_arr.append(t)
    e_arr.append(e)
    ax.clear()
    ax.plot(t_arr[-20:], e_arr[-20:], color='k')
    ax.axhline(y = 0, color = 'r', linestyle = ':')
    ax.set_ylim(-180, 180)
    fig.canvas.flush_events()
    
    continue