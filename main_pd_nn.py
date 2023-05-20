import time
from matplotlib import pyplot as plt
from bullet import RILSim
from tkApp import TkApp
import threading

import torch
import torch.nn as nn

bullet = RILSim()
bullet.loadRobot()


time.sleep(1)
plt.ion()
fig, ax = plt.subplots()


class PDNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.stack = nn.Sequential(
            nn.Linear(2, 1)
        )

    def forward(self, x):
        out = self.stack(x)        
        return out

p = 0
# i = 0
t = 0
t_arr = []
p_arr = []


# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device = torch.device('cpu')
pd_net = PDNet().to(device)
optimizer = torch.optim.Adam(pd_net.parameters(), lr = 0.1) 


while True:    
    start_time = time.perf_counter()
    last_p = p
    p = bullet.getPendulumAngle()
    # i += e
    d = p - last_p    

    target_pid = pd_net(torch.tensor([p, d]))
    bullet.setRotorAngle(target_pid.item())
    loss = torch.tensor(abs(p), requires_grad = True)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

    # print(f'pd output = {target_pid.item()}, loss = {loss.item()}')
    print(list(pd_net.parameters()))

    end_time = time.perf_counter()
    t += end_time - start_time
    t_arr.append(t)
    p_arr.append(p)
    ax.clear()
    ax.plot(t_arr[-20:], p_arr[-20:], color='k')
    ax.set_ylim(-50, 50)
    fig.canvas.flush_events()

    # break