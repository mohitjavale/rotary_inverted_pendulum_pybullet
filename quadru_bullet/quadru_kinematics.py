import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import cos, sin, tan, acos, asin, atan

# ##########

def Rx(theta):
    return np.array([[1, 0, 0, 0],
                     [0, cos(theta), -sin(theta), 0],
                     [0, sin(theta), cos(theta), 0],
                     [0, 0, 0, 1]])

def Ry(theta):
    return np.array([[cos(theta), 0, -sin(theta), 0],
                     [0, 1, 0, 0],
                     [sin(theta), 0, cos(theta), 0],
                     [0, 0, 0, 1]])

def Rz(theta):
    return np.array([[cos(theta), -sin(theta), 0, 0],
                     [sin(theta), cos(theta), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def T(x,y,z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

# ##########

def plt_scatter(pt, ax, color='b', style='.'):
    ax.scatter(pt[0], pt[1], pt[2], c=color, marker=style)

def plt_quiver(pt1, pt2, ax, color='b'):
    ax.quiver(pt1[0], pt1[1], pt1[2], pt2[0]-pt1[0], pt2[1]-pt1[1], pt2[2]-pt1[2], color=color, arrow_length_ratio = 0.1)

def plt_plot(pt1, pt2, ax, color='b'):
    plt.plot([pt1[0],pt2[0]], [pt1[1],pt2[1]], [pt1[2],pt2[2]], color=color)

# ##########
# plt.ion()
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')


plt.plot([0,1], [0,0], [0,0], color='black')
plt.plot([0,0], [0,1], [0,0], color='black')
plt.plot([0,0], [0,0], [0,1], color='black')

joint_angles = np.radians([0,-30,45, 0,-30,45, 0,-45,90, 0,-45,90])


# def plt_qudru(joint_angles):

o = np.array([0,0,0,1]).T

# plt_scatter(o, ax, color='black')

fl_base = T(85, 43.6, -13) @ o
fl_1 = T(85, 43.6, -13) @ Rx(joint_angles[0]) @ T(0, 56.5, 10.5) @ o
fl_2 = T(85, 43.6, -13) @ Rx(joint_angles[0]) @ T(0, 56.5, 10.5) @ Ry(joint_angles[1]) @ T(100, 0, 0) @ o
fl_3 = T(85, 43.6, -13) @ Rx(joint_angles[0]) @ T(0, 56.5, 10.5) @ Ry(joint_angles[1]) @ T(100, 0, 0) @ Ry(np.radians(180)) @ Ry(joint_angles[2]) @ T(100, 0, 0) @ o

bl_base = T(-85, 43.6, -13) @ o
bl_1 = T(-85, 43.6, -13) @ Rx(joint_angles[3]) @ T(0, 56.5, 10.5) @ o
bl_2 = T(-85, 43.6, -13) @ Rx(joint_angles[3]) @ T(0, 56.5, 10.5) @ Ry(joint_angles[4]) @ T(100, 0, 0) @ o
bl_3 = T(-85, 43.6, -13) @ Rx(joint_angles[3]) @ T(0, 56.5, 10.5) @ Ry(joint_angles[4]) @ T(100, 0, 0) @ Ry(np.radians(180)) @ Ry(joint_angles[5]) @ T(100, 0, 0) @ o

br_base = T(-85, -43.6, -13) @ o
br_1 = T(-85, -43.6, -13) @ Rx(joint_angles[6]) @ T(0, -56.5, 10.5) @ o
br_2 = T(-85, -43.6, -13) @ Rx(joint_angles[6]) @ T(0, -56.5, 10.5) @ Ry(joint_angles[7]) @ T(100, 0, 0) @ o
br_3 = T(-85, -43.6, -13) @ Rx(joint_angles[6]) @ T(0, -56.5, 10.5) @ Ry(joint_angles[7]) @ T(100, 0, 0) @ Ry(np.radians(180)) @ Ry(joint_angles[8]) @ T(100, 0, 0) @ o

fr_base = T(85, -43.6, -13) @ o
fr_1 = T(85, -43.6, -13) @ Rx(joint_angles[9]) @ T(0, -56.5, 10.5) @ o
fr_2 = T(85, -43.6, -13) @ Rx(joint_angles[9]) @ T(0, -56.5, 10.5) @ Ry(joint_angles[10]) @ T(100, 0, 0) @ o
fr_3 = T(85, -43.6, -13) @ Rx(joint_angles[9]) @ T(0, -56.5, 10.5) @ Ry(joint_angles[10]) @ T(100, 0, 0) @ Ry(np.radians(180)) @ Ry(joint_angles[11]) @ T(100, 0, 0) @ o

plt_plot(fl_base, bl_base, ax, color='g')
plt_plot(bl_base, br_base, ax, color='g')
plt_plot(br_base, fr_base, ax, color='g')
plt_plot(fr_base, fl_base, ax, color='g')



plt_plot(fl_base, fl_1, ax, color='r')
plt_plot(fl_1, fl_2, ax, color='r')
plt_plot(fl_2, fl_3, ax, color='r')

plt_plot(bl_base, bl_1, ax, color='r')
plt_plot(bl_1, bl_2, ax, color='r')
plt_plot(bl_2, bl_3, ax, color='r')

plt_plot(br_base, br_1, ax, color='r')
plt_plot(br_1, br_2, ax, color='r')
plt_plot(br_2, br_3, ax, color='r')

plt_plot(fr_base, fr_1, ax, color='r')
plt_plot(fr_1, fr_2, ax, color='r')
plt_plot(fr_2, fr_3, ax, color='r')

ax.set_aspect('equal')

# plt_qudru(joint_angles)
plt.show()
# fig.canvas.draw()
# fig.canvas.flush_events()
# plt.pause(1)
# plt.show()