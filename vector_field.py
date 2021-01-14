from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
from numpy import arange
from roblib import *

fig = plt.figure()
ax = fig.gca(projection='3d')
wpt = np.array([[-2., -5., 2.]]).T
wpt_f = wpt.flatten()
x, y, z = np.meshgrid(np.arange(-5, 5, 2),
                      np.arange(-5, 5, 2),
                      np.arange(-5, 5, 2))

u = (wpt_f[0] - x)
v = (wpt_f[1] - y)
w = (wpt_f[2] - z) 
ax.quiver(x, y, z, u, v, w, length=0.1)

dt = 0.05


pos = np.array([[5., 5., -3.]]).T

for t in arange(0, 1000, 1):
    cla()
    ax.quiver(x, y, z, u, v, w, length=0.1)
    force = wpt - pos
    pos += 1*force*dt
    ax.scatter(pos[0, 0], pos[1, 0], pos[2, 0], c='red')
    pause(0.001)

