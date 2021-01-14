import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class ConeVectorField:
    def __init__(self, O=np.array([[0], [0], [0]]), alpha=np.pi/4, h=10):
        self.O = O
        self.n = np.array([[0], [0], [1]])
        self.alpha = alpha
        self.h = h

        self.fig = plt.figure()
        self.axes1 = self.fig.add_subplot(111, projection='3d')
        self.axes1.set_xlabel("x")
        self.axes1.set_ylabel("y")
        self.axes1.set_zlabel("z")

    def __contains__(self, p):
        if np.linalg.norm(p - self.O) > 0.05:
            angle = np.arccos(self.n.T @ (p - self.O) / np.linalg.norm(p - self.O))
        else:
            angle = 0
        return 0 <= p[2, 0] - self.O[2, 0] <= self.h and np.abs(angle) <= self.alpha/2

    def vector(self, p):
        v = p - self.O
        theta = np.arctan(-v[2, 0])
        d = 0.5 * np.sqrt(v[0, 0]**2 + v[1, 0]**2)
        angle = np.arctan2(v[1, 0], v[0, 0])
        return -d*np.cos(angle), -d*np.sin(angle), theta
    
    def draw(self):
        z = self.O[2, 0] + np.arange(0, self.h, 0.5)
        theta = np.arange(0, 2 * np.pi + np.pi / 50, np.pi / 50)

        for zval in z:
            x = self.O[0, 0] + (zval - self.O[2, 0]) * np.tan(self.alpha / 2) * np.array([np.cos(q) for q in theta])
            y = self.O[1, 0] + (zval - self.O[2, 0]) * np.tan(self.alpha / 2) * np.array([np.sin(q) for q in theta])
            self.axes1.plot(x, y, zval, 'purple')

    def draw_point(self, p):
        if p in self:
            col="teal"
            vx, vy, vz = self.vector(p)
            self.axes1.quiver(p[0, 0], p[1, 0], p[2, 0], vx, vy, vz)
            self.axes1.scatter3D(p[0, 0], p[1, 0], p[2, 0], color=col)
        else:
            col="crimson"


if __name__ == "__main__":
    c = Cone(O = np.array([[1], [2], [1]]))

    p1 = np.array([[0], [0], [5]])
    p2 = np.array([[1], [0.5], [8]])
    p3 = np.array([[2], [3], [1]])
    p4 = np.array([[2], [2], [3.5]])
    p5 = np.array([[1], [2], [10.5]])
    p6 = np.array([[-0.5], [1], [11.5]])
    
    c.draw()
    c.draw_point(p1)
    c.draw_point(p2)
    c.draw_point(p3)
    c.draw_point(p4)
    c.draw_point(p5)
    c.draw_point(p6)
    plt.show()