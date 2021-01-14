#!/usr/bin/env python
import numpy as np

class PlaneVectorField:
    def __init__(self, h=10, r0=np.array([0, 0, 10])):
        self.h = h
        self.r0 = r0

    def distance(self, v1, v2, u):
        u = np.array(u, ndmin=2)
        v = np.vstack((v1, v2))
        vv = np.dot(v, v.T)
        uv = np.dot(u, v.T)
        ab = np.dot(np.linalg.inv(vv), uv.T)
        w = u - np.dot(ab.T, v)
        return np.sqrt(np.sum(w**2, axis=1))

    def vector(self, p):
        e = np.sign(self.h-p.flatten()[2]) * self.distance(np.array([1, 0, 0]), np.array([0, 1, 0]), p-self.r0)
        theta = np.arctan(e)
        d = np.sqrt((p.flatten()[0] - self.r0[0])**2 + (p.flatten()[1] - self.r0[1])**2)
        angle = np.arctan2(p.flatten()[1] -self.r0[1], p.flatten()[0] - self.r0[0])
        return -d*np.sin(angle), -d*np.cos(angle), theta[0]


if __name__ == '__main__':
    p = PlaneVectorField()
    print(p.vector(np.array([0, 0, 9])))