import numpy as np
import math

class Ground:
    def __init__(self):
        self.x_vals = None
        self.heights = None
        self.normals = None
        self.lines = None

    def get_height(self, x):
        dx = self.x_vals[1] - self.x_vals[0]
        i = math.floor(x / dx)
        t = (x - i*dx)/dx
        return self.heights[i]*(1-t) + self.heights[i+1]*(t)

    def get_normal(self, x):
        dx = self.x_vals[1] - self.x_vals[0]
        i = math.floor(x / dx)
        t = (x - i*dx)/dx
        return self.normals[i,:]*(1-t) + self.normals[i+1,:]*(t)

    def draw(self, ax):
        if (self.lines is None):
            (self.lines,) = ax.plot(self.x_vals, self.heights, animated=True)
        ax.draw_artist(self.lines)

    def _compute_normals(self):
        n = len(self.x_vals)
        dx = self.x_vals[1] - self.x_vals[0]
        self.normals = np.empty((n, 2))
        self.normals[:,0] = 1.0
        self.normals[1:-1,1] = 0.5*(self.heights[2:] - self.heights[:-2]) / dx
        self.normals[0,1] = (self.heights[1] - self.heights[0]) / dx
        self.normals[-1,1] = (self.heights[-1] - self.heights[-2]) / dx
        tmp = self.normals[:,0].copy()
        self.normals[:,0] = -self.normals[:,1]
        self.normals[:,1] = tmp
        for i in range(n):
            mag = math.sqrt(self.normals[i,0]**2 + self.normals[i,1]**2)
            self.normals[i,0] /= mag
            self.normals[i,1] /= mag

class Flat(Ground):
    def __init__(self, n):
        super().__init__()
        self.x_vals = np.linspace(0.0, 1.0, n)
        self.heights = np.zeros(n)
        self._compute_normals()
                        
class Ramp(Ground):
    def __init__(self, h, n):
        super().__init__()
        self.x_vals = np.linspace(0.0, 1.0, n)
        self.heights = np.zeros(n)
        self.heights[:int(n/2)+1] = np.linspace(h, 0.0, int(n/2)+1)
        self._compute_normals()

class Wedge(Ground):
    def __init__(self, h, n):
        super().__init__()
        self.x_vals = np.linspace(0.0, 1.0, n)
        self.heights = np.zeros(n)
        self.heights[:int(n/2)+1] = np.linspace(0, h, int(n/2)+1)
        self.heights[int(n/2):] = np.linspace(h, 0, int(n/2)+1)
        self._compute_normals()
