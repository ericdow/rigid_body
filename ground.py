import numpy as np

class Ground:
    def __init__(self):
        self.x_vals = None
        self.heights = None
        self.normals = None

    def get_height(self, x):
        # TODO
        return 1

    def get_normal(self, x):
        # TODO
        return 1

    def draw(self, ax):
        ax.plot(self.x_vals, self.heights)

class Flat(Ground):
    def __init__(self, n):
        self.x_vals = np.linspace(0.0, 1.0, n)
        self.heights = np.zeros(n)
        self.normals = np.tile([0.0,1.0], n)
                        
