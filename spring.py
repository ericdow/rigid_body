import numpy as np
import math

class Spring:
    '''
    . (x0, y0) \
    >          |
    <          |
    >          | L0
    <          |
    >          |
    . (x1, y1) /

    '''
    def __init__(self, xy0, xy1, L0, k, c, w):
        self.xy0 = xy0
        self.xy1 = xy1
        self.L0 = L0
        self.k = k
        self.c = c
        self.w = w # width for drawing
        self.lines = None

    def set_position(self, xy0, xy1):
        self.xy0 = xy0
        self.xy1 = xy1

    def force(self, vel):
        v = self.xy1 - self.xy0
        L = math.sqrt(v[0]**2 + v[1]**2)
        mag = self.k*(L - self.L0) - self.c*vel
        v /= L
        f = [v*mag, -v*mag]
        return f
    
    def draw(self, ax):
        n = self.xy1 - self.xy0
        nl = math.sqrt(n[0]**2 + n[1]**2)
        n /= nl
        coord = []
        coord.append([self.xy0[0], self.xy0[1]])
        s = 10
        ds = nl / s
        for i in range(1,s+1):
            coord.append([self.xy0[0] + i*n[0]*ds, self.xy0[1] + i*n[1]*ds])
        n[0],n[1] = n[1],n[0]
        dir = 1.0
        for i in range(1,s):
            coord[i][0] += 0.5*self.w*dir*n[0]
            coord[i][1] += 0.5*self.w*dir*n[1]
            dir *= -1.0
        xs, ys = zip(*coord)
        if (self.lines is None):
            (self.lines,) = ax.plot(xs, ys, animated=True)
        else:
            self.lines.set_data(xs, ys)
            ax.draw_artist(self.lines)


