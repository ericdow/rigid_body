import numpy as np
import math

class RigidBody:
    def __init__(self):
        self.pos = None
        self.theta = None
        self.corner_pos = None
        self.corner_pos0 = None
        self.mu = 0.0 # friction coefficient
        self.e = 0.0 # coefficient of restitution
        self.lambda_prev = None # impulses from last frame (for warm starting)

    def get_corner_pos(self):
        return self.corner_pos

    def get_state(self):
        return [self.pos, self.theta]

    def set_state(self, pos, theta):
        self.pos = pos
        self.theta = theta
        self._update_corner_pos()
    
    def draw(self, ax):
        raise NotImplementedError('You need to define draw()!')

    def _update_corner_pos(self):
        # apply rotation
        st = math.sin(self.theta)
        ct = math.cos(self.theta)
        for i in range(len(self.corner_pos0)):
            c = self.corner_pos[i]
            c[0] = self.corner_pos0[i,0]*ct - self.corner_pos0[i,1]*st
            c[1] = self.corner_pos0[i,0]*st + self.corner_pos0[i,1]*ct
        # apply translation
        for c in self.corner_pos:
            c += self.pos

class Box(RigidBody):
    def __init__(self, size, pos0, theta0):
        '''
         y
         ^ 
         |
     3---|---2        \
     |   |   |        |
     |   ----|---> x  | size
     |       |        |
     0-------1        /

        '''
        self.pos = pos0
        self.theta = theta0
        self.size = size
        hs = 0.5*size
        self.corner_pos0 = np.array([[-hs,-hs], [hs,-hs], [hs,hs], [-hs,hs]])
        self.corner_pos = self.corner_pos0.copy()
        self._update_corner_pos()

    def draw(self, ax):
        coord = self.corner_pos.tolist()
        coord.append(coord[0])
        xs, ys = zip(*coord)
        ax.plot(xs,ys)
