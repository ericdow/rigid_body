import numpy as np

class RigidBody:
    def __init__(self):
        self.pos = None
        self.theta = None
        self.corner_pos = None
        self.mu = 0.0 # friction coefficient
        self.e = 0.0 # coefficient of restitution
        self.lambda_prev = None # impulses from last frame (for warm starting)

    def get_corner_pos(self):
        return self.corner_pos
    
    def draw(self, ax):
        raise NotImplementedError('You need to define draw()!')

    def __update_corner_pos(self):
        raise NotImplementedError('You need to define __update_corner_pos()!')

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
        self.corner_pos = np.zeros((4, 2))
        self.__update_corner_pos()

    def draw(self, ax):
        coord = self.corner_pos.tolist()
        coord.append(coord[0])
        xs, ys = zip(*coord)
        ax.plot(xs,ys)

    def __update_corner_pos(self):
        # TODO
        self.corner_pos[0,:] = [0, 0]
        self.corner_pos[1,:] = [0.5, 0]
        self.corner_pos[2,:] = [0.5, 0.5]
        self.corner_pos[3,:] = [0.0, 0.5]
