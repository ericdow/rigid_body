import numpy as np
import math
import collision

class RigidBody:
    beta = 0.2 # Baumgarte bias velocity factor
    d_slop = 0.01 # penetration depth slop
    def __init__(self):
        self.pos = None
        self.theta = None
        self.corner_pos = None
        self.corner_pos0 = None
        self.lin_mom = np.array([0.0, 0.0])
        self.ang_mom = 0.0
        self.mass = 0.0
        self.I = 0.0 # moment of inertia
        self.mu = 0.0 # friction coefficient
        self.e = 0.0 # coefficient of restitution
        self.lambda_prev = None # impulses from last frame (for warm starting)

    def get_corner_pos(self):
        return self.corner_pos

    def get_state(self):
        return (self.pos, self.theta)

    def set_state(self, state):
        self.pos = state[0]
        self.theta = state[1]
        self._update_corner_pos()

    def interpolate_state(self, s0, s1, alpha):
        pos0, theta0 = s0[0], s0[1]
        pos1, theta1 = s1[0], s1[1]
        pos = alpha*pos1 + (1.0 - alpha)*pos0
        theta = alpha*theta1 + (1.0 - alpha)*theta0
        return (pos, theta)

    def get_velocity(self):
        return (self.lin_mom / self.mass, self.ang_mom / self.I)
    
    def draw(self, ax):
        raise NotImplementedError('You need to define draw()!')

    def do_physics_step(self, dt, grnd):
        # compute the contact points
        corners, normals, depths = collision.collide(grnd, self)
        
        # apply forces
        self.lin_mom[1] -= self.mass * 9.81 * dt
        
        # apply impulses
        n_iter = 50
        jn = np.zeros(self.corner_pos.shape[0])
        for _ in range(n_iter):
            for i in range(len(corners)):
                c = corners[i]
                n = normals[i]
                d = depths[i]

                vel, omega = self.get_velocity()
                rc = self.corner_pos[c] - self.pos
                v = vel + omega * np.array([-rc[1], rc[0]])
                vn = np.dot(v, n)
                rc_cross_n = rc[0]*n[1] - rc[1]*n[0]
                kn = 1.0 / self.mass + \
                        1.0 / self.I * rc_cross_n * (n[1]*rc[0] - n[0]*rc[1])
                v_bias = RigidBody.beta / dt * max(0.0, d - RigidBody.d_slop)
                djn = (-vn + v_bias) / kn
                jn0 = jn[c]
                jn[c] = max(jn[c] + djn, 0.0)
                djn = jn[c] - jn0
                self.lin_mom += djn * n
                self.ang_mom += djn * rc_cross_n

        # update position
        vel, omega = self.get_velocity()
        self.pos += dt * vel 
        self.theta += dt * omega

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
    def __init__(self, size, mass, pos0, theta0):
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
        super().__init__()
        self.pos = pos0
        self.theta = theta0
        self.size = size
        self.mass = mass
        self.I = mass * size**2 / 6.0
        hs = 0.5*size
        self.corner_pos0 = np.array([[-hs,-hs], [hs,-hs], [hs,hs], [-hs,hs]])
        self.corner_pos = self.corner_pos0.copy()
        self._update_corner_pos()

    def draw(self, ax):
        coord = self.corner_pos.tolist()
        coord.append(coord[0])
        xs, ys = zip(*coord)
        ax.plot(xs,ys)
