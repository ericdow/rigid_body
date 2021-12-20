import numpy as np
import matplotlib.pyplot as plt
import math
import collision

class RigidBody:
    beta = 0.3 # Baumgarte bias velocity factor
    d_slop = 0.002 # penetration depth slop
    restitution_vel_thresh = 0.01 # collisions above this velocity will have restitution
    def __init__(self, pos0, theta0):
        self.pos = pos0
        self.theta = theta0
        self.corner_pos = None
        self.corner_pos0 = None
        self.is_polygon = True
        self.lin_mom = np.array([0.0, 0.0])
        self.ang_mom = 0.0
        self.mass = 0.0
        self.I = 0.0 # moment of inertia
        self.mu = 0.0 # friction coefficient
        self.e = 0.0 # coefficient of restitution
        self.lambda_prev = None # impulses from last frame (for warm starting)
        self.lines = None
        self.prev_state = (self.pos, self.theta)
        self.curr_state = (self.pos, self.theta)

    def set_restitution(self, e):
        self.e = e

    def get_corner_pos(self):
        return self.corner_pos

    def get_state(self):
        return (self.pos, self.theta)

    def set_state(self, state):
        self.pos = state[0]
        self.theta = state[1]
        self._update_corner_pos()

    def store_prev_state(self):
        self.prev_state = (self.pos, self.theta)
    
    def store_curr_state(self):
        self.curr_state = (self.pos, self.theta)
    
    def update_position(self, dt):
        vel, omega = self.get_velocity()
        self.pos += dt * vel 
        self.theta += dt * omega

    def interpolate_state(self, alpha):
        pos0, theta0 = self.prev_state[0], self.prev_state[1]
        pos1, theta1 = self.curr_state[0], self.curr_state[1]
        self.pos = alpha*pos1 + (1.0 - alpha)*pos0
        self.theta = alpha*theta1 + (1.0 - alpha)*theta0

    def set_velocity(self, vel, omega):
        self.lin_mom = self.mass * vel
        self.ang_mom = self.I * omega

    def get_velocity(self):
        return (self.lin_mom / self.mass, self.ang_mom / self.I)
    
    def draw(self, ax):
        raise NotImplementedError('You need to define draw()!')

    def do_physics_step(self, dt, grnd):
        # compute the contact points
        corners, normals, depths = collision.collide(grnd, self)

        # bounce off of side walls
        for c in corners:
            if self.corner_pos[c][0] > 0.95 or self.corner_pos[c][0] < 0.05:
                self.lin_mom[0] *= -1.0
                break
        
        # apply forces
        self.lin_mom[1] -= self.mass * 9.81 * dt
        
        # apply impulses
        n_iter = 20
        jn = np.zeros(self.corner_pos.shape[0])
        for _ in range(n_iter):
            for i in range(len(corners)):
                c = corners[i]
                n = normals[i]
                d = depths[i]

                vel, omega = self.get_velocity()
                rc = self.corner_pos[c] - self.pos
                vc = vel + omega * np.array([-rc[1], rc[0]])
                vn = np.dot(vc, n)
                rc_cross_n = rc[0]*n[1] - rc[1]*n[0]
                kn = 1.0 / self.mass + \
                        1.0 / self.I * rc_cross_n * (n[1]*rc[0] - n[0]*rc[1])
                # only apply restitution if contact velocity is above threshold
                if (vn < RigidBody.restitution_vel_thresh):
                    v_bias = RigidBody.beta / dt * max(0.0, d - RigidBody.d_slop)
                    e = 0.0
                else:
                    v_bias = 0.0
                    e = self.e
                djn = (-vn + v_bias) / kn
                jn0 = jn[c]
                jn[c] = (1.0 + e)*max(jn[c] + djn, 0.0)
                djn = jn[c] - jn0
                self.lin_mom += djn * n
                self.ang_mom += djn * rc_cross_n

        # update position
        vel, omega = self.get_velocity()
        self.pos += dt * vel 
        self.theta += dt * omega

    def _update_corner_pos(self):
        if (not self.is_polygon):
            return
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
        super().__init__(pos0, theta0)
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
        if (self.lines is None):
            (self.lines,) = ax.plot(xs, ys, animated=True)
        else:
            self.lines.set_data(xs, ys)
            ax.draw_artist(self.lines)

class Circle(RigidBody):
    def __init__(self, radius, mass, pos0, theta0):
        super().__init__(pos0, theta0)
        self.radius = radius
        self.mass = mass
        self.I = 0.5*mass*radius*radius 
        self.is_polygon = False

    def draw(self, ax):
        coord = []
        coord.append([0.0, 0.0])
        for i in np.linspace(0, 2.0*math.pi, 100):
            coord.append([self.radius*math.cos(i), 
                self.radius*math.sin(i)])
        st = math.sin(self.theta)
        ct = math.cos(self.theta)
        for c in coord:
            c[0], c[1] = c[0]*ct - c[1]*st, c[0]*st + c[1]*ct
            c[0] += self.pos[0]
            c[1] += self.pos[1]
        xs, ys = zip(*coord)
        if (self.lines is None):
            (self.lines,) = ax.plot(xs, ys, animated=True)
        else:
            self.lines.set_data(xs, ys)
            ax.draw_artist(self.lines)

class Bar(RigidBody):
    def __init__(self, L, W, mass, pos0, theta0):
        '''
     3-------2        \
     |   y   |        |
     |   ^   |        |
     |   |   |        |
     |   |   |        |
     |   |   |        |
     |   ----|---> x  | L
     |       |        |
     |       |        |
     |       |        |
     |       |        |
     0-------1        /

     \_______/
     
         W

        '''
        super().__init__(pos0, theta0)
        self.L = L
        self.W = W
        self.mass = mass
        self.I = mass * (L**2 + W**2) / 12.0
        hL = 0.5*L
        hW = 0.5*W
        self.corner_pos0 = np.array([[-hW,-hL], [hW,-hL], [hW,hL], [-hW,hL]])
        self.corner_pos = self.corner_pos0.copy()
        self._update_corner_pos()

    def draw(self, ax):
        coord = self.corner_pos.tolist()
        coord.append(coord[0])
        xs, ys = zip(*coord)
        if (self.lines is None):
            (self.lines,) = ax.plot(xs, ys, animated=True)
        else:
            self.lines.set_data(xs, ys)
            ax.draw_artist(self.lines)

