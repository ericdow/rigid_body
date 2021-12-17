import matplotlib.pyplot as plt
import numpy as np
import time
import math
import ground
import rigid_body
import spring
import collision

# socket motion function
def xs(t):
    return 0.1 + 0.4*(1.0 - math.cos(t))

# socket velocity function
def vs(t):
    return 0.4*math.sin(t)

def compute_f_ext(bar, wheel, spring):
    f_sp = spring.force(bar.get_velocity()[0][1])[1]
    f = np.zeros([6,1])
    f[0] = f_sp[0]
    f[1] = f_sp[1] - bar.mass*9.81
    f[4] = -wheel.mass*9.81
    return f

def do_physics_step(bar, wheel, spring, grnd, M_inv, t, dt):
    # form initial velocity
    V0 = np.zeros([6,1])
    uvb, omb = bar.get_velocity()
    uvw, omw = wheel.get_velocity()
    V0[0], V0[1], V0[2] = uvb[0], uvb[1], omb
    V0[3], V0[4], V0[5] = uvw[0], uvw[1], omw

    # compute external forces
    f_ext = compute_f_ext(bar, wheel, spring)

    # check for contact between wheel and ground
    nc = 4 # number of constraints
    _, nwg, dwg = collision.collide(grnd, wheel)
    # if (nwg):
    #     nc += 2

    # form J V1 forcing
    beta = 0.2
    JV1 = np.zeros([nc,1])
    JV1[0] = vs(t + dt)
    JV1[2] = -beta*(wheel.pos[0] - (bar.pos[0] + 0.5*bar.L*math.sin(bar.theta)))
    JV1[3] = -beta*(wheel.pos[1] - (bar.pos[1] - 0.5*bar.L*math.cos(bar.theta)))

    # compute the Jacobian
    J = np.zeros([nc,6])
    # constraint on bar x position and orientation
    J[0,0] = 1.0
    J[1,2] = 1.0
    # constraint on wheel/bar relative position
    J[2,0] = -1.0
    J[2,2] = -0.5*bar.L*math.cos(bar.theta)
    J[2,3] = 1.0
    J[3,1] = -1.0
    J[3,2] = -0.5*bar.L*math.sin(bar.theta)
    J[3,4] = 1.0

    # form the system
    A = J.dot(M_inv.dot(J.transpose()))
    b = JV1 - J.dot(V0 + dt*M_inv.dot(f_ext))
    l = np.linalg.solve(A, b)

    # compute the new velocity
    V1 = V0 + dt*M_inv.dot(f_ext) + M_inv.dot(J.transpose().dot(l))
    uvb[0], uvb[1], omb = V1[0], V1[1], V1[2]
    uvw[0], uvw[1], omw = V1[3], V1[4], V1[5]
    bar.set_velocity(uvb, omb)
    wheel.set_velocity(uvw, omw)

    # update the positions
    bar.update_position(dt)
    wheel.update_position(dt)

# setup the ground and rigid bodies
grnd = ground.Wedge(0.1, 21)
bar = rigid_body.Bar(0.5, 0.1, 1.0, np.array([0.1,0.5]), 0)
wheel = rigid_body.Circle(0.15, 1.0, np.array([0.1,0.25]), 0)
spring = spring.Spring(np.array([0.1,0.75]), np.array([0.1,0.5]), 0.25, 100.0, 1.0, 0.15)

# form the inverse mass matrix
M_inv = np.zeros([6,6])
M_inv[0,0] = 1.0 / bar.mass
M_inv[1,1] = 1.0 / bar.mass
M_inv[2,2] = 1.0 / bar.I
M_inv[3,3] = 1.0 / wheel.mass
M_inv[4,4] = 1.0 / wheel.mass
M_inv[5,5] = 1.0 / wheel.I

# to run GUI event loop
fig, ax = plt.subplots()
ax.set(xlim=(0.0, 1.0), ylim=(0.0, 1.0))
ax.set_aspect('equal', 'box')
plt.show(block=False)
plt.pause(0.1)
grnd.draw(ax)
bar.draw(ax)
wheel.draw(ax)
spring.draw(ax)
bg = fig.canvas.copy_from_bbox(fig.bbox)
fig.canvas.blit(fig.bbox)

last_loop_time = time.time()
draw_wait_time = 0.0
t_physics = 0.0
dt_physics = 1.0 / 60.0 # don't make this too big or small
loop_lock_time = 1.0/120.0
last_draw_time = 0.0
t_accumulator = 0.0
current_state = bar.get_state()
previous_state = bar.get_state()
while True:
    # compute loop time
    current_loop_time = time.time()
    dt_loop = current_loop_time - last_loop_time
    last_loop_time = current_loop_time
    t_accumulator += min(dt_loop, 0.25)
    
    # update the rigid body state
    bar.set_state(current_state) # restore state after rendering
    while (t_accumulator >= dt_physics):
        previous_state = current_state
        # update the spring location
        spring.set_position(np.array([xs(t_physics), spring.xy0[1]]), 
                bar.pos)
        do_physics_step(bar, wheel, spring, grnd, M_inv, t_physics, dt_physics)
        current_state = bar.get_state()
        t_physics += dt_physics
        t_accumulator -= dt_physics
    
    # interpolate the state vector for rendering
    alpha = t_accumulator / dt_physics
    render_state = bar.interpolate_state(previous_state, current_state, alpha)
    bar.set_state(render_state)
    
    # draw the scene
    draw_wait_time += dt_loop
    if (draw_wait_time > 1.0 / 60.0):
        draw_wait_time = 0.0
        # compute frame time
        current_draw_time = time.time()
        dt_draw = current_draw_time - last_draw_time
        last_draw_time = current_draw_time

        # draw the scene
        fig.canvas.restore_region(bg)
        grnd.draw(ax)
        bar.draw(ax)
        wheel.draw(ax)
        spring.draw(ax)
        fig.canvas.blit(fig.bbox)
        fig.canvas.flush_events()

    # sleep (if possible)
    end_loop_time = time.time()
    sleep_duration = loop_lock_time - (end_loop_time - current_loop_time)
    if (sleep_duration > 0.0):
        time.sleep(sleep_duration)

