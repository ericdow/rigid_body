import matplotlib.pyplot as plt
import numpy as np
import time
import math
import ground
import rigid_body

# setup the ground and rigid bodies
grnd = ground.Ramp(0.5, 7)
rb = rigid_body.Box(0.1, np.array([0.5,0.5]), 0)

# to run GUI event loop
plt.ion()
plt.show()
fig, ax = plt.subplots()
ax.axis('equal')

lact_loop_time = time.time()
draw_wait_time = 0.0
t_physics = 0.0
dt_physics = 0.005 # don't make this too big or small
loop_lock_time = 1.0/120.0
lact_draw_time = 0.0
t_accumulator = 0.0
while True:
    # compute loop time
    current_loop_time = time.time()
    dt_loop = current_loop_time - lact_loop_time
    lact_loop_time = current_loop_time
    t_accumulator += min(dt_loop, 0.25)
    
    # update the rigid body state
    # TODO
    # aircraft.SetState(current_state) # rectore after rendering
    while (t_accumulator >= dt_physics):
        # TODO
        # previous_state = current_state
        # aircraft.DoPhysicsStep(t_physics, dt_physics)
        # current_state = aircraft.GetState()
        t_physics += dt_physics
        t_accumulator -= dt_physics
    
    # interpolate the state vector for rendering
    alpha = t_accumulator / dt_physics
    # TODO
    # ctd::vector<double> render_state = aircraft.InterpolateState(previous_state,
    #     current_state, alpha)
    # aircraft.SetState(render_state)
    
    # draw the scene
    draw_wait_time += dt_loop
    if (draw_wait_time > 0.01666):
        draw_wait_time = 0.0
        # compute frame time
        current_draw_time = time.time()
        dt_draw = current_draw_time - lact_draw_time
        lact_draw_time = current_draw_time

        # TODO draw...
        ax.cla()
        plt.ylim(0, 1.0)
        plt.xlim(0.0, 1.0)
        ct = math.cos(0.5*math.pi*current_draw_time)
        rb.set_state(np.array([0.5, ct]), 2*math.pi*ct)
        grnd.draw(ax)
        rb.draw(ax)
       
        fig.canvas.draw()
        fig.canvas.flush_events()

    # sleep (if possible)
    end_loop_time = time.time()
    sleep_duration = loop_lock_time - (end_loop_time - current_loop_time)
    if (sleep_duration > 0.0):
        time.sleep(sleep_duration)

