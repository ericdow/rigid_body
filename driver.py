import matplotlib.pyplot as plt
import numpy as np
import time
import math
import ground
import rigid_body
import collision

# setup the ground and rigid bodies
grnd = ground.Flat(7)
rb = rigid_body.Box(0.1, 1.0, np.array([0.5,0.5]), 0)

# to run GUI event loop
plt.ion()
plt.show()
fig, ax = plt.subplots()
ax.axis('equal')

last_loop_time = time.time()
draw_wait_time = 0.0
t_physics = 0.0
dt_physics = 0.005 # don't make this too big or small
loop_lock_time = 1.0/120.0
last_draw_time = 0.0
t_accumulator = 0.0
current_state = rb.get_state()
previous_state = rb.get_state()
while True:
    # compute loop time
    current_loop_time = time.time()
    dt_loop = current_loop_time - last_loop_time
    last_loop_time = current_loop_time
    t_accumulator += min(dt_loop, 0.25)
    
    # update the rigid body state
    rb.set_state(current_state) # restore state after rendering
    while (t_accumulator >= dt_physics):
        previous_state = current_state
        rb.do_physics_step(dt_physics, grnd)
        current_state = rb.get_state()
        t_physics += dt_physics
        t_accumulator -= dt_physics
    
    # interpolate the state vector for rendering
    alpha = t_accumulator / dt_physics
    render_state = rb.interpolate_state(previous_state, current_state, alpha)
    rb.set_state(render_state)
    
    # draw the scene
    draw_wait_time += dt_loop
    if (draw_wait_time > 0.01666):
        draw_wait_time = 0.0
        # compute frame time
        current_draw_time = time.time()
        dt_draw = current_draw_time - last_draw_time
        last_draw_time = current_draw_time

        # draw the scene
        ax.cla()
        plt.ylim(0, 1.0)
        plt.xlim(0.0, 1.0)
        grnd.draw(ax)
        rb.draw(ax)
       
        fig.canvas.draw()
        fig.canvas.flush_events()

    # sleep (if possible)
    end_loop_time = time.time()
    sleep_duration = loop_lock_time - (end_loop_time - current_loop_time)
    if (sleep_duration > 0.0):
        time.sleep(sleep_duration)

