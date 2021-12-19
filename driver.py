import matplotlib.pyplot as plt
import numpy as np
import time
import math
import ground
import rigid_body
import collision

# setup the ground and rigid bodies
grnd = ground.Ramp(0.02, 21)
rb = rigid_body.Box(0.1, 10.0, np.array([0.1,1.0]), 0)
rb.set_restitution(0.5)

# to run GUI event loop
fig, ax = plt.subplots()
ax.set(xlim=(0.0, 1.0), ylim=(0.0, 1.0))
ax.set_aspect('equal', 'box')
plt.show(block=False)
plt.pause(0.1)
grnd.draw(ax)
rb.draw(ax)
bg = fig.canvas.copy_from_bbox(fig.bbox)
fig.canvas.blit(fig.bbox)

last_loop_time = time.time()
draw_wait_time = 0.0
t_physics = 0.0
dt_physics = 1.0 / 60.0 # don't make this too big or small
loop_lock_time = 1.0/120.0
last_draw_time = 0.0
t_accumulator = 0.0
while True:
    # compute loop time
    current_loop_time = time.time()
    dt_loop = current_loop_time - last_loop_time
    last_loop_time = current_loop_time
    t_accumulator += min(dt_loop, 0.25)
    
    # update the rigid body state
    rb.set_state(rb.curr_state) # restore state after rendering
    while (t_accumulator >= dt_physics):
        rb.store_prev_state()
        rb.do_physics_step(dt_physics, grnd)
        rb.store_curr_state()
        t_physics += dt_physics
        t_accumulator -= dt_physics
    
    # interpolate the state vector for rendering
    alpha = t_accumulator / dt_physics
    rb.interpolate_state(alpha)
    
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
        rb.draw(ax)
        fig.canvas.blit(fig.bbox)
        fig.canvas.flush_events()

    # sleep (if possible)
    end_loop_time = time.time()
    sleep_duration = loop_lock_time - (end_loop_time - current_loop_time)
    if (sleep_duration > 0.0):
        time.sleep(sleep_duration)

