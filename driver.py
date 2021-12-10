import matplotlib.pyplot as plt
import time
import math

fig, ax = plt.subplots()

# to run GUI event loop
plt.ion()
plt.show()

last_loop_time = time.time()
draw_wait_time = 0.0
t_physics = 0.0
dt_physics = 0.005 # don't make this too big or small
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
    # TODO
    # aircraft.SetState(current_state) # restore after rendering
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
    # std::vector<double> render_state = aircraft.InterpolateState(previous_state,
    #     current_state, alpha)
    # aircraft.SetState(render_state)
    
    # draw the scene
    draw_wait_time += dt_loop
    if (draw_wait_time > 0.01666):
        draw_wait_time = 0.0
        # compute frame time
        current_draw_time = time.time()
        dt_draw = current_draw_time - last_draw_time
        last_draw_time = current_draw_time

        # TODO draw...
        ax.cla()
        plt.ylim(-1.0, 1.0)
        ax.plot(0.0, math.sin(current_draw_time), 'x')
       
        fig.canvas.draw()
        fig.canvas.flush_events()

    # sleep (if possible)
    end_loop_time = time.time()
    sleep_duration = loop_lock_time - (end_loop_time - current_loop_time)
    if (sleep_duration > 0.0):
        time.sleep(sleep_duration)

