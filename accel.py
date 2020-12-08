def find_optimal_acceleration(vg, vc):
    v_goal = vg # output of neural net in m/s
    v_curr = vc # current velocity in m/s
    steps = 10 # number of steps ahead we're calculating
    step_dist = 0.1 # in meters
    max_safe_a = 10 # in m/s^2 needs to be adjusted, absolute value
    min_steps = steps
    accel = 0
    accel_step = min_steps

    while min_steps > 0:
        a = ((v_goal**2)-(v_curr**2))/(2*(min_steps*step_dist))
        if abs(a) < max_safe_a:
            accel = a
            accel_step = min_steps
        min_steps = min_steps - 1

    return accel, accel_step


def vel_pid(vg, vc, dt):
    v_goal = vg # output of neural net in m/s
    v_curr = vc # current velocity in m/s
    prev_err = 0.0
    windup_guard = 10 # needs to be changed??
    kp = 1
    ki = 0.1
    kd = 0.1
    error = v_goal - v_curr
    delta_error = error - prev_err
    p = kp*error
    i = i + error * dt
    if i < -windup_guard:
        i = windup_guard
    elif i > windup_guard:
        i = windup_guard
    d = 0.0 # use acceleration calculated above??
    if delta_time > 0:
        d = delta_error/dt
    prev_err = error
    vel = p + ki*i + kd*d
    return vel