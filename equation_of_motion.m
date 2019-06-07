function theta_ddot = equation_of_motion(theta, theta_dot, torque, m, g, l, damping)

theta_ddot = (torque - damping * theta_dot - m * g * l * sin(theta)) / (m * l);