'''
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2025-04-20
'''
import numpy as np

from controllers.controller import ControllerFSF

# System dynamics
def drone_dynamics(t, z, params, controller):
    # unpack state
    x, y, theta, x_dot, y_dot, theta_dot = z

    # unpack parameters
    m, I, L, g = params["m"], params["I"], params["L"], params["g"]

    # apply control law
    F1, F2 = controller.law(t, z, params)

    # system dynamics
    x_ddot = (-1 * (F1 + F2) * np.sin(theta)) / m
    y_ddot = ((F1 + F2) * np.cos(theta)) / m - g
    theta_ddot = ((F2 - F1)*L) / (2*I)

    # state derivative
    zdot = [x_dot, y_dot, theta_dot, x_ddot, y_ddot, theta_ddot]
    return zdot
