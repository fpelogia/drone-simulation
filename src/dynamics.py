'''
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2025-04-20
'''
import numpy as np
from controllers.pid import PID

def target_traj(t):    
    return 20 * np.sin(t), 5*t

# Input vector (Thrust forces of the left (F1) and right (F2) engines of the drone at time t)
def u(t):
    # Manual trajectory (example)
    if(t<2):
        F1 = 15 #left engine
        F2 = 15 #right engine
    elif(t<3):
        F1 = 15
        F2 = 14.9
    elif(t<5):
        F1 = 14
        F2 = 15
    else:
        F1 = 18.6
        F2 = 18

    return [F1, F2]

def controller(t, z, params):
    # unpack state
    x, y, theta, x_dot, y_dot, theta_dot = z
    # unpack parameters
    m, I, L, g = params["m"], params["I"], params["L"], params["g"]
    
    #x_traj, y_traj = target_traj(t);

    # State feedback (full state)
    K = np.load('controller_design_julia/K.npy')

    # equilibrium conditions
    u_eq = np.array([m*g/2, m*g/2])
    z_eq = np.array([5, 5, 0, 0, 0, 0])

    [F1, F2] = u_eq - K @ (z - z_eq)

    return [F1, F2]

# System dynamics
def drone_dynamics(t, z, params):
    # unpack state
    x, y, theta, x_dot, y_dot, theta_dot = z
    # unpack inputs
    #F1, F2 = u(t)
    F1, F2 = controller(t,z, params)

    # unpack parameters
    m, I, L, g = params["m"], params["I"], params["L"], params["g"]

    # system dynamics
    x_ddot = (-1 * (F1 + F2) * np.sin(theta)) / m
    y_ddot = ((F1 + F2) * np.cos(theta)) / m - g
    theta_ddot = ((F2 - F1)*L) / (2*I)

    # state derivative
    zdot = [x_dot, y_dot, theta_dot, x_ddot, y_ddot, theta_ddot]
    return zdot
