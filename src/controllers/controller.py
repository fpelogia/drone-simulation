'''
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2026-08-30
'''
import numpy as np

def target_traj(t):
    return 20 * np.sin(t), 5*t


class Controller:

    def __init__(self):
        pass

    ''' law: Control law for the 2D-Drone
    @param t: time
    @param z: state vector [x, y, theta, x_dot, y_dot, theta_dot]
    @param params: dictionary with system parameters
    @return: control inputs [F1, F2]'''
    def law(self, t, z, params):

        # unpack parameters
        m, I, L, g = params["m"], params["I"], params["L"], params["g"]


        x_ref, y_ref = target_traj(t)

        # State feedback (full state)
        K = np.load('src/controller_design/K_full_state_feedback.npy')

        # equilibrium conditions
        u_eq = np.array([m*g/2, m*g/2])
        z_ref = np.array([x_ref, y_ref, 0, 0, 0, 0])

        u = u_eq - K @ (z - z_ref)

        return u # [F1, F2]
