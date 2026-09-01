"""
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2026-08-30
"""
from abc import ABC, abstractmethod
import numpy as np


def target_traj(t):
    return 20 * np.sin(t), 5 * t

class Controller(ABC):
    @abstractmethod
    def law(self, t, z, params):
        """Control law for the 2D-Drone.

        @param t: time
        @param z: state vector [x, y, theta, x_dot, y_dot, theta_dot]
        @param params: dictionary with system parameters
        @return: control inputs [F1, F2]
        """
        pass


class ControllerFSF(Controller):
    """Full State Feedback Controller for the 2D-Drone."""

    def __init__(self, type='lqr', gain_matrix=None):
        self.type = type

        if gain_matrix is not None:
            # If a specific gain matrix is provided, use it
            self.K = gain_matrix
        elif type == 'lqr':
            print('LQR FSF')
            # If LQR, use the designed LQR gain matrix
            self.K = np.load('src/controller_design/K_fsf_pp.npy') 
        elif type == 'pole_placement':
            print('Pole Placement FSF')
            # If Pole Placement, use the designed Pole Placement gain matrix
            self.K = np.load('src/controller_design/K_fsf_pp.npy') 
        else:
            raise ValueError("Invalid controller type. Choose 'lqr' or 'pole_placement'.")
        
    def law(self, t, z, params):
        m, g = params["m"], params["g"]

        x_ref, y_ref = target_traj(t)

        # equilibrium conditions
        u_eq = np.array([m * g / 2, m * g / 2])
        z_ref = np.array([x_ref, y_ref, 0, 0, 0, 0])

        u = u_eq - self.K @ (z - z_ref)

        return u  # [F1, F2]