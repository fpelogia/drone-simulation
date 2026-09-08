"""
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2026-08-30
"""
from abc import ABC, abstractmethod
import numpy as np


def target_traj(t):
    return np.sin(t) + t, 2 * t

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

    def __init__(self, type='lqr', target_fn=target_traj, gain_matrix=None):
        self.type = type
        self.target_fn = target_fn

        if gain_matrix is not None:
            # If a specific gain matrix is provided, use it
            self.K = gain_matrix
        elif type == 'lqr':
            print('LQR FSF')
            # If LQR, use the designed LQR gain matrix
            self.K = np.load('src/controller_design/K_fsf_lqr.npy') 
        elif type == 'pole_placement':
            print('Pole Placement FSF')
            # If Pole Placement, use the designed Pole Placement gain matrix
            self.K = np.load('src/controller_design/K_fsf_pp.npy') 
        else:
            raise ValueError("Invalid controller type. Choose 'lqr' or 'pole_placement'.")
        
    def law(self, t, z, params):
        m, g = params["m"], params["g"]

        # get only system state (ignore the integrators)
        z = z[:6]

        x_ref, y_ref = self.target_fn(t)

        # equilibrium conditions
        u_eq = np.array([m * g / 2, m * g / 2])
        z_ref = np.array([x_ref, y_ref, 0, 0, 0, 0])

        u = u_eq - self.K @ (z - z_ref)

        return u  # [F1, F2]

# Performance is not great... maybe I'll improve later
class ControllerPID(Controller):
    """Cascaded PID Controller for the 2D-Drone."""
    
    def __init__(self, target_fn=target_traj):
        self.target_fn = target_fn
        
        self.k_x = {'p': 2.0,  'd': 1.5, 'i': 0.05} # Horizontal PID (set theta reference)
        self.k_y = {'p': 12.0, 'd': 6.0, 'i': 0.5}  # Height PID
        self.k_theta = {'p': 25.0, 'd': 8.0, 'i': 1.0}  # Attitude PID
        
    def law(self, t, z, params):
        m, g, L = params["m"], params["g"], params["L"]
        x, y, theta, x_dot, y_dot, theta_dot, int_x, int_y, int_theta = z
        
        x_ref, y_ref = self.target_fn(t)

        # Height PID -> total force
        e_y = y - y_ref
        F_total = m * g - (self.k_y['p']*e_y + self.k_y['d']*y_dot + self.k_y['i']*int_y)
        
        # Horizontal PID -> theta reference
        e_x = x - x_ref
        # horizontal force is F_total * sin(theta) ~ F_total * theta
        theta_ref = (self.k_x['p']*e_x + self.k_x['d']*x_dot + self.k_x['i']*int_x) / max((F_total/m), 1.0)
        
        # Attitude PID -> torque
        e_theta = theta - theta_ref
        torque = -(self.k_theta['p']*e_theta + self.k_theta['d']*theta_dot + self.k_theta['i']*int_theta)
        
        # F_total = F1 + F2
        # torque is (F2 - F1) * (L/2)
        delta_F = 2 * torque / L
        
        F1 = (F_total - delta_F) / 2
        F2 = (F_total + delta_F) / 2

        # engines cannot push down
        F1, F2 = max(0, F1), max(0, F2)
        
        return np.array([F1, F2])