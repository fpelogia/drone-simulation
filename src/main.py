
import numpy as np
from scipy.integrate import solve_ivp
from dynamics import drone_dynamics
from plots import plot_results, plot_trajectory

def main():
    # system parameters (2D Drone)
    m = 2 # mass (kg)
    L = 2 # length (m)
    g = 9.81 # gravity (m/s^2)
    I = (1/12) * m * L**2 # moment of inertia (kg*m^2)

    params = {"m": m, "L": L, "g": g, "I": I}

    # initial conditions
    x0 = 0
    y0 = 0
    theta0 = 0
    x_dot0 = 0
    y_dot0 = 0
    theta_dot0 = 0

    z0 = [x0, y0, theta0, x_dot0, y_dot0, theta_dot0]

    # time interval
    t_start = 0
    t_end = 10
    t = np.linspace(t_start, t_end, 100)

    # solve ODE
    sol = solve_ivp(drone_dynamics, (t_start, t_end), z0, t_eval=t, args=(params,), rtol=1e-3, atol=1e-6)
    print(sol)

    # unpack solution
    x, y, theta, x_dot, y_dot, theta_dot = sol.y

    # plot results
    plot_results(t, x, y, theta)

    # plot trajectory
    plot_trajectory(x, y, theta, t, params)


if __name__ == '__main__':
    main()