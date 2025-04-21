'''
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2025-04-20
'''
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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

# System dynamics
def drone_dynamics(t, z, params):

    # unpack state
    x, y, theta, x_dot, y_dot, theta_dot = z
    # unpack inputs
    F1, F2 = u(t)
    # unpack parameters
    m, I, L, g = params["m"], params["I"], params["L"], params["g"]

    # system dynamics
    x_ddot = (-1 * (F1 + F2) * np.sin(theta)) / m
    y_ddot = ((F1 + F2) * np.cos(theta)) / m - g
    theta_ddot = ((F2 - F1)*L) / (2*I)

    # state derivative
    zdot = [x_dot, y_dot, theta_dot, x_ddot, y_ddot, theta_ddot]
    return zdot


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
    sol = solve_ivp(drone_dynamics, (t_start, t_end), z0, t_eval=t, args=(params,))
    print(sol)

    # unpack solution
    x, y, theta, x_dot, y_dot, theta_dot = sol.y

    # plot results
    plot_results(t, x, y, theta)

    # plot trajectory
    plot_trajectory(x, y, theta, t_end, params)


# plot results
def plot_results(t, x, y, theta):
    fig, axs = plt.subplots(2, 2, figsize=(10, 10))
    plt.subplots_adjust(hspace=0.5)

    axs[0, 0].plot(t, x)
    axs[0, 0].set_title("Horizontal Position")
    axs[0, 0].set_xlabel("t (s)")
    axs[0, 0].set_ylabel("x (m)")

    axs[0, 1].plot(t, y)
    axs[0, 1].set_title("Vertical Position")
    axs[0, 1].set_xlabel("t (s)")
    axs[0, 1].set_ylabel("y (m)")

    axs[1, 0].plot(t, theta)
    axs[1, 0].set_title("Pitch Angle")
    axs[1, 0].set_xlabel("t (s)")
    axs[1, 0].set_ylabel("theta (rad)")

    axs[1, 1].plot(x, y)
    axs[1, 1].set_title("Trajectory")
    axs[1, 1].set_xlabel("x (m)")
    axs[1, 1].set_ylabel("y (m)")

    plt.savefig("results.png")
    plt.show()

# Plot trajectory (animated)
def plot_trajectory(x, y, theta, t_end, params):
    # unpack parameters
    m, I, L, g = params["m"], params["I"], params["L"], params["g"]

    fig, ax = plt.subplots()
    traj_plot, = ax.plot(x, y)
    patch = ax.add_patch(plt.Rectangle((x[0] - L/2, y[0]-0.1), L, 0.2, angle=theta[0], color='r'))
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory")
    # set xlim and ylim to show the entire trajectory
    plt.xlim(min(x) - L, max(x) + L)
    plt.ylim(0, max(y) + 1)


    # animation (docs: https://matplotlib.org/stable/users/explain/animations/animations.html#funcanimation)
    def update(frame):
        # for each frame, update the data stored on each artist.
        x_i = x[:frame]
        y_i = y[:frame]
        
        # update the line plot:
        traj_plot.set_xdata(x_i[:frame])
        traj_plot.set_ydata(y_i[:frame])

        # update patch
        patch.set_x(x[frame] - L/2)
        patch.set_y(y[frame] - 0.1)
        patch.set_angle(theta[frame])
        
        return (traj_plot, patch)

    ani = animation.FuncAnimation(fig=fig, func=update, frames=len(x), interval=(t_end*1000)/len(x), repeat_delay=4000)
    ani.save('trajectory.gif', writer='pillow')
    plt.show()

if __name__ == '__main__':
    main()