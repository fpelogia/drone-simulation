import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dynamics import target_traj

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

    # traj_x, traj_y = target_traj(t)
    # axs[1, 1].plot(traj_x, traj_y, linestyle='--')


    plt.savefig("results.png")
    plt.show()

# Plot trajectory (animated)
def plot_trajectory(x, y, theta, t_end, params):
    # unpack parameters
    m, I, L, g = params["m"], params["I"], params["L"], params["g"]

    print(theta)
    fig, ax = plt.subplots()
    traj_plot, = ax.plot(x, y)
    patch = ax.add_patch(plt.Rectangle((x[0] - L/2, y[0]-0.1), L, 0.2, angle=theta[0]*(180/np.pi), color='r'))


    # [TO-DO] - Improve this... get t array directly
    # t = np.linspace(0, t_end, 100)
    # traj_x, traj_y = target_traj(t)
    # ax.plot(traj_x, traj_y, linestyle='--')

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
        patch.set_angle(theta[frame]*(180/np.pi))
        
        return (traj_plot, patch)

    ani = animation.FuncAnimation(fig=fig, func=update, frames=len(x), interval=(t_end*1000)/len(x), repeat_delay=4000)
    ani.save('trajectory.gif', writer='pillow')
    plt.show()

    return ani