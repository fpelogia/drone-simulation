'''
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2026-08-30
'''

import numpy as np
import control as ct
import matplotlib.pyplot as plt

# system parameters (2D Drone)
m = 2 # mass (kg)
L = 2 # length (m)
g = 9.81 # gravity (m/s^2)
In = (1/12) * m * L**2 # moment of inertia (kg*m^2)

# X' = AX + Bu
A = np.array([
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, -g, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]
], dtype=float)

B = np.array([
    [0, 0],
    [0, 0],
    [0, 0],
    [0, 0],
    [1/m, 1/m],
    [-L/(2*In), L/(2*In)]
], dtype=float)

# u = [F1, F2] (forces from the left and right engines)

# Controllability Matrix
Ctrb = np.hstack([B, A @ B, np.linalg.matrix_power(A, 2) @ B, np.linalg.matrix_power(A, 3) @ B, np.linalg.matrix_power(A, 4) @ B, np.linalg.matrix_power(A, 5) @ B])
print("Controllability Matrix\n", Ctrb)

print("Rank(Ctrb): ", np.linalg.matrix_rank(Ctrb))

eigenvalues, eigenvectors = np.linalg.eig(A)
print("Eigenvalues of A: ", eigenvalues)

# ======================================================================================

# Linear Quadratic Regulator (LQR)
Q = np.array([
    [10, 0, 0, 0, 0, 0], # error penalty for x
    [0, 10, 0, 0, 0, 0], # error penalty for y
    [0, 0, 1, 0, 0, 0], # error penalty for theta
    [0, 0, 0, 1, 0, 0], # error penalty for x_dot
    [0, 0, 0, 0, 1, 0], # error penalty for y_dot
    [0, 0, 0, 0, 0, 1] # error penalty for theta_dot
])

R = np.array([
    [1, 0],
    [0, 1]
])

K, S, E = ct.lqr(A, B, Q, R)
print(K)

# u = -KX
# X' = AX + Bu
# => X' = AX -BKX = (A-BK)X

# Closed Loop System
A_cl = A - B @ K
sys_sim = ct.ss(A_cl, np.zeros((6, 1)), np.eye(6), np.zeros((6, 1)))

eigenvalues_ambk, eigenvectors_ambk = np.linalg.eig(A_cl)
print("Eigenvalues of (A-BK): ", eigenvalues_ambk)

# Save K as a npy file
np.save("src/controller_design/K_fsf_lqr.npy", K)

# ======================================================================================

# Response to initial conditions
x0 = [10.0, 10.0, np.pi/4, 0.0, 0.0, 0.0]  # [x, y, theta, vx, vy, vtheta]
t = np.linspace(0, 5, 500)
t, y = ct.initial_response(sys_sim, T=t, X0=x0)

fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

axs[0].plot(t, y[0, :], label='x (m)', color='r', linewidth=2)
axs[0].plot(t, y[1, :], label='y (m)', color='g', linewidth=2)
axs[0].plot(t, y[2, :], label=r'$\theta$ (rad)', color='b', linestyle='--', linewidth=2)
axs[0].set_ylabel('Position / Attitude')
axs[0].set_title('Drone response to initial condition')
axs[0].grid(True)
axs[0].legend()

# Subplot 2: Velocidades
axs[1].plot(t, y[3, :], label=r'$v_x$ (m/s)', color='r', linestyle=':')
axs[1].plot(t, y[4, :], label=r'$v_y$ (m/s)', color='g', linestyle=':')
axs[1].plot(t, y[5, :], label=r'$\omega$ (rad/s)', color='b', linestyle=':')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Velocity')
axs[1].grid(True)
axs[1].legend()

plt.tight_layout()
plt.show()

plt.close('all')

# Drone trajectory
plt.figure(figsize=(10, 6))
plt.plot(y[0, :], y[1, :], color='purple', linewidth=2, label='Trajectory')
plt.plot(x0[0], x0[1], 'go', label='Start')
plt.plot(0, 0, 'rx', label='Target (Origin)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Drone trajectory')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()

plt.close('all')

# Bode Diagram - frequency response
# # Frequency response of the drone height to the left engine input (F1)
sys_mimo = ct.ss(A_cl, B, np.eye(6), np.zeros((6, 2)))
plt.figure(figsize=(10, 6))
ct.bode_plot(sys_mimo[1, 0], dB=True, Hz=False, deg=True)
plt.suptitle('Frequency response of the drone height to the left engine input (F1)')
plt.show()

plt.close('all')
plt.figure(figsize=(10, 6))
# # Frequency response of the drone height to the left engine input (F1)
ct.bode_plot(sys_mimo[1, 1], dB=True, Hz=False, deg=True)
plt.suptitle('Frequency response of the drone height to the right engine input (F2)')
plt.show()