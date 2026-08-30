'''
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2026-08-30
'''

import numpy as np
import control

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

# Controllability Matrix
C = np.hstack([B, A @ B, np.linalg.matrix_power(A, 2) @ B, np.linalg.matrix_power(A, 3) @ B, np.linalg.matrix_power(A, 4) @ B, np.linalg.matrix_power(A, 5) @ B])
print("Controllability Matrix\n", C)

print("Rank(C): ", np.linalg.matrix_rank(C))

eigenvalues, eigenvectors = np.linalg.eig(A)
print("Eigenvalues of A: ", eigenvalues)

# p = [-5, -5, -5, -5, -5, -5]
# p = [-2, -2.5, -3, -3.5, -4, -4.5]
p = [-2, -5, -3, -3.5, -4, -4.5]

# Em Python/Control, place(A, B, p) retorna a matriz K
K = control.place(A, B, p)
print(K)

# u = -KX
# X' = AX + Bu
# => X' = AX -BKX = (A-BK)X
print(A - B @ K)

eigenvalues_ambk, eigenvectors_ambk = np.linalg.eig(A - B @ K)
print("Eigenvalues of (A-BK): ", eigenvalues_ambk)

# Save K as a npy file
np.save("K_full_state_feedback.npy", K)