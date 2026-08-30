using LinearAlgebra
using ControlSystems
using ControlSystemsBase
using Plots
using NPZ

 # system parameters (2D Drone)
m = 2 # mass (kg)
L = 2 # length (m)
g = 9.81 # gravity (m/s^2)
In = (1/12) * m * L^2 # moment of inertia (kg*m^2)

A = [
    0 0 0 1 0 0
    0 0 0 0 1 0
    0 0 0 0 0 1
    0 0 -g 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
]

B = [
    0 0
    0 0
    0 0
    0 0
    1/m 1/m
    -L/(2*In) L/(2*In)
]

C = [B A*B A^2*B A^3*B A^4*B A^5*B]
println("Controllability Matrix", C)

print("Rank(C): ", rank(C))

eigenvalues, eigenvectors = eigen(A)
print("Eigenvalues of A: ", eigenvalues)


#p = [-5, -5, -5, -5, -5, -5]
#p = [−2,−2.5,−3,−3.5,−4,−4.5]
p = [-2, -5, -3, -3.5, -4, -4.5]
K = place(A, B, p)
println(K)

# u = -KX

# X' = AX + Bu

# => X' = AX -BKX = (A-BK)X
display(A-B*K)

eigenvalues_ambk, eigenvectors_ambk = eigen(A-B*K)
println("Eigenvalues of (A-BK): ", eigenvalues_ambk)

# Save to a .npy file
npzwrite("K.npy", K)