# 2D Drone Simulation

This project contains a basic 2D simulation of a drone using Python. 

The goal is to build a sandbox environment with different levels of modelling complexity, in order to test different control strategies and study the system dynamics.

## Click here to run it interactively: [![Streamlit App](https://static.streamlit.io/badges/streamlit_badge_black_white.svg)](https://drone-simulation.streamlit.app/)

![2d drone trajectory](https://github.com/fpelogia/drone-simulation/blob/main/trajectory.gif?raw=true)

## Features

- Models the translational and rotational dynamics of a 2D drone
- Simulates the effect of individual motor thrusts
- Includes a simple animation to visualize drone movement
- Available Controllers:
  - Full State Feedback (FSF)
    - With Pole Placement
    - With Linear Quadratic Regulator (LQR)

TO-DO:
- Specify trajectory (or point) from streamlit interface
  - Maybe later plot using Plotly and get user mouse click position as target point

## System Modeling & Control Analysis
### 1. Non-linear State-Space Dynamics

The 2D drone dynamics are modeled in state-space form, where the state vector $X$ and input vector $u$ are defined as:

$$X = \begin{bmatrix} x \\ y \\ \theta \\ \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}, \quad u = \begin{bmatrix} F_1 \\ F_2 \end{bmatrix}$$

The non-linear system state equation $\dot{X} = f(X, u)$ is given by:

$$\dot{X} = \begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta} \\
-\frac{\sin(\theta)}{m}(F_1 + F_2) \\
\frac{\cos(\theta)}{m}(F_1 + F_2) - g \\
\frac{(F_2 - F_1)L}{2I}
\end{bmatrix}$$

Where:
- $x, y$: Drone position in 2D space
- $\theta$: Orientation (pitch angle)
- $F_1, F_2$: Thrust forces from left and right engines
- $m$: Total drone mass
- $L$: Distance between engines
- $I$: Moment of inertia ($I = \frac{1}{12} m L^2$)
- $g$: Gravity acceleration

---

### 2. Equilibrium Points

Setting $\dot{X} = 0$ at flat orientation ($\theta_e = 0$) yields the hover equilibrium state $X_e$ and input $u_e$:

$$X_e = \begin{bmatrix} x_e \\ y_e \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}, \quad u_e = \begin{bmatrix} \frac{mg}{2} \\ \frac{mg}{2} \end{bmatrix}$$

---

### 3. Linearized Model ($\dot{X} = AX + Bu$)

Evaluating the Jacobian matrices at the equilibrium point ($\theta = 0$, $F_1 + F_2 = mg$):

$$A = \left. \frac{\partial f}{\partial X} \right|_{eq} = \begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & -g & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}$$

$$B = \left. \frac{\partial f}{\partial u} \right|_{eq} = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
\frac{1}{m} & \frac{1}{m} \\
-\frac{L}{2I} & \frac{L}{2I}
\end{bmatrix}$$

---

### 4. Stability Analysis

Solving $\det(A - \lambda I) = 0$ for the open-loop system:

$$\lambda^6 = 0 \implies \lambda_i = 0 \quad \forall i \in \{1, \dots, 6\}$$

The equilibrium point is **non-hyperbolic** because all eigenvalues have zero real parts ($\text{Re}(\lambda_i) = 0$), indicating marginal stability at open loop.

---

### 5. Controllability & State Feedback Control

The controllability matrix $\mathcal{C}$ is calculated as:

$$\mathcal{C} = \begin{bmatrix} B & AB & A^2B & A^3B & A^4B & A^5B \end{bmatrix}$$

$$\text{rank}(\mathcal{C}) = 6 \implies \text{The system is fully controllable.}$$

#### Full State Feedback (FSF)
Assuming full state measurement ($y = C X$ with $C = I_{6 \times 6}$):

$$u = u_e - K (X - X_e)$$

Where $K$ is the gain matrix synthesized using Pole Placement or Linear Quadratic Regulator (LQR) strategies.
