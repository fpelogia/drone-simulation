'''
Simple 2D Drone Simulation
@Author: Frederico José Ribeiro Pelogia
@Date: 2025-04-20
'''

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import streamlit as st

import sys
from pathlib import Path
src_path = Path(__file__).resolve().parent / "src"
if str(src_path) not in sys.path:
    sys.path.insert(0, str(src_path))

from controllers.controller import ControllerFSF
from dynamics import drone_dynamics
from plots import plot_results, plot_trajectory


st.set_page_config(layout="wide")    
st.title("2D Drone Simulation")

# system parameters (2D Drone)
m = 2.0 # mass (kg)
L = 2.0 # length (m)
g = 9.81 # gravity (m/s^2)
I = (1/12) * m * L**2 # moment of inertia (kg*m^2)

col1, col2 = st.columns(2)

with col1:
    col11, col12 = st.columns(2)
    with col11:
        st.write("### System Parameters")
        m = st.slider("Mass (kg)", 0.5, 3.0, m, 0.1) # mass (kg)
        L = st.slider("Length (m)", 0.5, 50.0, L, 0.5) # length (m)
        g = 9.81 # gravity (m/s^2)
        st.write("Gravity acceleration (m/s^2):", g)
        st.write("Moment of Inertia (kg*m^2):", (1/12) * m * L**2)
    with col12:
        st.space()
        st.write("### Initial Conditions")
        x0 = st.number_input("Initial x position (m)", value=0.0)
        y0 = st.number_input("Initial y position (m)", value=0.0)
        theta0 = np.radians(st.number_input("Initial orientation (degrees)", value=0.0))

    st.space()
    st.write("### Controller Settings")
    control_method_selected = st.selectbox("Select Control Method", ["Full State Feedback (FSF) with Pole Placement", "Full State Feedback (FSF) with LQR"])

    # map to accepted control method names (TO-DO: Improve this)
    control_method = "pole_placement" if control_method_selected == "Full State Feedback (FSF) with Pole Placement" else "lqr"

    params = {"m": m, "L": L, "g": g, "I": I}

    # initial conditions
    x_dot0 = 0
    y_dot0 = 0
    theta_dot0 = 0

    z0 = [x0, y0, theta0, x_dot0, y_dot0, theta_dot0]

    # time interval
    t_start = 0
    t_end = 10
    t = np.linspace(t_start, t_end, 100)

    # Instantiate the controller based on the selected control method
    controller = ControllerFSF(type=control_method)

    # solve ODE
    sol = solve_ivp(drone_dynamics, (t_start, t_end), z0, t_eval=t, args=(params, controller), rtol=1e-3, atol=1e-6)

    # unpack solution
    x, y, theta, x_dot, y_dot, theta_dot = sol.y
    

with col2:

    with st.spinner("Wait for it...", show_time=True):
        # plot trajectory
        ani = plot_trajectory(x, y, theta, t, params)
        st.iframe(ani.to_jshtml(), height=600)

        fig = plot_results(t, x, y, theta)
        st.pyplot(fig)
    

with col1:
    tab_latex, tab_paper = st.tabs(["LaTeX", "Handwritten 😉"])

    with tab_latex:
        st.markdown("### 1. System Dynamics (Non-linear Model)")
        st.latex(
            r"""
        X = \begin{bmatrix} x \\ y \\ \theta \\ \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}, \quad
        u = \begin{bmatrix} F_1 \\ F_2 \end{bmatrix}
        """
        )
        st.latex(
            r"""
        \dot{X} = f(X, u) = \begin{bmatrix}
        \dot{x} \\
        \dot{y} \\
        \dot{\theta} \\
        -\frac{\sin(\theta)}{m}(F_1 + F_2) \\
        \frac{\cos(\theta)}{m}(F_1 + F_2) - g \\
        \frac{(F_2 - F_1)L}{2I}
        \end{bmatrix}
        """
        )

        st.caption(
            r"Where $I = \frac{1}{12} m L^2$ is the drone's moment of inertia."
        )

        st.divider()

        st.markdown("### 2. Equilibrium Points")
        st.markdown("$\dot{X} = 0$ with $\theta_e = 0$:")
        st.latex(
            r"""
        X_e = \begin{bmatrix} x_e \\ y_e \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}, \quad
        u_e = \begin{bmatrix} \frac{mg}{2} \\ \frac{mg}{2} \end{bmatrix}
        """
        )

        st.divider()

        st.markdown("## 3. Linearized System ($\dot{X} = A X + B u$)")
        st.markdown(
            r"Jacobian Matrix evaluated at the equilibrium point ($\theta = 0, F_1 + F_2 = mg$):"
        )

        st.latex(
            r"""
        A = \left. \frac{\partial f}{\partial X} \right|_{eq} = \begin{bmatrix}
        0 & 0 & 0 & 1 & 0 & 0 \\
        0 & 0 & 0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 0 & 0 & 1 \\
        0 & 0 & -g & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0
        \end{bmatrix}
        """
        )

        st.latex(
            r"""
        B = \left. \frac{\partial f}{\partial u} \right|_{eq} = \begin{bmatrix}
        0 & 0 \\
        0 & 0 \\
        0 & 0 \\
        0 & 0 \\
        \frac{1}{m} & \frac{1}{m} \\
        -\frac{L}{2I} & \frac{L}{2I}
        \end{bmatrix}
        """
        )

        st.divider()

        st.markdown("## 4. Stability Analysis")
        st.markdown("Calculating the eigenvalues of $A$ via $\det(A - \lambda I) = 0$:")
        st.latex(r"\lambda^6 = 0 \implies \lambda_i = 0 \quad \forall i \in \{1, \dots, 6\}")
        st.markdown(
            "Equilibrium point is not hyperbolic (all eigenvalues have zero real part)."
        )

        st.divider()

        st.markdown("## 5. Controllability and Control")
        st.markdown(
            r"Controllability Matrix $\mathcal{C} = \begin{bmatrix} B & AB & A^2B & A^3B & A^4B & A^5B \end{bmatrix}$:"
        )
        st.latex(r"\text{rank}(\mathcal{C}) = 6 \quad \text{System is controllable.}")
        st.markdown("State Feedback for the Equilibrium Point:")
        st.latex(r"u = u_e - K (X - X_e)")

        st.markdown(
            "Where $K$ is the gain matrix designed via Pole Placement or LQR methods."
        )

        st.markdown("OBS: In this case, we are using Full State Feedback (FSF), since:")
        st.latex(r"y = C X, \quad C = I_{6x6} \implies y = X")
    with tab_paper:
        st.pdf('notes/notes_drone_fred.pdf')