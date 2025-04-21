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
import streamlit.components.v1 as components
from drone_2d import u, drone_dynamics, plot_trajectory

def main():
    st.set_page_config(layout="wide")    
    st.title("2D Drone Simulation")

    # system parameters (2D Drone)
    m = 2.0 # mass (kg)
    L = 2.0 # length (m)
    g = 9.81 # gravity (m/s^2)
    I = (1/12) * m * L**2 # moment of inertia (kg*m^2)

    col1, col2, col3 = st.columns([1,3,1])

    with col1:
        st.write("System Parameters")
        m = st.slider("Mass (kg)", 0.5, 3.0, m, 0.1) # mass (kg)
        L = st.slider("Length (m)", 0.5, 50.0, L, 0.5) # length (m)
        g = st.slider("Gravity (m/s^2)", 0.5, 10.0, g, 0.5) # gravity (m/s^2)
        st.write("Moment of Inertia (kg*m^2):", (1/12) * m * L**2)

        st.write("Inputs (u(t))")
        st.latex(r"u(t) = \begin{bmatrix} F_1(t) \\ F_2(t) \end{bmatrix}")

    left, right = st.columns(2)
    with left:
        input_type = st.radio(
            "Inputs (u(t))",
            ["***Predefined***", "***User defined***"],
            captions=[
                "Predefined trajectory (example)",
                "Manually provide engine thrust forces",
            ],
            horizontal=True)
    with right:
        if(input_type == '***Predefined***'):
            F1 = None
            F2 = None
        else:
            F1 = st.slider("Left Engine Thrust Force (N)", 0.0, 40.0, 15.0, 0.1)
            F2 = st.slider("Right Engine Thrust Force (N)", 0.0, 40.0, 15.0, 0.1)

        params = {"m": m, "L": L, "g": g, "I": I, "F1": F1, "F2": F2}

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

        # # plot results
        # plot_results(t, x, y, theta)

    with col2:
        # plot trajectory
        ani = plot_trajectory(x, y, theta, t_end, params)
        components.html(ani.to_jshtml(), height=600)
        
    with col3:
        st.write("### System Dynamics")
        st.latex(r"\ddot{x} = -\frac{(F_1 + F_2) \cdot \sin(\theta)}{m}")
        st.latex(r"\ddot{y} = \frac{(F_1 + F_2) \cdot \cos(\theta)}{m} - g")
        st.latex(r"\ddot{\theta} = \frac{(F_2 - F_1) \cdot L}{2I}")
        st.divider()
        st.latex(r"I = \frac{1}{12} \cdot m \cdot L^2")

if __name__ == '__main__':
    main()