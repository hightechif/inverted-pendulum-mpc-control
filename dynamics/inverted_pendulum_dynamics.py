import numpy as np

# ==========================================
# 1. System Parameters
# ==========================================
l_bar = 2.0      # length of bar [m]
M = 1.0          # mass of cart [kg]
m = 0.3          # mass of pendulum [kg]
g = 9.8          # gravity [m/s^2]

# Control & Simulation Parameters
nx = 4           # number of states [x, v, theta, omega]

T = 30           # Horizon length
delta_t = 0.02   # time tick [s]

# ==========================================
# 2. Mathematical Model Implementation
# ==========================================
def get_model_matrix():
    """
    Returns the linearized, discretized State-Space matrices A and B
    based on the Lagrangian derivation.
    """

    # Continuous Time A Matrix (System Dynamics)
    # Rows: [dx/dt, d(dx)/dt, dtheta/dt, d(dtheta)/dt]
    A_cont = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, (-1.0) * m * g / M, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, g * (M + m) / (l_bar * M), 0.0]
    ])

    # Continuous Time B Matrix (Input Dynamics)
    # Effect of Force F on the states
    B_cont = np.array([
        [0.0],
        [1.0 / M],
        [0.0],
        [(-1.0) / (l_bar * M)]
    ])

    # Discretization (Forward Euler Method)
    # q[k+1] = (I + A*dt)q[k] + (B*dt)u[k]
    A_discrete = np.eye(nx) + delta_t * A_cont
    B_discrete = delta_t * B_cont

    return A_discrete, B_discrete