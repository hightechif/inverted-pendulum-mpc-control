import numpy as np
from dataclasses import dataclass
from typing import Tuple

# ==========================================
# 1. Configuration Data Class
# ==========================================
@dataclass
class PendulumConfig:
    """
    Data class to hold physical and simulation parameters.
    This replaces global variables for better encapsulation.
    """
    l_bar: float = 2.0      # length of bar [m]
    M: float = 1.0          # mass of cart [kg]
    m: float = 0.3          # mass of pendulum [kg]
    g: float = 9.8          # gravity [m/s^2]
    delta_t: float = 0.02   # time tick [s]
    
    @property
    def nx(self) -> int:
        """Number of states (fixed for this physics model)."""
        return 4

    @property
    def nu(self) -> int:
        """Number of inputs (fixed for this physics model)."""
        return 1

# ==========================================
# 2. Mathematical Model Implementation
# ==========================================
def get_model_matrix(config: PendulumConfig) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generates the linearized, discretized State-Space matrices A and B 
    based on the Lagrangian derivation.

    Args:
        config (PendulumConfig): The physical parameters of the system.

    Returns:
        Tuple[np.ndarray, np.ndarray]: A tuple containing the discrete A and B matrices.
    """
    
    # Unpack config for readability
    l = config.l_bar
    M = config.M
    m = config.m
    g = config.g
    dt = config.delta_t

    # Continuous Time A Matrix (System Dynamics)
    # Rows: [dx/dt, d(dx)/dt, dtheta/dt, d(dtheta)/dt]
    A_cont: np.ndarray = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0 * m * g / M, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, g * (M + m) / (l * M), 0.0]
    ])

    # Continuous Time B Matrix (Input Dynamics)
    # Effect of Force F on the states
    B_cont: np.ndarray = np.array([
        [0.0],
        [1.0 / M],
        [0.0],
        [-1.0 / (l * M)]
    ])

    # Discretization (Forward Euler Method)
    # q[k+1] = (I + A*dt)q[k] + (B*dt)u[k]
    A_discrete: np.ndarray = np.eye(config.nx) + (A_cont * dt)
    B_discrete: np.ndarray = B_cont * dt

    return A_discrete, B_discrete
