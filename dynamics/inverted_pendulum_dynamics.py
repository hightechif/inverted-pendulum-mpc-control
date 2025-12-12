import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional

import yaml
import os

# ==========================================
# 1. Configuration Data Class
# ==========================================
@dataclass
class PendulumConfig:
    """
    Data class to hold physical and simulation parameters.
    Loads from config.yaml if available, otherwise uses defaults.
    """
    # Physical Parameters
    l_bar: float = 2.0      # Length of the bar (distance to mass?) [m]
    M: float = 1.0          # Mass of the cart [kg]
    m: float = 0.3          # Mass of the pendulum [kg]
    g: float = 9.8          # Gravity acceleration [m/s^2]
    delta_t: float = 0.02   # Simulation time step [s]
    
    @property
    def nq(self) -> int:
        """Number of state variables (p, p_dot, theta, theta_dot)."""
        return 4

    @property
    def nu(self) -> int:
        """Number of control inputs (force u)."""
        return 1

    # MPC Parameters
    T: int = 100             # Prediction horizon length (steps)
    Q_diag: Tuple[float, ...] = (0.0, 1.0, 1.0, 0.0) # State cost weights
    R_diag: Tuple[float, ...] = (0.01,)              # Input cost weights

    # Simulation Constants
    sim_time: float = 5.0   # Total simulation duration [s]
    
    # Visualization Constants
    cart_w: float = 1.0     # Cart width [m]
    cart_h: float = 0.5     # Cart height [m]
    cart_r: float = 0.1     # Wheel radius [m]

    def __post_init__(self):
        """Load parameters from config.yaml if it exists."""
        config_path = os.path.join(os.path.dirname(__file__), '..', 'config.yaml')
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    
                if config:
                    self.l_bar = config.get('l_bar', self.l_bar)
                    self.M = config.get('M', self.M)
                    self.m = config.get('m', self.m)
                    self.g = config.get('g', self.g)
                    self.delta_t = config.get('delta_t', self.delta_t)
                    self.T = config.get('T', self.T)
                    if 'Q_diag' in config:
                        self.Q_diag = tuple(config['Q_diag'])
                    if 'R_diag' in config:
                        self.R_diag = tuple(config['R_diag'])
                    self.sim_time = config.get('sim_time', self.sim_time)
                    self.cart_w = config.get('cart_w', self.cart_w)
                    self.cart_h = config.get('cart_h', self.cart_h)
                    self.cart_r = config.get('cart_r', self.cart_r)
                    print(f"Loaded configuration from {config_path}")
            except Exception as e:
                print(f"Failed to load config.yaml: {e}. Using defaults.")
        else:
            print("config.yaml not found. Using defaults.")

# ==========================================
# 2. Mathematical Model Implementation
# ==========================================

def get_continuous_model_matrix(config: PendulumConfig) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generates the linearized Continuous Time State-Space matrices A and B.
    Linearized around the UP position (theta = 0).
    
    State vector x:
        x[0] = p          (Cart position)
        x[1] = p_dot      (Cart velocity)
        x[2] = theta      (Pendulum angle, 0 = Up)
        x[3] = theta_dot  (Pendulum angular velocity)
        
    Input vector u:
        u[0] = F          (Force applied to cart)

    System: dx/dt = A*x + B*u

    Args:
        config (PendulumConfig): The physical parameters of the system.

    Returns:
        Tuple[np.ndarray, np.ndarray]: A tuple containing the continuous A and B matrices.
    """
    # Unpack config for readability
    l = config.l_bar
    M = config.M
    m = config.m
    g = config.g

    # Continuous Time A Matrix (System Dynamics)
    # Rows correspond to derivatives of [p, p_dot, theta, theta_dot]
    A_cont: np.ndarray = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0 * m * g / M, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, g * (M + m) / (l * M), 0.0]
    ])

    # Continuous Time B Matrix (Input Dynamics)
    # Effect of Force F on the state derivatives
    B_cont: np.ndarray = np.array([
        [0.0],
        [1.0 / M],
        [0.0],
        [-1.0 / (l * M)]
    ])
    
    return A_cont, B_cont

def get_discrete_model_matrix(config: PendulumConfig) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generates the linearized, discretized State-Space matrices A and B 
    using Forward Euler discretization.
    
    Discrete model: q[k+1] = A_d * q[k] + B_d * u[k]

    Args:
        config (PendulumConfig): The physical parameters of the system.

    Returns:
        Tuple[np.ndarray, np.ndarray]: A tuple containing the discrete A and B matrices.
    """
    A_cont, B_cont = get_continuous_model_matrix(config)
    dt = config.delta_t

    # Discretization (Forward Euler Method)
    # q[k+1] = q[k] + (A*q[k] + B*u[k])*dt
    #        = (I + A*dt)*q[k] + (B*dt)*u[k]
    A_discrete: np.ndarray = np.eye(config.nq) + (A_cont * dt)
    B_discrete: np.ndarray = B_cont * dt

    return A_discrete, B_discrete

def get_model_matrix(config: PendulumConfig) -> Tuple[np.ndarray, np.ndarray]:
    """
    Alias for get_discrete_model_matrix for backward compatibility.
    """
    return get_discrete_model_matrix(config)

def nonlinear_dynamics(state: np.ndarray, u: float, config: PendulumConfig) -> np.ndarray:
    """
    Computes the time derivative of the state vector using the full nonlinear dynamics.
    
    State x = [p, p_dot, theta, theta_dot]
    
    Equations of motion (Lagrangian derivation):
    1. (M+m)p_dd + m*l*theta_dd*cos(theta) - m*l*theta_d^2*sin(theta) = u
    2. m*l*p_dd*cos(theta) + m*l^2*theta_dd - m*g*l*sin(theta) = 0
    
    This system is solved for accelerations p_dd and theta_dd.
    
    Args:
        state (np.ndarray): Current state [p, p_dot, theta, theta_dot]
        u (float): Input force
        config (PendulumConfig): Physical parameters
        
    Returns:
        np.ndarray: State derivative [p_dot, p_ddot, theta_dot, theta_ddot]
    """
    # Unpack parameters
    l = config.l_bar
    M = config.M
    m = config.m
    g = config.g
    
    # Unpack state
    # p = state[0] # Position not explicitly used in acceleration equations
    p_dot = state[1]
    theta = state[2]
    theta_dot = state[3]
    
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    
    # Mass Matrix D(q) * [p_dd; theta_dd] = RHS
    # [ M+m          m*l*cos(theta) ] [ p_dd   ]   [ m*l*theta_dot^2*sin(theta) + u ]
    # [ m*l*cos(theta)    m*l^2     ] [ theta_dd ] = [ m*g*l*sin(theta)             ]
    
    D = np.array([
        [M + m, m * l * cos_theta],
        [m * l * cos_theta, m * l**2]
    ])
    
    RHS = np.array([
        m * l * theta_dot**2 * sin_theta + u,
        m * g * l * sin_theta
    ])
    
    # Solve linear system for accelerations
    accels = np.linalg.solve(D, RHS)
    p_ddot = accels[0]
    theta_ddot = accels[1]
    
    return np.array([p_dot, p_ddot, theta_dot, theta_ddot])

def rk4_step(state: np.ndarray, u: float, dt: float, config: PendulumConfig) -> np.ndarray:
    """
    Performs a single Runge-Kutta 4th order (RK4) integration step.
    
    RK4 provides a more accurate approximation of the next state than Euler integration.
    
    Args:
        state (np.ndarray): Current state
        u (float): Control input (held constant over dt)
        dt (float): Time step
        config (PendulumConfig): Physical parameters
        
    Returns:
        np.ndarray: Next state after time dt
    """
    k1 = nonlinear_dynamics(state, u, config)
    k2 = nonlinear_dynamics(state + 0.5 * dt * k1, u, config)
    k3 = nonlinear_dynamics(state + 0.5 * dt * k2, u, config)
    k4 = nonlinear_dynamics(state + dt * k3, u, config)
    
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
