import numpy as np
import math
import matplotlib.pyplot as plt
from typing import Tuple, Optional, Any, List
from dynamics.inverted_pendulum_dynamics import PendulumConfig, rk4_step

def simulation(x: np.ndarray, u: float, dt: float, config: PendulumConfig) -> np.ndarray:
    """
    Simulates the system for one time step using RK4 integration.
    
    This function handles input shape consistency (1D vs 2D arrays)
    and calls the RK4 integrator.
    
    Args:
        x: Current state [p, p_dot, theta, theta_dot]
        u: Control input (force)
        dt: Time step [s]
        config: Physical parameters
        
    Returns:
        Next state vector (same shape as input x)
    """
    # Ensure x is 1D array for dynamics calculation
    x_flat = x.flatten()
    
    # Integrate using Runge-Kutta 4th Order
    x_next = rk4_step(x_flat, u, dt, config)
    
    # Return as column vector to match expected shape in main loop if needed, 
    # but usually keeping it consistent is best. Let's return as (4,1) if input was (4,1).
    if x.ndim == 2:
        return x_next.reshape(-1, 1)
    return x_next

def flatten(a: Any) -> np.ndarray:
    """Helper to flatten arrays for plotting."""
    return np.array(a).flatten()

def plot_cart(xt: float, theta: float, config: PendulumConfig) -> None:
    """
    Visualizes the cart and pendulum system using Matplotlib.
    
    Args:
        xt: Cart position [m]
        theta: Pendulum angle [rad]
        config: Visualization parameters
    """
    cart_w = config.cart_w
    cart_h = config.cart_h
    radius = config.cart_r

    # Define Cart Geometry (Rectangle)
    cx: np.ndarray = np.array([-cart_w / 2.0, cart_w / 2.0, cart_w /
                   2.0, -cart_w / 2.0, -cart_w / 2.0])
    cy: np.ndarray = np.array([0.0, 0.0, cart_h, cart_h, 0.0])
    cy += radius * 2.0 # Lift cart above wheels

    # Shift cart to current position xt
    cx = cx + xt

    # Define Pendulum Geometry (Line)
    # Note: theta=0 is UP, so we plot relative to vertical
    bx: np.ndarray = np.array([0.0, config.l_bar * math.sin(-theta)])
    bx += xt
    by: np.ndarray = np.array([cart_h, config.l_bar * math.cos(-theta) + cart_h])
    by += radius * 2.0

    # Define Wheels (Circles)
    angles: np.ndarray = np.arange(0.0, math.pi * 2.0, math.radians(3.0))
    ox: np.ndarray = np.array([radius * math.cos(a) for a in angles])
    oy: np.ndarray = np.array([radius * math.sin(a) for a in angles])

    rwx: np.ndarray = np.copy(ox) + cart_w / 4.0 + xt
    rwy: np.ndarray = np.copy(oy) + radius
    lwx: np.ndarray = np.copy(ox) - cart_w / 4.0 + xt
    lwy: np.ndarray = np.copy(oy) + radius

    # Pendulum Mass (Circle at end of bar)
    wx: np.ndarray = np.copy(ox) + bx[-1]
    wy: np.ndarray = np.copy(oy) + by[-1]

    # Plotting
    plt.plot(flatten(cx), flatten(cy), "-b") # Cart body
    plt.plot(flatten(bx), flatten(by), "-k") # Pendulum bar
    plt.plot(flatten(rwx), flatten(rwy), "-k") # Right wheel
    plt.plot(flatten(lwx), flatten(lwy), "-k") # Left wheel
    plt.plot(flatten(wx), flatten(wy), "-k")   # Pendulum mass
    plt.title(f"x: {xt:.2f} , theta: {math.degrees(theta):.2f}")

    # Enable closing the plot with Escape key
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    plt.axis("equal")
