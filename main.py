import numpy as np
import math
import matplotlib.pyplot as plt
import csv
import os
from dynamics.inverted_pendulum_dynamics import PendulumConfig
from control.mpc_control import MPCController
from simulation.simulation import simulation, plot_cart

def main() -> None:
    """
    Main function to run the Inverted Pendulum MPC simulation.
    
    This function:
    1. Initializes the system configuration and MPC controller.
    2. Sets the initial state of the pendulum (inverted with a small angle).
    3. Runs the simulation loop:
        a. Solves the MPC optimization problem to find the optimal control input.
        b. Applies the control input to the system using nonlinear dynamics.
        c. Updates the system state and time.
        d. Visualizes the system (optional).
        e. Logs the simulation data.
    4. Saves the logged data to a CSV file.
    """
    # Simulation parameters
    # delta_t is passed to config init, others are in config defaults
    delta_t = 0.02
    show_animation = True
    
    # Initialize configuration (loads from config.yaml if available)
    config = PendulumConfig(delta_t=delta_t)
    
    # Initialize MPC controller
    mpc = MPCController(config)
    
    # Initial state [p, p_dot, theta, theta_dot]
    # p: Cart position [m]
    # p_dot: Cart velocity [m/s]
    # theta: Pendulum angle [rad] (0 is upright)
    # theta_dot: Pendulum angular velocity [rad/s]
    x0: np.ndarray = np.array([
        [0.0],
        [0.0],
        [0.3], # Initial angle (radians) - slightly tipped
        [0.0]
    ])

    x: np.ndarray = np.copy(x0)
    current_time: float = 0.0

    # Data logging container
    log_data = []

    print("Starting simulation...")
    try:
        while current_time < config.sim_time:
            # 1. Calculate optimal control input using MPC
            # Returns: optimal state trajectory, optimal input sequence
            opt_x, opt_dx, opt_theta, opt_dtheta, opt_input = mpc.get_input(x)

            # Check if optimization succeeded
            if opt_input is None:
                print("Optimization failed, exiting.")
                break

            # 2. Get the first control input from the optimal sequence
            u: float = opt_input[0]

            # 3. Log data for analysis
            # Columns: time, p, p_dot, theta, theta_dot, u
            log_data.append([current_time, x[0, 0], x[1, 0], x[2, 0], x[3, 0], u])

            # 4. Simulate system one step forward using Non-linear dynamics (RK4)
            x = simulation(x, u, delta_t, config)
            
            # Update simulation time
            current_time += delta_t

            # 5. Visualize the system
            if show_animation:
                plt.clf()
                px: float = float(x[0, 0])
                theta: float = float(x[2, 0])
                plot_cart(px, theta, config)
                plt.xlim([-5.0, 5.0])
                plt.ylim([0.0, 4.0])
                plt.grid(True)
                plt.title(f"Time: {current_time:.2f}s, x: {px:.2f}, theta: {math.degrees(theta):.2f}")
                plt.pause(0.001)

    except KeyboardInterrupt:
        print("Simulation interrupted.")
    finally:
        print("Finish")
        print(f"Final State: x={float(x[0, 0]):.2f} [m] , theta={math.degrees(x[2, 0]):.2f} [deg]")
        
        # Ensure data directory exists
        os.makedirs('data', exist_ok=True)
        
        # Save log data to CSV
        csv_path = 'data/simulation_data.csv'
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'p', 'p_dot', 'theta', 'theta_dot', 'u'])
            writer.writerows(log_data)
        print(f"Data saved to {csv_path}")

        if show_animation:
            plt.show()

if __name__ == "__main__":
    main()