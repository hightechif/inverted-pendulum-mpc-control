import numpy as np
import math
import matplotlib.pyplot as plt
import csv
from dynamics.inverted_pendulum_dynamics import PendulumConfig
from control.mpc_control import MPCController
from simulation.simulation import simulation, plot_cart

def main() -> None:
    # Simulation parameters
    # delta_t is passed to config init, others are in config defaults
    delta_t = 0.02
    show_animation = True
    
    # Initialize configuration
    config = PendulumConfig(delta_t=delta_t)
    
    # Initialize controller
    mpc = MPCController(config)
    
    # Initial state [p, p_dot, theta, theta_dot]
    x0: np.ndarray = np.array([
        [0.0],
        [0.0],
        [0.3], # Initial angle (radians)
        [0.0]
    ])

    x: np.ndarray = np.copy(x0)
    current_time: float = 0.0

    # Data logging
    log_data = []

    print("Starting simulation...")
    try:
        while current_time < config.sim_time:
            # Calculate control input
            opt_x, opt_dx, opt_theta, opt_dtheta, opt_input = mpc.get_input(x)

            # Check if optimization succeeded
            if opt_input is None:
                print("Optimization failed, exiting.")
                break

            # Get input (first element of optimal sequence)
            u: float = opt_input[0]

            # Log data
            # time, p, p_dot, theta, theta_dot, u
            log_data.append([current_time, x[0, 0], x[1, 0], x[2, 0], x[3, 0], u])

            # Simulate system (Non-linear dynamics)
            x = simulation(x, u, delta_t, config)
            
            # Update time
            current_time += delta_t

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
        
        # Save log data
        with open('data/simulation_data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'p', 'p_dot', 'theta', 'theta_dot', 'u'])
            writer.writerows(log_data)
        print("Data saved to data/simulation_data.csv")

        if show_animation:
            plt.show()

if __name__ == "__main__":
    main()