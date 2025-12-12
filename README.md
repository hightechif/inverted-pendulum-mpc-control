# Inverted Pendulum MPC Control

A robust Model Predictive Control (MPC) implementation for stabilizing an inverted pendulum on a cart. This project demonstrates the application of optimization-based control to a nonlinear system.

## Features

- **Model Predictive Control**: Uses a linearized model for prediction and optimization.
- **Nonlinear Simulation**: Validates the controller against full nonlinear dynamics using Runge-Kutta 4 (RK4) integration.
- **Real-time Visualization**: Animates the cart and pendulum system using Matplotlib.
- **Robust Stabilization**: Tuned to stabilize from significant initial angles (e.g., 0.3 rad).

## Prerequisites

- Python 3.8+
- [CVXPY](https://www.cvxpy.org/) (for convex optimization)
- [NumPy](https://numpy.org/) (for numerical operations)
- [Matplotlib](https://matplotlib.org/) (for visualization)

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd inverted-pendulum-mpc-control
   ```

2. Create and activate a virtual environment (optional but recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install numpy cvxpy matplotlib
   ```

## Usage

Run the main simulation script:

```bash
python main.py
```

The simulation will start, and you should see a window showing the cart balancing the pendulum. The terminal will output the simulation status.

## Testing

To run the unit tests for the dynamics and controller:

```bash
python -m unittest discover tests
```

## Project Structure

- `control/`: Contains the MPC controller implementation.
  - `mpc_control.py`: Defines the `MPCController` class and optimization problem.
- `dynamics/`: Contains the system dynamics and configuration.
  - `inverted_pendulum_dynamics.py`: Defines `PendulumConfig`, linearization logic, and nonlinear equations of motion.
- `simulation/`: Contains simulation utilities.
  - `simulation.py`: RK4 integration and plotting functions.
- `main.py`: Entry point for the simulation.
- `docs/`: Documentation files.
- `data/`: Simulation data files.

## Theory

The system consists of a pendulum of mass $m$ attached to a cart of mass $M$. The control input is the force $u$ applied to the cart. For a detailed derivation of the dynamics using Lagrangian mechanics, see [docs/inverted_pendulum_lagrangian.md](docs/inverted_pendulum_lagrangian.md).

The control strategy involves:
1. **Linearization**: The nonlinear dynamics are linearized around the upright equilibrium point ($\theta = 0$).
2. **Discretization**: The continuous linear model is discretized using Forward Euler method.
3. **MPC Formulation**: An optimization problem is solved at each time step to minimize a cost function over a finite horizon $T$, subject to system dynamics and constraints.

$$
J = \sum_{k=0}^{T-1} (x_{k+1}^T Q x_{k+1} + u_k^T R u_k)
$$

where $Q$ and $R$ are weighting matrices penalizing state deviation and control effort, respectively.

## License

See [LICENSE](LICENSE) for details.
