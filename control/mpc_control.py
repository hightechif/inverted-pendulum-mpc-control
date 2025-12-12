import cvxpy
import time
import numpy as np
from typing import Tuple, Optional, List
from dynamics.inverted_pendulum_dynamics import PendulumConfig, get_model_matrix

class MPCController:
    """
    Model Predictive Controller for the Inverted Pendulum.
    Uses CVXPY with parameters to cache the problem structure for faster solving.
    """
    def __init__(self, config: PendulumConfig):
        """
        Initialize the MPC Controller.

        Args:
            config: Configuration object containing system and MPC parameters.
        """
        self.config = config
        self.T = config.T
        self.nq = config.nq
        self.nu = config.nu
        
        # Cost matrices
        # Q: Penalty on state deviation (p, p_dot, theta, theta_dot)
        self.Q = np.diag(config.Q_diag)
        # R: Penalty on control input (force u)
        self.R = np.diag(config.R_diag)
        
        # Initialize the optimization problem
        self._init_problem()

    def _init_problem(self):
        """
        Sets up the CVXPY optimization problem.
        
        This method defines the variables, cost function, and constraints
        symbolically. The initial state is a parameter that is updated
        at each control step, allowing for fast re-solving.
        """
        # Get linearized discrete-time model matrices (A, B)
        # x[k+1] = A * x[k] + B * u[k]
        self.A, self.B = get_model_matrix(self.config)
        
        # Optimization Variables
        # x: State trajectory over the horizon (nq, T+1)
        self.x = cvxpy.Variable((self.nq, self.T + 1))
        # u: Control input sequence over the horizon (nu, T)
        self.u = cvxpy.Variable((self.nu, self.T))
        
        # Parameter for initial state (updated in get_input)
        self.x0_param = cvxpy.Parameter((self.nq,))
        
        cost = 0.0
        constr = []
        
        for t in range(self.T):
            # Quadratic cost function: x'Qx + u'Ru
            cost += cvxpy.quad_form(self.x[:, t + 1], self.Q)
            cost += cvxpy.quad_form(self.u[:, t], self.R)
            
            # System dynamics constraint
            constr += [self.x[:, t + 1] == self.A @ self.x[:, t] + self.B @ self.u[:, t]]
            
        # Initial state constraint: x[0] must match current system state
        constr += [self.x[:, 0] == self.x0_param]
        
        # Create the optimization problem object
        self.prob = cvxpy.Problem(cvxpy.Minimize(cost), constr)
        
        # Solver selection
        self.solver = cvxpy.CLARABEL

    def get_input(self, x_current: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Solves the MPC problem for the current state.
        
        Args:
            x_current: Current state vector (shape (4, 1) or (4,))
            
        Returns:
            Tuple containing:
            - opt_x: Optimal state trajectory (p)
            - opt_dx: Optimal state trajectory (p_dot)
            - opt_theta: Optimal state trajectory (theta)
            - opt_dtheta: Optimal state trajectory (theta_dot)
            - opt_u: Optimal input sequence
        """
        # Update the initial state parameter with the current system state
        self.x0_param.value = x_current.flatten()
        
        start = time.time()
        try:
            # Solve the optimization problem
            self.prob.solve(solver=self.solver, verbose=False)
        except cvxpy.error.SolverError:
            print("Solver failed.")
            return None, None, None, None, None
            
        # elapsed_time = time.time() - start
        # print(f"MPC calc time: {elapsed_time:.6f} [sec]")

        if self.prob.status == cvxpy.OPTIMAL:
            # Extract optimal trajectories
            opt_x = self.x.value[0, :]
            opt_dx = self.x.value[1, :]
            opt_theta = self.x.value[2, :]
            opt_dtheta = self.x.value[3, :]
            opt_u = self.u.value[0, :]
            return opt_x, opt_dx, opt_theta, opt_dtheta, opt_u
        else:
            print(f"Optimization failed with status: {self.prob.status}")
            return None, None, None, None, None