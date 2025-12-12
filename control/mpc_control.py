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
        self.config = config
        self.T = config.T
        self.nx = config.nx
        self.nu = config.nu
        
        # Cost matrices
        self.Q = np.diag(config.Q_diag)  # State cost
        self.R = np.diag(config.R_diag)  # Input cost
        
        # Initialize the optimization problem
        self._init_problem()

    def _init_problem(self):
        """
        Sets up the CVXPY problem with parameters.
        """
        # Get linearized model
        self.A, self.B = get_model_matrix(self.config)
        
        # Variables
        self.x = cvxpy.Variable((self.nx, self.T + 1))
        self.u = cvxpy.Variable((self.nu, self.T))
        
        # Parameter for initial state
        self.x0_param = cvxpy.Parameter((self.nx,))
        
        cost = 0.0
        constr = []
        
        for t in range(self.T):
            cost += cvxpy.quad_form(self.x[:, t + 1], self.Q)
            cost += cvxpy.quad_form(self.u[:, t], self.R)
            constr += [self.x[:, t + 1] == self.A @ self.x[:, t] + self.B @ self.u[:, t]]
            
        # Initial constraint
        constr += [self.x[:, 0] == self.x0_param]
        
        self.prob = cvxpy.Problem(cvxpy.Minimize(cost), constr)
        
        # Check available solvers
        self.solver = cvxpy.CLARABEL
        # You could add logic here to check for OSQP or others if needed

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
        # Update parameter
        self.x0_param.value = x_current.flatten()
        
        start = time.time()
        try:
            self.prob.solve(solver=self.solver, verbose=False)
        except cvxpy.error.SolverError:
            print("Solver failed.")
            return None, None, None, None, None
            
        elapsed_time = time.time() - start
        # print(f"MPC calc time: {elapsed_time:.6f} [sec]")

        if self.prob.status == cvxpy.OPTIMAL:
            ox = self.x.value[0, :]
            dx = self.x.value[1, :]
            theta = self.x.value[2, :]
            d_theta = self.x.value[3, :]
            ou = self.u.value[0, :]
            return ox, dx, theta, d_theta, ou
        else:
            print(f"Optimization failed with status: {self.prob.status}")
            return None, None, None, None, None