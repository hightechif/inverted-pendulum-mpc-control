import unittest
import numpy as np
from dynamics.inverted_pendulum_dynamics import PendulumConfig, get_continuous_model_matrix, nonlinear_dynamics, rk4_step

class TestPendulumDynamics(unittest.TestCase):
    def setUp(self):
        self.config = PendulumConfig()

    def test_continuous_matrix_shape(self):
        A, B = get_continuous_model_matrix(self.config)
        self.assertEqual(A.shape, (4, 4))
        self.assertEqual(B.shape, (4, 1))

    def test_equilibrium_up(self):
        """Test that the pendulum stays up if perfectly balanced with no input."""
        # State: [p, p_dot, theta, theta_dot]
        # UP position: theta = 0
        state = np.array([0.0, 0.0, 0.0, 0.0])
        u = 0.0
        
        deriv = nonlinear_dynamics(state, u, self.config)
        
        # Expected derivative is all zeros
        np.testing.assert_array_almost_equal(deriv, np.zeros(4))

    def test_equilibrium_down(self):
        """Test that the pendulum stays down if perfectly balanced with no input."""
        # DOWN position: theta = pi
        state = np.array([0.0, 0.0, np.pi, 0.0])
        u = 0.0
        
        deriv = nonlinear_dynamics(state, u, self.config)
        
        # Expected derivative is all zeros
        np.testing.assert_array_almost_equal(deriv, np.zeros(4))

    def test_fall_from_small_angle(self):
        """Test that the pendulum begins to fall if slightly off-center."""
        # Small angle
        state = np.array([0.0, 0.0, 0.1, 0.0])
        u = 0.0
        
        deriv = nonlinear_dynamics(state, u, self.config)
        
        # theta_ddot should be positive (falling away from 0)
        # Gravity pulls it down, increasing theta
        self.assertGreater(deriv[3], 0.0)

    def test_rk4_integration(self):
        """Test that RK4 step advances the state."""
        state = np.array([0.0, 0.0, 0.1, 0.0])
        u = 0.0
        dt = 0.01
        
        next_state = rk4_step(state, u, dt, self.config)
        
        # State should change
        self.assertFalse(np.array_equal(state, next_state))
        # Theta should increase (falling)
        self.assertGreater(next_state[2], state[2])

if __name__ == '__main__':
    unittest.main()
