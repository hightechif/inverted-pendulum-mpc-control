import unittest
import numpy as np
from dynamics.inverted_pendulum_dynamics import PendulumConfig
from control.mpc_control import MPCController

class TestMPCController(unittest.TestCase):
    def setUp(self):
        self.config = PendulumConfig()
        # Use a shorter horizon for faster tests if needed, but standard is fine
        self.controller = MPCController(self.config)

    def test_initialization(self):
        """Test that the controller initializes correctly."""
        self.assertIsInstance(self.controller, MPCController)
        self.assertEqual(self.controller.T, self.config.T)
        self.assertEqual(self.controller.nq, self.config.nq)
        self.assertEqual(self.controller.nu, self.config.nu)

    def test_get_input_success(self):
        """Test that get_input returns a valid solution for a standard state."""
        # State: [p, p_dot, theta, theta_dot]
        # Small angle error
        x = np.array([0.0, 0.0, 0.1, 0.0])
        
        opt_x, opt_dx, opt_theta, opt_dtheta, opt_input = self.controller.get_input(x)
        
        # Check that solution is not None
        self.assertIsNotNone(opt_input)
        self.assertIsNotNone(opt_x)
        
        # Check shapes
        self.assertEqual(len(opt_input), self.config.T)
        self.assertEqual(len(opt_x), self.config.T + 1)

    def test_stabilization_direction(self):
        """Test that the controller pushes in the correct direction."""
        # If theta is positive (tipping right), cart should move right (positive force) 
        # to get under the pendulum.
        # Wait, if theta is positive (tipping right), cart needs to accelerate right to catch it.
        # Force = Mass * Accel. So Force should be positive.
        
        x = np.array([0.0, 0.0, 0.1, 0.0]) # Tipping right
        
        _, _, _, _, opt_input = self.controller.get_input(x)
        u = opt_input[0]
        
        # This depends on the sign convention.
        # In our model:
        # theta_dd approx g/l * theta - 1/(l*M) * u
        # To reduce positive theta, we need negative theta_dd.
        # So -1/(l*M) * u should be negative => u should be positive.
        
        self.assertGreater(u, 0.0)

    def test_zero_state(self):
        """Test that zero state produces near-zero input."""
        x = np.array([0.0, 0.0, 0.0, 0.0])
        
        _, _, _, _, opt_input = self.controller.get_input(x)
        u = opt_input[0]
        
        self.assertAlmostEqual(u, 0.0, places=2)

if __name__ == '__main__':
    unittest.main()
