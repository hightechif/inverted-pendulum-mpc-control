import unittest
import numpy as np
from dynamics.inverted_pendulum_dynamics import PendulumConfig
from simulation.simulation import simulation

class TestSimulation(unittest.TestCase):
    def setUp(self):
        self.config = PendulumConfig()

    def test_simulation_1d_input(self):
        """Test that simulation handles 1D input array correctly."""
        # State: [p, p_dot, theta, theta_dot]
        x = np.array([0.0, 0.0, 0.1, 0.0])
        u = 0.0
        dt = 0.01
        
        x_next = simulation(x, u, dt, self.config)
        
        # Should return 1D array
        self.assertIsInstance(x_next, np.ndarray)
        self.assertEqual(x_next.ndim, 1)
        self.assertEqual(x_next.shape, (4,))

    def test_simulation_2d_input(self):
        """Test that simulation handles 2D input array (column vector) correctly."""
        # State: [p, p_dot, theta, theta_dot] as column vector
        x = np.array([[0.0], [0.0], [0.1], [0.0]])
        u = 0.0
        dt = 0.01
        
        x_next = simulation(x, u, dt, self.config)
        
        # Should return 2D array (column vector)
        self.assertIsInstance(x_next, np.ndarray)
        self.assertEqual(x_next.ndim, 2)
        self.assertEqual(x_next.shape, (4, 1))

    def test_integration_occurs(self):
        """Test that the state actually changes during simulation."""
        x = np.array([0.0, 0.0, 0.1, 0.0])
        u = 0.0
        dt = 0.1 # Larger step to ensure visible change
        
        x_next = simulation(x, u, dt, self.config)
        
        # State should not be identical (gravity acts on pendulum)
        self.assertFalse(np.array_equal(x, x_next))
        # Theta should increase (falling)
        self.assertGreater(x_next[2], x[2])

if __name__ == '__main__':
    unittest.main()
