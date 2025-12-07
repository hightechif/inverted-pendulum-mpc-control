import numpy as np
from dynamics.inverted_pendulum_dynamics import PendulumConfig, get_model_matrix

def create_dynamics_model():
    # Create a configuration object
    config = PendulumConfig()
    
    # Generate matrices
    A, B = get_model_matrix(config)
    
    print("Discrete Matrix A:\n", A)
    print("\nDiscrete Matrix B:\n", B)
    
    # Type check verification (Manual check)
    assert isinstance(A, np.ndarray)
    assert A.shape == (4, 4)

def setup_control():
    pass

def simulate_control_system():
    pass

def main():
    create_dynamics_model()

if __name__ == "__main__":
    main()