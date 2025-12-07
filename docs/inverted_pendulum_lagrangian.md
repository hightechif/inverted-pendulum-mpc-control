# System Dynamics of Inverted Pendulum on a Cart with Lagrangian Derivation

## 1. System Description

Diagram:

```ascii
                        ^  y (up)
                        |
                        |           o <-- pendulum (mass m)
                        | θ(theta) /
                        |         /
                        |        /
                        |       /
                        |      /
                        |     / l
                        |    /
                        |   /
                        |  /
                        | /
       ------------------------------------
       |                                  |
F --> +|          CART (mass M)           |+ ---------- > x (right)
       |                                  |
       ------------------------------------
              0                    0
     =========================================
     /////////////////////////////////////////
```

We consider:

- Cart mass: $M$
- Pendulum mass: $m$
- Pendulum length: $l$
- Cart position: $x$
- Pendulum angle: $\theta$
- Force applied to cart: $F$

## 2. Kinematics

Pendulum center of mass:

$$ x_p = x + l\sin\theta $$

$$ y_p = l\cos\theta $$

Velocities:

$$ \dot{x}\_p = \dot{x} + l\dot{\theta}\cos\theta $$

$$ \dot{y}\_p = -l\dot{\theta}\sin\theta $$

## 3. Energies

### Kinetic Energy

Cart:

$$ T\_{cart} = \tfrac{1}{2} M \dot{x}^2 $$

Pendulum:

$$ T\_{pendulum} = \tfrac{1}{2} m(\dot{x}\_p^2 + \dot{y}\_p^2) $$

Total kinetic energy:

$$
T = T_{cart} + T_{pendulum}
$$

$$
T = \tfrac{1}{2}(M+m)\dot{x}^2
+ m l \dot{x}\dot{\theta}\cos\theta
+ \tfrac{1}{2} m l^2 \dot{\theta}^2
$$

### Potential Energy

$$
V = m g l \cos\theta
$$

## 4. Lagrangian

$$
\mathcal{L} = T - V
$$

$$
\mathcal{L} =
\tfrac{1}{2}(M+m)\dot{x}^2
+ m l \dot{x}\dot{\theta}\cos\theta
+ \tfrac{1}{2} m l^2 \dot{\theta}^2
- m g l \cos\theta
$$

## 5. Euler–Lagrange Equations

General form:

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right) -
\frac{\partial \mathcal{L}}{\partial q}
= Q
$$

where:

$$
q =
\begin{bmatrix}
x \\
\theta
\end{bmatrix},

\dot{q}=
\begin{bmatrix}
\dot{x} \\
\dot{\theta}
\end{bmatrix},

Q = \begin{bmatrix} Q_x \\ Q_\theta \end{bmatrix}
$$

We got:

$$
\frac{\partial \mathcal{L}}{\partial \dot{q}}
=
\begin{bmatrix}
\frac{\partial \mathcal{L}}{\partial \dot{x}} \\
\frac{\partial \mathcal{L}}{\partial \dot{\theta}}
\end{bmatrix}
=
\begin{bmatrix}
(M+m)\dot{x} + m l \dot{\theta} \cos\theta \\
m l \dot{x}\cos\theta + m l^2 \dot{\theta}
\end{bmatrix}
$$

Time derivative for Euler–Lagrange:

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right)
=
\begin{bmatrix}
(M+m)\ddot{x} + m l\left(\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta \right) \\
m l\left(\ddot{x} \cos\theta - \dot{x}\dot{\theta} \sin\theta\right) + m l^2 \ddot{\theta}
\end{bmatrix}
$$

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right)
=
\begin{bmatrix}
(M+m)\ddot{x} + m l\left(\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta \right) \\
ml\ddot{x} \cos\theta - ml\dot{x}\dot{\theta} \sin\theta + m l^2 \ddot{\theta}
\end{bmatrix}
$$

and

$$
\frac{\partial \mathcal{L}}{\partial q}
=
\begin{bmatrix}
0 \\
- m l \dot{x}\dot{\theta} \sin\theta
+ m g l \sin\theta
\end{bmatrix}
$$

The equation becomes:

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right) -
\frac{\partial \mathcal{L}}{\partial q}
= \begin{bmatrix}
(M+m)\ddot{x} + m l\left(\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta\right) \\
ml\ddot{x} \cos\theta - ml\dot{x}\dot{\theta} \sin\theta + m l^2 \ddot{\theta} + m l \dot{x}\dot{\theta} \sin\theta
- m g l \sin\theta
\end{bmatrix}
$$

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right) -
\frac{\partial \mathcal{L}}{\partial q}
= \begin{bmatrix}
(M+m)\ddot{x} + m l\left(\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta\right) \\
ml\ddot{x} \cos\theta + m l^2 \ddot{\theta} - m g l \sin\theta
\end{bmatrix}
$$

and then

$$
\begin{bmatrix}
(M+m)\ddot{x} + m l\left(\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta\right) \\
ml\ddot{x} \cos\theta + m l^2 \ddot{\theta} - m g l \sin\theta
\end{bmatrix}
= \begin{bmatrix} Q_x \\ Q_\theta \end{bmatrix}
$$

Generalized forces:

$$ Q_x = F $$

$$ Q\_\theta = 0 $$

So, the final equations are:

1. Cart Equation of Motion

$$
(M+m)\ddot{x}
+ m l \ddot{\theta}\cos\theta
- m l \dot{\theta}^2\sin\theta
= F
$$

2. Pendulum Equation of Motion

$$
m l \ddot{x} \cos\theta
+ m l^2 \ddot{\theta}
- m g l \sin\theta = 0
$$

since $m \not= 0$ and $l \not= 0$, we can minize the equation into

$$
\ddot{x} \cos\theta
+ l\ddot{\theta}
- g\sin\theta = 0
$$

Final Dynamic Equations

$$
(M+m)\ddot{x}
+ m l \ddot{\theta}\cos\theta
- m l \dot{\theta}^2\sin\theta = F
$$

$$
\ddot{x}\cos\theta
+ l\ddot{\theta}
= g\sin\theta
$$

---

## 6. Linearization

Nonlinear equations:

$$
(M+m)\ddot{x} + m l \ddot{\theta} \cos\theta - m l \dot{\theta}^2 \sin\theta = F
$$

$$
\ddot{x}\cos\theta
+ l\ddot{\theta}
= g\sin\theta
$$

Linearize at $\theta \to 0$:

- $\sin\theta \approx \theta$
- $\cos\theta \approx 1$
- $\dot{\theta}^2 \to 0$

Result:

$$
(M+m)\ddot{x} + m l \ddot{\theta} = F
$$

$$
\ddot{x} + l\ddot{\theta} = g\theta
$$

Solution for $\ddot{x}$ and $\ddot{\theta}$:

$$
\ddot{x} = \frac{1}{M}F - \frac{m g}{M}\theta
$$

$$
\ddot{\theta} = \frac{g(M+m)}{l M}\theta - \frac{1}{l M}F
$$

---

## 5. State-Space Representation

We define the state vector $\mathbf{x}$ and the input $u$:

$$
\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \end{bmatrix}, \quad u = F
$$

The first-order derivatives are:

- $\dot{x}_1 = \dot{x} = x_2$
- $\dot{x}_2 = \ddot{x} = -\frac{mg}{M}x_3 + \frac{1}{M}u$
- $\dot{x}_3 = \dot{\theta} = x_4$
- $\dot{x}_4 = \ddot{\theta} = \frac{(M+m)g}{Ml}x_3 - \frac{1}{Ml}u$

We can now write the system in the standard matrix form:

$$
\dot{\mathbf{x}} = A\mathbf{x} + Bu
$$

$$
y = C\mathbf{x} + Du
$$

### The System Matrix (A)

This matrix determines the internal dynamics of the system.

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & -\frac{mg}{M} & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & \frac{(M+m)g}{Ml} & 0
\end{bmatrix}
$$

### The Input Matrix (B)

This matrix determines how the force $F$ affects the states.

$$
B = \begin{bmatrix}
0 \\
\frac{1}{M} \\
0 \\
-\frac{1}{Ml}
\end{bmatrix}
$$

### The Output Matrix (C) and Feedforward Matrix (D)

Assuming we can measure both the cart position $x$ and the pendulum angle $\theta$:

$$
C = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0
\end{bmatrix}, \quad
D = \begin{bmatrix}
0 \\
0
\end{bmatrix}
$$

### Summary of Stability

Notice the element $A_{4,3} = \frac{(M+m)g}{Ml}$. Since $M, m, g, l$ are all positive, this term is positive. In a simplified 2nd order characteristic equation $s^2 - A_{4,3} = 0$, this yields poles at $s = \pm \sqrt{A_{4,3}}$.The presence of a positive real pole confirms that the upright equilibrium is unstable without control.

---

## 8. Discretization: Forward Euler Method

Computers cannot calculate continuous derivatives ($\dot{x}$); they calculate in discrete time steps ($\Delta t$). The Forward Euler Method approximates the continuous system for digital simulation.

We approximate velocity ($\dot{x}$) as the slope between two time steps:

$$
\dot{x} \approx \frac{x_{k+1} - x_k}{\Delta t}
$$

Substitute this into the continuous state-space equation ($\dot{x} = Ax + Bu$):

$$
\frac{x_{k+1} - x_k}{\Delta t} = A x_k + B u_k
$$

Solve for the next state ($x_{k+1}$):

$$
x_{k+1} - x_k = (A \cdot \Delta t) x_k + (B \cdot \Delta t) u_k
$$

$$
x_{k+1} = x_k + (A \cdot \Delta t) x_k + (B \cdot \Delta t) u_k
$$

Factor out $x_k$ (using Identity matrix $I$):

$$
x_{k+1} = (I + A \cdot \Delta t) x_k + (B \cdot \Delta t) u_k
$$

---

## 9. Software Implementation

Here is an example of the model implementation in Python code:

```python
import numpy as np

# ==========================================
# 1. System Parameters
# ==========================================
l_bar = 2.0   # length of bar [m]
M = 1.0       # mass of cart [kg]
m = 0.3       # mass of pendulum [kg]
g = 9.8       # gravity [m/s^2]

# Control & Simulation Parameters
nx = 4        # number of states [x, v, theta, omega]
nu = 1        # number of inputs [Force]
Q = np.diag([0.0, 1.0, 1.0, 0.0])  # state cost matrix (for LQR/MPC)
R = np.diag([0.01])                # input cost matrix (for LQR/MPC)

T = 30           # Horizon length
delta_t = 0.02    # time tick [s]
sim_time = 5.0   # simulation time [s]

# ==========================================
# 2. Mathematical Model Implementation
# ==========================================
def get_model_matrix():
    """
    Returns the linearized, discretized State-Space matrices A and B
    based on the Lagrangian derivation.
    """

    # Continuous Time A Matrix (System Dynamics)
    # Rows: [dx/dt, d(dx)/dt, dtheta/dt, d(dtheta)/dt]
    A_cont = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, (-1.0) * m * g / M, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, g * (M + m) / (l_bar * M), 0.0]
    ])

    # Continuous Time B Matrix (Input Dynamics)
    # Effect of Force F on the states
    B_cont = np.array([
        [0.0],
        [1.0 / M],
        [0.0],
        [(-1.0) / (l_bar * M)]
    ])

    # Discretization (Forward Euler Method)
    # x[k+1] = (I + A*dt)x[k] + (B*dt)u[k]
    A_discrete = np.eye(nx) + delta_t * A_cont
    B_discrete = delta_t * B_cont

    return A_discrete, B_discrete
```

---

## 10. Conclusion

In conclusion, the Lagrangian formulation offers a distinct advantage over Newtonian mechanics for complex systems by utilizing energy scalars rather than vector forces, often simplifying the derivation process. Furthermore, converting these nonlinear dynamics into a linearized state-space model is a critical step for engineering applications. This standardized matrix format not only facilitates stability analysis but also enables the efficient implementation of modern control algorithms in software.
