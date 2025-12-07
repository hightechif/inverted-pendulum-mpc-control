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
T = \tfrac{1}{2}(M+m)\dot{x}^2 + m l \dot{x}\dot{\theta}\cos\theta + \tfrac{1}{2} m l^2 \dot{\theta}^2
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
\mathcal{L} = \tfrac{1}{2}(M+m)\dot{x}^2 + m l \dot{x}\dot{\theta}\cos\theta + \tfrac{1}{2} m l^2 \dot{\theta}^2 - m g l \cos\theta
$$

## 5. Euler–Lagrange Equations

General form:

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right) - \frac{\partial \mathcal{L}}{\partial q} = Q
$$

where:

$$
q = \begin{bmatrix}x \\
\theta
\end{bmatrix}, \quad
\dot{q}=
\begin{bmatrix}
\dot{x} \\
\dot{\theta}
\end{bmatrix} \quad
Q = \begin{bmatrix} Q_x \\
Q_\theta
\end{bmatrix}
$$

We got:

$$
\frac{\partial \mathcal{L}}{\partial \dot{q}} = \begin{bmatrix}\frac{\partial \mathcal{L}}{\partial \dot{x}} \\
\frac{\partial \mathcal{L}}{\partial \dot{\theta}} \end{bmatrix}
= \begin{bmatrix}
(M+m)\dot{x} + m l \dot{\theta} \cos\theta \\
m l \dot{x}\cos\theta + m l^2 \dot{\theta}
\end{bmatrix}
$$

Time derivative for Euler–Lagrange:

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right)
= \begin{bmatrix}
(M+m)\ddot{x} + m l\left(\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta \right) \\
m l\left(\ddot{x} \cos\theta - \dot{x}\dot{\theta} \sin\theta\right) + m l^2 \ddot{\theta}
\end{bmatrix}
$$

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right)
= \begin{bmatrix}
(M+m)\ddot{x} + m l\left(\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta \right) \\
ml\ddot{x} \cos\theta - ml\dot{x}\dot{\theta} \sin\theta + m l^2 \ddot{\theta}
\end{bmatrix}
$$

and

$$
\frac{\partial \mathcal{L}}{\partial q} = \begin{bmatrix}
0 \\ - m l \dot{x}\dot{\theta} \sin\theta + m g l \sin\theta
\end{bmatrix}
$$

The equation becomes:

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right) -
\frac{\partial \mathcal{L}}{\partial q} = \begin{bmatrix} (M+m)\ddot{x} + m l\left(\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta\right) \\
ml\ddot{x} \cos\theta - ml\dot{x}\dot{\theta} \sin\theta + m l^2 \ddot{\theta} + m l \dot{x}\dot{\theta} \sin\theta - m g l \sin\theta
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
\end{bmatrix} = \begin{bmatrix} Q_x \\
Q_\theta \end{bmatrix}
$$

Generalized forces:

$$ Q*x = F, \quad Q*\theta = 0 $$

So, the final equations are:

1. Cart Equation of Motion

$$
(M+m)\ddot{x} + m l \ddot{\theta}\cos\theta - m l \dot{\theta}^2\sin\theta = F
$$

2. Pendulum Equation of Motion

$$
m l \ddot{x} \cos\theta + m l^2 \ddot{\theta} - m g l \sin\theta = 0
$$

since $m \not= 0$ and $l \not= 0$, we can minize the equation into

$$
\ddot{x} \cos\theta + l\ddot{\theta} - g\sin\theta = 0
$$

Final Dynamic Equations

$$
(M+m)\ddot{x} + m l \ddot{\theta}\cos\theta - m l \dot{\theta}^2\sin\theta = F
$$

$$
\ddot{x}\cos\theta + l\ddot{\theta} = g\sin\theta
$$

---

## 6. Linearization

Nonlinear equations:

$$
(M+m)\ddot{x} + m l \ddot{\theta} \cos\theta - m l \dot{\theta}^2 \sin\theta = F
$$

$$
\ddot{x}\cos\theta + l\ddot{\theta} = g\sin\theta
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

State-Space Diagram:

```ascii
  u   +---+  (Bu)   +---+ (x_dot) +---+       x
----->| B |-------->| + |-------->| ∫ |-------------->
      +---+         +---+         +---+       |
                      ^                       |
                      |                       |
                      |    (Ax)   +---+       v
                      +-----------| A |<------+
                                  +---+
```

Here is the step-by-step breakdown of the State-Space Representation equation for the inverted pendulum. The core equation is:

$$
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
$$

### Step 1: The State Vector ($\mathbf{x}$)

The "State" is a snapshot of exactly what the system is doing at a specific moment. For the pendulum, we need 4 numbers to fully describe it:

$$
\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix} \begin{aligned} & \leftarrow \text{Cart Position} \\ & \leftarrow \text{Cart Velocity} \\ & \leftarrow \text{Pendulum Angle} \\ & \leftarrow \text{Pendulum Angular Velocity} \end{aligned}
$$

### Step 2: The Derivative Vector ($\dot{\mathbf{x}}$)

This represents how the state is changing. It is simply the time derivative of the vector above.

$$
\dot{\mathbf{x}} = \begin{bmatrix} \dot{x} \\ \ddot{x} \\ \dot{\theta} \\ \ddot{\theta} \end{bmatrix} \begin{aligned} & \leftarrow \text{Velocity (change in position)} \\ & \leftarrow \text{Acceleration (change in velocity)} \\ & \leftarrow \text{Angular Velocity (change in angle)} \\ & \leftarrow \text{Angular Accel (change in ang. vel)} \end{aligned}
$$

### Step 3: The System Matrix ($A$) - "Internal Physics"

The $A$ matrix tells us how the system behaves naturally if no external force is applied. It connects the current state ($\mathbf{x}$) to the changes ($\dot{\mathbf{x}}$).

$$
A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & -\frac{mg}{M} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & \frac{(M+m)g}{Ml} & 0 \end{bmatrix}
$$

Let's multiply the first row of $A$ by $\mathbf{x}$ to see what it means:

- Row 1 (Kinematics):

$$
\dot{x} = (0)x + (1)\dot{x} + (0)\theta + (0)\dot{\theta} \implies \mathbf{\dot{x} = \dot{x}}
$$

Translation: "The change in position is equal to the velocity." (This is just a definition).

- Row 2 (Dynamics):

$$
\ddot{x} = (0)x + (0)\dot{x} + (-\frac{mg}{M})\theta + (0)\dot{\theta}
$$

Translation: "The cart's acceleration depends on the pendulum angle." (As the pendulum falls, it pushes the cart).

- Row 3 (Kinematics):

$$
\dot{\theta} = (0)x + (0)\dot{x} + (0)\theta + (1)\dot{\theta} \implies \mathbf{\dot{\theta} = \dot{\theta}}
$$

Translation: "The change in angle is equal to the angular velocity."

- Row 4 (Dynamics):

$$
\ddot{\theta} = (0)x + (0)\dot{x} + (\frac{(M+m)g}{Ml})\theta + (0)\dot{\theta}
$$

Translation: "The angular acceleration is driven by gravity pulling on the angle." (This is the instability term).

### Step 4: The Input Matrix ($B$) - "External Force"

The $B$ matrix tells us how the external input $\mathbf{u}$ (Force $F$) affects the system.

$$
B = \begin{bmatrix} 0 \\ \frac{1}{M} \\ 0 \\ -\frac{1}{Ml} \end{bmatrix}
$$

Let's look at how the Input ($u$) adds to the equation:

- Row 1 & 3 (Zeros): Force does not directly change position or angle. It only causes acceleration, which eventually changes position/angle.

- Row 2 (Cart Acceleration):

$$
\ddot{x}_{new} = \ddot{x}_{old} + (\frac{1}{M}) u
$$

Translation: Newton's Law ($F=ma \rightarrow a = F/m$). Pushing the cart accelerates it.

- Row 4 (Pendulum Acceleration):

$$
\ddot{\theta}_{new} = \ddot{\theta}_{old} + (-\frac{1}{Ml}) u
$$

Translation: Pushing the cart creates a "reaction torque" on the pendulum rod, causing it to rotate in the opposite direction.

### Summary Equation

Putting it all together, the matrix equation $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$ is just a compact way of writing these four linear equations at once:

1. $\text{Velocity} = \text{Velocity}$
2. $\text{Cart Accel} = (\text{Gravity effects}) + (\text{Force effects})$
3. $\text{Ang. Velocity} = \text{Ang. Velocity}$
4. $\text{Ang. Accel} = (\text{Gravity effects}) + (\text{Reaction Force effects})$

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

[source](../dynamics/inverted_pendulum_dynamics.py)

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

T = 30           # Horizon length
delta_t = 0.02    # time tick [s]

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
