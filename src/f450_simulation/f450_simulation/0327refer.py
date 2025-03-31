import numpy as np
import matplotlib.pyplot as plt

# Sampling time
Ts = 0.1

# System matrices (this is a discrete double integrator)
A = np.array([[1, Ts],
              [0, 1]])
B = np.array([[0.5 * Ts**2],
              [Ts]])
C = np.array([[1, 0],
              [0, 1]])
D = np.array([[0],
              [0]])

# System dimensions
m = B.shape[1]  # Input dimension
p = C.shape[0]  # Output dimension

# Initial state
x0 = np.array([[0],
               [0]])

# System evolution dynamics
def dynamics(x, u):
    x_next = A @ x + B @ u
    y = C @ x + D @ u
    return x_next, y

# Simulation parameters
T_end = 5  # seconds to simulate
N = int(T_end / Ts)  # number of time steps
u_signal = np.ones((N, m))  # Unit step input

# Initialize arrays
x = np.zeros((2, N + 1))
y = np.zeros((p, N))
x[:, [0]] = x0

# Simulate
for k in range(N):
    x_k = x[:, [k]]
    u_k = u_signal[k].reshape(m, 1)
    x_next, y_k = dynamics(x_k, u_k)
    x[:, [k + 1]] = x_next
    y[:, [k]] = y_k

# Plot results
time = np.linspace(0, T_end, N)
plt.figure()
plt.plot(time, y[0, :].T, label="Position")
plt.plot(time, y[1, :].T, label="Velocity")
plt.xlabel('Time [s]')
plt.ylabel('Output')
plt.title('Double Integrator Response to Step Input')
plt.legend()
plt.grid(True)
plt.show()

#%%

import numpy as np
import matplotlib.pyplot as plt

# Parameters
Ts = 0.1          # Sampling time [s]
g = 9.81          # Gravity [m/s^2]
m = 1.5           # Mass of the quadrotor [kg]
hover_thrust = m * g  # Thrust required to hover [N]

# Linearized around hover: delta_u = u - mg
# So: d²z = (1/m) * delta_u

# Discrete-time state-space (double integrator)
A = np.array([[1, Ts],
              [0, 1]])
B = np.array([[0.5 * Ts**2 / m],
              [Ts / m]])
C = np.array([[1, 0],
              [0, 1]])  # Output: altitude and velocity
D = np.array([[0],
              [0]])

# Initial state (at rest at z = 0)
x0 = np.array([[0],
               [0]])

# Simulation setup
T_end = 10
N = int(T_end / Ts)

# Input: small thrust bump (e.g. delta_u = 1 N for first second)
delta_u = np.zeros((N, 1))
delta_u[0:10] = 1.0  # 1 N thrust for 1 second

# Initialize
x = np.zeros((2, N + 1))
y = np.zeros((2, N))
x[:, [0]] = x0

# Simulate
def dynamics(x, u):
    x_next = A @ x + B @ u
    y = C @ x + D @ u
    return x_next, y

for k in range(N):
    x_k = x[:, [k]]
    u_k = delta_u[k].reshape(1, 1)
    x_next, y_k = dynamics(x_k, u_k)
    x[:, [k + 1]] = x_next
    y[:, [k]] = y_k

# Plot
time = np.linspace(0, T_end, N)
plt.figure()
plt.plot(time, y[0, :].T, label="Altitude [m]")
plt.plot(time, y[1, :].T, label="Vertical Velocity [m/s]")
plt.xlabel('Time [s]')
plt.ylabel('Output')
plt.title('Quadrotor Altitude Response')
plt.legend()
plt.grid(True)
plt.show()
# %%
