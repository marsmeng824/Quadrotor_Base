import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

# =====Parameter Setting=====
J = np.diag([0.0411, 0.0478, 0.0599])
J_inv = np.linalg.inv(J)

#step time
dt = 0.1

#Preditive horizon
N = 3  

# ===== CasADi State Variance Definition =====
x = ca.SX.sym('x', 6)   # state：[roll, pitch, yaw, p, q, r]
u = ca.SX.sym('u', 3)   # input：[tau_x, tau_y, tau_z]

# === T_inv(roll, pitch) ===
def T_inv(roll, pitch):
    sin_r = ca.sin(roll)
    cos_r = ca.cos(roll)
    tan_p = ca.tan(pitch)
    sec_p = 1 / ca.cos(pitch)
    return ca.vertcat(
        ca.horzcat(1, sin_r * tan_p, cos_r * tan_p),
        ca.horzcat(0, cos_r, -sin_r),
        ca.horzcat(0, sin_r * sec_p, cos_r * sec_p)
    )

# === function：angular acceleration ===
omega = x[3:6]
omega_cross = ca.vertcat(
    ca.horzcat(0, -omega[2], omega[1]),
    ca.horzcat(omega[2], 0, -omega[0]),
    ca.horzcat(-omega[1], omega[0], 0)
)
domega = ca.mtimes(J_inv, (u - ca.mtimes(omega_cross, ca.mtimes(J, omega))))

# === function：the derivation of attitude ===
Tinv = T_inv(x[0], x[1])
att_dot = ca.mtimes(Tinv, omega)
att_dot = ca.vertcat(att_dot[0], att_dot[1], 0)

# === kinematics and dynamics ===
x_dot = ca.vertcat(att_dot, domega)
f = ca.Function('f', [x, u], [x_dot])

# ===== create controller =====
X = ca.SX.sym('X', 6, N+1)  # State trajectory
U = ca.SX.sym('U', 3, N)    # input trajectory
P = ca.SX.sym('P', 6 + 3)   # initial condition


cost = 0
constraints = []
Q = np.diag([10, 10, 10, 0, 0, 0])
R = np.diag([0.1, 0.1, 0.1])

for k in range(N):
    x_k = X[:, k]
    u_k = U[:, k]
    x_next = X[:, k+1]
    x_next_pred = x_k + dt * f(x_k, u_k)
    constraints.append(x_next - x_next_pred)

    # cost function target: P[6:9]
    e = x_k[0:3] - P[6:9]
    cost += ca.mtimes([e.T, Q[0:3,0:3], e]) + ca.mtimes([u_k.T, R, u_k])


# terminal cost
e_terminal = X[0:3, -1] - P[6:9]
cost += ca.mtimes([e_terminal.T, Q[0:3,0:3], e_terminal])

opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
opt_params = P
opt_constraints = ca.vertcat(*constraints)

nlp = {'x': opt_vars, 'f': cost, 'g': opt_constraints, 'p': opt_params}
solver = ca.nlpsol('solver', 'ipopt', nlp, {'ipopt.print_level': 0, 'print_time': 0})

# ===== simulator =====
def simulate(initial_angle, target_angle, sim_time=5.0):
    x0 = np.concatenate([initial_angle, np.zeros(3)])
    target = target_angle
    X_sim = [x0.copy()]
    u_log = []
    t_steps = int(sim_time / 0.2)

    for _ in range(t_steps):
        # state+target
        p = np.concatenate([x0, target])
        x_guess = np.tile(x0.reshape(6,1), (1,N+1))
        u_guess = np.zeros((3,N))

        vars_init = np.concatenate([x_guess.flatten(), u_guess.flatten()])
        lbx = -np.inf * np.ones_like(vars_init)
        ubx = np.inf * np.ones_like(vars_init)

        lbg = np.zeros(opt_constraints.shape[0])
        ubg = np.zeros(opt_constraints.shape[0])

        sol = solver(x0=vars_init, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

        if solver.stats()['success']:
            sol_x = sol['x'].full().flatten()
            u0 = sol_x[6*(N+1):6*(N+1)+3]
        else:
            u0 = np.zeros(3)  # fallback：no torque


        # simulator
        x0 = x0 + dt * f(x0, u0).full().flatten()
        X_sim.append(x0.copy())
        u_log.append(u0)

    return np.array(X_sim),np.array(u_log)

# ===== result =====
initial = np.deg2rad([0, 0, 0])
target  = np.deg2rad([20, 10, 0])
result, u_record = simulate(initial, target)

plt.plot(np.rad2deg(result[:,0]), label='Roll')
plt.plot(np.rad2deg(result[:,1]), label='Pitch')
plt.plot(np.rad2deg(result[:,2]), label='Yaw')
plt.axhline(20, color='gray', ls='--')
plt.axhline(10, color='gray', ls='--')
plt.axhline(0, color='gray', ls='--')
plt.title("Euler Angle Tracking with Nonlinear MPC (CasADi)")
plt.xlabel("Time Step")
plt.ylabel("Angle (deg)")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.figure()
plt.plot(u_record[:,0], label='tau_x')
plt.plot(u_record[:,1], label='tau_y')
plt.plot(u_record[:,2], label='tau_z')
plt.title("Control Torque (tau) over Time")
plt.xlabel("Time Step")
plt.ylabel("Torque")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()