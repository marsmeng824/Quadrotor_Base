import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from controller.QP_MPC import get_optimal_acceleration
from transform3d import acc_to_euler

# ==== 参数 ====
dt = 0.1
steps = 10
N = 3

# ==== 物理参数 ====
J = np.diag([0.0411, 0.0478, 0.0599])
J_inv = np.linalg.inv(J)

# ==== CasADi 模型定义 ====
x = ca.SX.sym('x', 6)
u = ca.SX.sym('u', 3)

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

omega = x[3:6]
omega_cross = ca.vertcat(
    ca.horzcat(0, -omega[2], omega[1]),
    ca.horzcat(omega[2], 0, -omega[0]),
    ca.horzcat(-omega[1], omega[0], 0)
)
domega = J_inv @ (u - omega_cross @ J @ omega)
att_dot = T_inv(x[0], x[1]) @ omega
x_dot = ca.vertcat(att_dot, domega)
f = ca.Function('f', [x, u], [x_dot])

X = ca.SX.sym('X', 6, N+1)
U = ca.SX.sym('U', 3, N)
P = ca.SX.sym('P', 6 + 3)

Q = np.diag([10, 10, 10, 0, 0, 0])
R = np.diag([0.1, 0.1, 0.1])
cost = 0
constraints = []

for k in range(N):
    x_k = X[:, k]
    u_k = U[:, k]
    x_next_pred = x_k + dt * f(x_k, u_k)
    constraints.append(X[:, k+1] - x_next_pred)
    e = x_k[0:3] - P[6:9]
    cost += ca.mtimes([e.T, Q[0:3, 0:3], e]) + ca.mtimes([u_k.T, R, u_k])

e_terminal = X[0:3, -1] - P[6:9]
cost += ca.mtimes([e_terminal.T, Q[0:3, 0:3], e_terminal])

opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
opt_constraints = ca.vertcat(*constraints)
nlp = {'x': opt_vars, 'f': cost, 'g': opt_constraints, 'p': P}
solver = ca.nlpsol('solver', 'ipopt', nlp, {'ipopt.print_level': 0, 'print_time': 0})

# ==== 初始状态 ====
x0_pos = np.array([0, 0, 0, 0, 0, 0])
x_ref_pos = np.array([1, 0, 1, 0, 1, 0])
yaw_des = 0.0
att_state = np.zeros(6)

# ==== 记录变量 ====
euler_log = []
acc_log = []
u_log = []

# ==== 主循环 ====
for _ in range(steps):
    acc_des = get_optimal_acceleration(x0_pos, x_ref_pos)
    euler_target = acc_to_euler(acc_des, yaw_des)

    acc_log.append(acc_des)
    euler_log.append(euler_target)

    # MPC 控制求解
    p = np.concatenate([att_state, euler_target])
    x_guess = np.tile(att_state.reshape(6, 1), (1, N+1))
    u_guess = np.zeros((3, N))
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
        u0 = np.zeros(3)

    u_log.append(u0)
    att_state += dt * f(att_state, u0).full().flatten()

    # 积分得到下一步位置（简化模拟）
    x0_pos[1] += acc_des[0] * dt
    x0_pos[0] += x0_pos[1] * dt
    x0_pos[3] += acc_des[1] * dt
    x0_pos[2] += x0_pos[3] * dt
    x0_pos[5] += acc_des[2] * dt
    x0_pos[4] += x0_pos[5] * dt

# ==== 可视化 ====
euler_log = np.array(euler_log)
acc_log = np.array(acc_log)
u_log = np.array(u_log)

plt.figure()
plt.plot(np.rad2deg(euler_log[:,0]), label='Roll (deg)')
plt.plot(np.rad2deg(euler_log[:,1]), label='Pitch (deg)')
plt.plot(np.rad2deg(euler_log[:,2]), label='Yaw (deg)')
plt.title("Euler Angle Target from QP + Transform")
plt.xlabel("Time Step")
plt.ylabel("Angle (deg)")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.figure()
plt.plot(u_log)
plt.title("Control Torque from CasADi MPC")
plt.xlabel("Time Step")
plt.ylabel("Torque (tau)")
plt.legend(['tau_x', 'tau_y', 'tau_z'])
plt.grid(True)
plt.tight_layout()

plt.show()