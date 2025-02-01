import numpy as np
import matplotlib.pyplot as plt

# 物理参数
g = 9.81  # Gravity Acceleration (m/s^2)
dt = 0.01  #  Step Time(s)
T = 10  # Total time (s)
steps = int(T / dt)
m=1

# PID 参数（使用 Ziegler-Nichols 方法计算）
Kp = 10
Kd = 30

# target position
z_target = 10

# Initial state
z = 0  # Position
v = 0  # Velocity
e_prev = 0  # 之前的误差


# 记录数据
time = np.linspace(0, T, steps)
z_history = np.zeros(steps)
u_history = np.zeros(steps)

# 仿真循环
for i in range(steps):
    # 计算误差
    e = z_target - z
    e_diff = (e - e_prev) / dt
    
    # 计算控制力 F (PID 控制 + 重力补偿)
    F = Kp * e + Kd * e_diff + g
    
    # 更新系统状态（F = ma，a = F）
    a = F-g  # 这里 m=1，所以 F 直接就是加速度
    v += a * dt
    z += v * dt

    # 记录数据
    z_history[i] = z
    u_history[i] = F
    e_prev = e  # 记录上次误差

# 画图
plt.figure(figsize=(10, 5))

# 位置曲线
plt.subplot(2, 1, 1)
plt.plot(time, z_history, label="Position x (m)", color="b")
plt.axhline(y=z_target, color="r", linestyle="--", label="Target Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.title("PID 控制的物体位置变化")

# 控制力曲线
plt.subplot(2, 1, 2)
plt.plot(time, u_history, label="Control Force F (N)", color="g")
plt.axhline(y=g, color="r", linestyle="--", label="Gravity Compensation (N)")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.legend()
plt.title("PID 控制力随时间变化")

plt.tight_layout()
plt.show()
