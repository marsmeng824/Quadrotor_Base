import numpy as np
import matplotlib.pyplot as plt

# Initial Paramater

dt = 0.1  #  Step Time(s)
T = 60  # Total time (s)
steps = int(T / dt)
m=1

# PID Parameter（）
Kp = 1
Kd = 1

# target position
z_target = 10

# Initial state
z = 0  # Position
v = 0  # Velocity
e_prev = 0  # previous error


# record data
time = np.linspace(0, T, steps)
z_history = np.zeros(steps)
u_history = np.zeros(steps)

# iteration
for i in range(steps):
    # error
    e = z_target - z
    e_diff = (e - e_prev) / dt
    
    #  F 
    F = Kp * e + Kd * e_diff 
    
    # state（F = ma，a = F）
    a = F  # m=1
    v += a * dt
    z += v * dt

    # 
    z_history[i] = z
    u_history[i] = F
    e_prev = e  

    #print F
    print(f"Time: {i*dt:.2f} s, F: {F:.4f} N")

# plot
plt.figure(figsize=(10, 5))

# position curve
plt.subplot(2, 1, 1)
plt.plot(time, z_history, label="Position x (m)", color="b")
plt.axhline(y=z_target, color="r", linestyle="--", label="Target Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.title("PID pisition chart")

# 控制力曲线
plt.subplot(2, 1, 2)
plt.plot(time, u_history, label="Control Force F (N)", color="g")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.legend()
plt.title("PID chart")

plt.tight_layout()
plt.show()
