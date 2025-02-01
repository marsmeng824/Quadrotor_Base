import numpy as np
import matplotlib.pyplot as plt

# weight
m = 1.0
# sample time
Ts= 0.1
# target position
X_target = 10
# total time
T = 10

# gain parameter
Kp = 4
Ki = 0
Kd = 4

#initial parameter
x = 0
v = 0
a = 0
error_prev = 0
integral = 0

# record datas
time = np.arange(0,T,Ts)
x_history = []
v_history = []
control_history = []

# control_loop
for i in time:
    error = X_target - x
    integral += error*Ts
    derivative = (error - error_prev) / Ts
    F = Kp * error + Ki * integral + Kd * derivative
    control_history.append(F)

    #state
    a=F/m
    v += a*Ts
    v_history.append(v)
    x += v*Ts
    x_history.append(x)

    error_prev = error

# 画图
plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(time, x_history, label="Position x")
plt.axhline(y=X_target, color="r", linestyle="--", label="Target Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(time, control_history, label="Control Force F")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.legend()
plt.grid()

plt.show()