import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.1):
        self.kp = np.array(kp)
        self.ki = np.array(ki)
        self.kd = np.array(kd)
        self.dt = dt
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)
    
    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class Quadrotor:
    def __init__(self):
        self.mass = 2.0  # kg
        self.g = 9.81  # gravity m/s^2
        self.J = np.diag([0.04, 0.04, 0.04])  # Moment of inertia matrix
        
        self.x_d, self.y_d, self.z_d = 3.0, 0.0, 4.0  # Target position
        self.current_state = {"x": 0, "y": 0, "z": 0, "phi": 0, "theta": 0, "psi": 0}
        
        self.pid_pos = PIDController(kp=[1,1,1], ki=[0.1,0.1,0.1], kd=[0.2,0.2,0.2])
        self.pid_att = PIDController(kp=[1,1,1], ki=[0.1,0.1,0.1], kd=[0.2,0.2,0.2])
    
    def control_loop(self):
        if self.current_state is None:
            return

        x, y, z = self.current_state["x"], self.current_state["y"], self.current_state["z"]
        phi, theta, psi = self.current_state["phi"], self.current_state["theta"], self.current_state["psi"]
        
        e_pos = np.array([self.x_d - x, self.y_d - y, self.z_d - z])
        x_acc, y_acc, z_acc = self.pid_pos.compute(e_pos)
        
        T = self.mass * (z_acc + self.g)
        phi_d = -np.arcsin(np.clip((self.mass / T) * y_acc, -1, 1))
        theta_d = np.arcsin(np.clip((self.mass / T) * x_acc, -1, 1))
        phi_d = np.clip(phi_d, -np.pi/6, np.pi/6)
        theta_d = np.clip(theta_d, -np.pi/6, np.pi/6)
        psi_d = 0.0
        
        e_att = np.array([phi_d - phi, theta_d - theta, psi_d - psi])
        torques = self.J @ self.pid_att.compute(e_att)
        
        R_world_body = np.array([
            [np.cos(psi) * np.cos(theta), np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi), np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi)],
            [np.sin(psi) * np.cos(theta), np.sin(psi) * np.sin(theta) * np.sin(phi) + np.cos(psi) * np.cos(phi), np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)],
            [-np.sin(theta), np.cos(theta) * np.sin(phi), np.cos(theta) * np.cos(phi)]
        ])
        
        thrust = R_world_body @ np.array([0, 0, T])
        return thrust, torques, phi, theta, psi

# Simulation
quad = Quadrotor()
time_steps = 500
dt = 0.1
positions = []
attitudes = []

for _ in range(time_steps):
    thrust, torques, phi, theta, psi = quad.control_loop()
    
    quad.current_state["x"] += thrust[0] * dt / quad.mass
    quad.current_state["y"] += thrust[1] * dt / quad.mass
    quad.current_state["z"] += (thrust[2] - quad.mass * quad.g) * dt / quad.mass
    
    quad.current_state["phi"] += torques[0] * dt / quad.J[0, 0]
    quad.current_state["theta"] += torques[1] * dt / quad.J[1, 1]
    quad.current_state["psi"] += torques[2] * dt / quad.J[2, 2]
    
    positions.append((quad.current_state["x"], quad.current_state["y"], quad.current_state["z"]))
    attitudes.append((quad.current_state["phi"], quad.current_state["theta"], quad.current_state["psi"]))

# Plot results
positions = np.array(positions)
attitudes = np.array(attitudes)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:,0], positions[:,1], positions[:,2], label='Quadrotor Path')
ax.scatter([quad.x_d], [quad.y_d], [quad.z_d], color='r', label='Target')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

# Plot position changes
fig, axs = plt.subplots(3, 1, figsize=(8, 6))
times = np.arange(time_steps) * dt
labels = ["X position", "Y position", "Z position"]
for i in range(3):
    axs[i].plot(times, positions[:, i], label=labels[i])
    axs[i].set_ylabel(labels[i])
    axs[i].legend()
axs[2].set_xlabel("Time (s)")
plt.tight_layout()


# Plot attitude changes
fig, axs = plt.subplots(3, 1, figsize=(8, 6))
times = np.arange(time_steps) * dt
labels = ["Roll (phi)", "Pitch (theta)", "Yaw (psi)"]
for i in range(3):
    axs[i].plot(times, attitudes[:, i], label=labels[i])
    axs[i].set_ylabel(labels[i])
    axs[i].legend()
axs[2].set_xlabel("Time (s)")
plt.tight_layout()
plt.show()

        


