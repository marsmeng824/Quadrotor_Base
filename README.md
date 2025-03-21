# 🚀 Quadrotor_PID_control

## 📖 Purpose Tools and Methods
In today's world, drones play a crucial role in various fields such as inspection, search, and rescue. In these tasks, the ability of drones to autonomously navigate to target locations is essential for mission success.

The goal of this project is to achieve autonomous control of drones in the Gazebo simulation environment, including:

Position control: Precisely navigating to target points.

Attitude control: Ensure the drone remains stable and oriented correctly during flight.

Autonomous flight：Simulate various flight behaviors of drones, such as takeoff, forward, circling, and hovering by cascade PID controller.

## 🛠 Installation
This project requires the following software environment:

WSL 2 + Ubuntu 22.04 (Run Linux on Windows)
ROS 2 Humble (For robot communication)
Gazebo (For UAV simulation)
Python 3 / C++ (For control algorithm development)

You can install them on your computer through the official website

1. **Clone the repository**:
   ```sh
  git clone https://github.com/marsmeng824/Quadrotor_Base.git

2. **Setup your ROS2 workspace**:
      mkdir -p ~/ros2_ws/src
      cd ~/ros2_ws/src
   
      # Move the cloned repository into the workspace
      mv ~/Quadrotor_Base ./
      cd ..
   
     # Install dependencies
     rosdep install --from-paths src --ignore-src -r -y
     # Build the workspace
     colcon build
     # Source the environment
     source install/setup.bash
   
3. **Run the simulation**

## 📦 Functions
    

   
