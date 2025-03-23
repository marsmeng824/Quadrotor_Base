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
   ```bash
   git clone https://github.com/marsmeng824/Quadrotor_Base.git
   cd Quadrotor_Base
   

2. **Setup your ROS2 workspace**:
   ```bash   
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # Install dependencies
   rosdep install --from-paths src --ignore-src -r -y
   # Build the workspace
   colcon build
   # Source the environment
   source install/setup.bash
   
3. **Run the simulation**

## 📦 Functions
🚁 **1. Spawn the F450 Quadrotor Model**
This command will launch the simulation world and spawn the quadrotor model named F450.
        
       ```bash 
      ros2 launch f450_simulation spawn_quad.launch.py

It integrates Gazebo plugins that expose a wide range of services and topics for drone control,such as
   - `/gazebo/set_entity_state`
   - `/gazebo/get_entity_state`

       ```bash
       ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: 'F450'}"
This command retrieves the position and orientation of the F450 model.

       ```bash
       # Set drone state (teleport to new position)
       ros2 service call /demo/set_entity_state gazebo_msgs/srv/SetEntityState "state:
         name: 'F450'
         pose:
           position: {x: 2.0, y: 1.0, z: 1.5}
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
         twist:
           linear: {x: 0.0, y: 0.0, z: 0.0}
           angular: {x: 0.0, y: 0.0, z: 0.0}"
This command retrieves the position and orientation of the F450 model.
       
   - `/demo/force` (for applying thrust and torque)
        ```bash
         ros2 topic pub -1 /demo/force geometry_msgs/Wrench "force: {x: 10.0}"
This command publishes a force of 10.0 (Newtons) in the positive X direction. 
      
    

   
