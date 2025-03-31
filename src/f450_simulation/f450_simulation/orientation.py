import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import GetEntityState
import numpy as np
from f450_simulation.vectorPID import VectorPID
import f450_simulation.transform3d as tf
from geometry_msgs.msg import Vector3


class QuadrotorPIDController(Node):
    def __init__(self):
        super().__init__('quadrotor_pid_controller')

        # Create a Client to Gazebo entity state
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        # create a Publisher to publish control command
        self.force_publisher = self.create_publisher(Wrench, '/demo/force', 10)
        
        # PID gain
        self.Kp_att = 1.0
        self.Kd_att = 0.2
      
        # target position
        self.x_d, self.y_d, self.z_d = 2.0, 0.0, 2.0  

        # physics parameter
        self.mass = 2  # kg
        self.g = 9.81  # m/s² 
        self.drone_name = 'F450'
        
        # PID chontrol
        self.pid_pos = VectorPID(kp=[0.5,0.5,2], ki=[0.01,0.01,0.1], kd=[2,2,3])
        self.pid_att = VectorPID(self.Kp_att, 0.05, self.Kd_att)
        self.pid_ang = VectorPID(self.Kp_att, 0.05, self.Kd_att)


        self.J = np.array([
            [0.0411, 0, 0],
            [0, 0.0478, 0],
            [0, 0, 0.0599]
         ])

       

        # sample time
        self.dt = 0.1
        
        # save the current entity state
        self.current_state = None  

        # switch on timer
        self.timer = self.create_timer(self.dt, self.get_state)  # call get_state()

    def get_state(self):
        """ get entity state by gezabo(async) """
        req = GetEntityState.Request()
        req.name = self.drone_name
        req.reference_frame = 'world'
        
        if not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Gazebo get_entity_state fail")
            return
        
        future = self.get_entity_state_client.call_async(req)
        future.add_done_callback(self.get_entity_state_callback)  # callback async

    def get_entity_state_callback(self, future):
        """  executethe procedure by data  """
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error("can't get Gazebo entity state")
            return
        
        pos = response.state.pose.position
        ori = response.state.pose.orientation
        lin_vel = response.state.twist.linear
        ang_vel = response.state.twist.angular

        # **switch quaternion to euler angular**
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        euler = tf.quat2euler(quaternion)

        # save state
        self.current_state = {
            "x": pos.x, "y": pos.y, "z": pos.z,
            "vx": lin_vel.x, "vy": lin_vel.y, "vz": lin_vel.z,
            "phi": euler[0], "theta": euler[1], "psi": euler[2],
            "p": ang_vel.x, "q": ang_vel.y, "r": ang_vel.z
        }

        # call control loop
        self.control_loop()

    def send_control(self, thrusts, torques):
        """ publish thrusts and torque """
        msg = Wrench()
        msg.force = Vector3(x=thrusts[0], y=thrusts[1], z=thrusts[2])
        msg.torque = Vector3(x=torques[0], y=torques[1], z=torques[2])

        self.force_publisher.publish(msg)



    def control_loop(self):
        """ only `self.current_state` not None execute """
        if self.current_state is None:
            return

        x, y, z = self.current_state["x"], self.current_state["y"], self.current_state["z"]
        phi, theta, psi = self.current_state["phi"], self.current_state["theta"], self.current_state["psi"]
        euler=np.array([phi,theta,psi])
        omega=np.array([self.current_state["p"], self.current_state["q"], self.current_state["r"]])

        # **Step 1: position control(outer control)**
        e_pos = np.array([self.x_d - x, self.y_d - y, self.z_d - z])
        x_acc ,y_acc,z_acc = self.pid_pos.compute(e_pos)  # Translational Dynamics Model 

        # Limit acceleration in each axis to (-3, 3)
        a_des = np.array([
           np.clip(x_acc, -3.0, 3.0),
           np.clip(y_acc, -3.0, 3.0),
           np.clip(z_acc, -3.0, 3.0)
        ])

        ## T = np.clip(self.mass * (z_acc + self.g), 0, 40)  Compute required thrust


        # **Step# Compute roll (ϕ_d) and pitch (θ_d) using Eq. (29) and (30)
        g = 9.81
        acc_des = a_des + np.array([0.0, 0.0, g])

     # 3. compute expectation attitude 
        euler_des = tf.acc_to_euler(acc_des,psi)
        phi_d = euler_des[0]
        theta_d = euler_des[1]
        psi_d = 0.0  # fixed yaw

        # **Step 3: orientation control**
        e_att = np.array([phi_d - phi, theta_d - theta, psi_d - psi])
        angular_d = self.pid_att.compute(e_att)

        omega_d = tf.euler_to_omega(euler, angular_d)
        e_oma = omega_d - omega
        torques= self.J @self.pid_ang.compute(e_oma)

        thrust_x = self.mass*acc_des[0]  
        thrust_y = self.mass*acc_des[1]  
        thrust_z = self.mass*acc_des[2]
        thrust = np.array([thrust_x, thrust_y, thrust_z])  # update thrust
        
        # **set entity state**
        self.send_control(thrust, torques)

        print(f"[DEBUG] Position Error: {e_pos}")
        print(f"[DEBUG] Attitude Error: {e_att}")
        print(f"[DEBUG] Thrust: {thrust}")
        print(f"[DEBUG] Torques: {torques}")


def main():
    rclpy.init()
    controller = QuadrotorPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
