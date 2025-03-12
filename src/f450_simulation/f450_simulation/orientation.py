import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Wrench
import numpy as np
from f450_simulation.vectorPID import VectorPID
import f450_simulation.transform3d as tf
from geometry_msgs.msg import Vector3
from f450_simulation.trajectorypid import TPID

class QuadrotorPIDController(Node):
    def __init__(self):
        super().__init__('quadrotor_pid_controller')

        # Create a Client to Gazebo entity state
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        # create a Publisher to publish control command
        self.force_publisher = self.create_publisher(Wrench, '/demo/force', 10)
        
        # PID gain
        self.Kp_att = 1.5
        self.Kd_att = 0.8
        self.Kp_rate = 2.0
        self.Kd_rate = 1.0
        
        # target position
        self.x_d, self.y_d, self.z_d = 5.0, 0.0, 3.0  

        # physics parameter
        self.mass = 2  # kg
        self.g = 9.81  # m/s²
        self.drone_name = 'F450'
        
        # PID chontrol
        self.pid_x = TPID(0.5, 0.5, 10.0)
        self.pid_y = TPID(0.5, 0.5, 10.0)
        self.pid_z = TPID(2.0, 0.5, 10.0)
        self.pid_att = VectorPID(np.array(self.Kp_att), np.array(0.1), np.array(self.Kd_att))
        self.pid_rate = VectorPID(np.array(self.Kp_rate), np.array(0.1), np.array(self.Kd_rate))

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
        print(f"[CONTROL OUTPUT] Thrusts: [{thrusts[0]:.4f}, {thrusts[1]:.4f}, {thrusts[2]:.4f}] N")
        print(f"[CONTROL OUTPUT] Torques: [{torques[0]:.4f}, {torques[1]:.4f}, {torques[2]:.4f}] Nm")



    def control_loop(self):
        """ only `self.current_state` not None execute """
        if self.current_state is None:
            return

        x, y, z = self.current_state["x"], self.current_state["y"], self.current_state["z"]
        phi, theta, psi = self.current_state["phi"], self.current_state["theta"], self.current_state["psi"]
        p, q, r = self.current_state["p"], self.current_state["q"], self.current_state["r"]

        # **Step 1: position control(outer control)**
        e_pos = np.array([self.x_d - x, self.y_d - y, self.z_d - z])
        
        vx_d = self.pid_x.compute(e_pos[0])
        vy_d = self.pid_y.compute(e_pos[1])
        vz_d = self.pid_z.compute(e_pos[2])  # target velocity

        # **Step 2: calculate target euler angular**
        theta_d = np.clip((1 / 9.81) * (0.05 * vx_d), -np.pi / 6, np.pi / 6)  # limit Pitch  ±30°
        phi_d = np.clip(-(1 / 9.81) * (0.05 * vy_d), -np.pi / 6, np.pi / 6)   # limit Roll  ±30°
        psi_d = 0.0  # fixed yaw

        # **Step 3: orientation control**
        e_att = np.array([phi_d - phi, theta_d - theta, psi_d - psi])
        dot_att_d = self.pid_att.compute(e_att)
        dot_phi_d, dot_theta_d, dot_psi_d = dot_att_d

        # **Step 4: euler angular rate → body rate**
        R = np.array([
            [1, 0, -np.sin(theta)],
            [0, np.cos(phi), np.cos(theta) * np.sin(phi)],
            [0, -np.sin(phi), np.cos(theta) * np.cos(phi)]
        ])
        body_rate_d = np.dot(R, np.array([dot_phi_d, dot_theta_d, dot_psi_d]))
        p_d, q_d, r_d = body_rate_d

        # **Step 5: body rate - torques**
        e_rate = np.array([p_d - p, q_d - q, r_d - r])
        torques = self.J @ self.pid_rate.compute(e_rate)
       

         # **Step 6: body frame to world frame**
        R_world_body = np.array([
            [np.cos(psi) * np.cos(theta), np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi), np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi)],
            [np.sin(psi) * np.cos(theta), np.sin(psi) * np.sin(theta) * np.sin(phi) + np.cos(psi) * np.cos(phi), np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)],
            [-np.sin(theta), np.cos(theta) * np.sin(phi), np.cos(theta) * np.cos(phi)]
        ])
        torques_world = R_world_body @ torques  



        # **output thrust**
        thrust = np.array([vx_d, vy_d ,9.81*self.mass + vz_d])  # maintain altitude
        
        # **set entity state**
        self.send_control(thrust, torques_world)


def main():
    rclpy.init()
    controller = QuadrotorPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
