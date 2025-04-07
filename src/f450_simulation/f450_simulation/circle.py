import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Wrench, Vector3
import time
import numpy as np
from f450_simulation.controller.PID import PID
import f450_simulation.transform3d as tf


class QuadrotorCircleController(Node):
    def __init__(self):
        super().__init__('quadrotor_circle_controller')
        self.radius = 2.0
        self.angular_speed = 0.5
        self.target_height = 2.0

        # PID gain
        self.Kp_att = 1
        self.Kd_att = 3

        #PID control
        self.pid_pos = PID(kp=[0.5,0.5,2], ki=[0.1,0.1,0.1], kd=[3,3,3])
        self.pid_att = PID(self.Kp_att, 0.5, self.Kd_att)

         # parameter
        self.mass = 2  # kg
        self.g = 9.81  # m/s^2
        self.drone_name = 'F450'  # my Gazebo model name

        self.J = np.array([
            [0.0411, 0, 0],
            [0, 0.0478, 0],
            [0, 0, 0.0599]
         ])

         # publish /demo/force topic
        self.force_publisher = self.create_publisher(Wrench, '/demo/force', 10)
        # Gazebo get client service
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /demo/get_entity_state service...')

        # save the current entity state
        self.current_state = None

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.get_state)  # 0.1s control loop

    def get_state(self):
        # get entity state
        request = GetEntityState.Request()
        request.name = self.drone_name
        future = self.get_entity_state_client.call_async(request)
        request.reference_frame = 'world'

        future = self.get_entity_state_client.call_async(request)
        future.add_done_callback(self.get_entity_state_callback)

    def get_entity_state_callback(self, future):
         
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

        # **Step 1: position control(outer control)**
        current_time = time.time()

        # calculate time and target trajectory
        t = current_time - self.start_time
        target_x = self.radius * np.cos(self.angular_speed * t)
        target_y = self.radius * np.sin(self.angular_speed * t)
        target_z = self.target_height

        e_pos = np.array([target_x - x, target_y - y, target_z - z])
        
        x_acc ,y_acc,z_acc = self.pid_pos.compute(e_pos)  # target acceleration

        # Limit acceleration in each axis to (-3, 3)
        a_des = np.array([
           np.clip(x_acc, -3.0, 3.0),
           np.clip(y_acc, -3.0, 3.0),
           np.clip(z_acc, -3.0, 3.0)
        ])

        #add gravity compensation
        g = 9.81
        acc_des = a_des + np.array([0.0, 0.0, g])

     # Expectation Attitude: Compute roll (ϕ_d) and pitch (θ_d) using Eq. (29) and (30)
        euler_des = tf.acc_to_euler(acc_des,psi)
        phi_d = euler_des[0]
        theta_d = euler_des[1]
        psi_d = 0.0  # fixed yaw

        # **Step 3: orientation control**
        e_att = np.array([phi_d - phi, theta_d - theta, psi_d - psi])
        angular_d = self.pid_att.compute(e_att)

        torques= self.J @angular_d

        thrust_x = np.clip(self.mass*acc_des[0], -2.0, 2.0)  # X axis limit in ±2N
        thrust_y = np.clip(self.mass*acc_des[1], -2.0, 2.0)  # Y axis limit in ±2N
        thrust_z = self.mass*acc_des[2]
        thrust=np.array([thrust_x,thrust_y,thrust_z])
        
        # **set entity state**
        self.send_control(thrust, torques)

        print(f"[DEBUG] thrusts: {thrust}")
        print(f"[DEBUG] torques: {torques}")
            
        
def main(args=None):
    rclpy.init(args=args)
    node = QuadrotorCircleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

