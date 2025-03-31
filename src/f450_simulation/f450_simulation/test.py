import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import GetEntityState
import f450_simulation.QP as QP
from geometry_msgs.msg import Vector3


class QuadrotorPIDController(Node):
    def __init__(self):
        super().__init__('quadrotor_pid_controller')

        # Create a Client to Gazebo entity state
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        # create a Publisher to publish control command
        self.force_publisher = self.create_publisher(Wrench, '/demo/force', 10)
        
        # target position
        self.z_d = 2.0

        # physics parameter
        self.mass = 2  # kg
        self.g = 9.81  # m/s² 
        self.drone_name = 'F450'
        
        # sample time
        self.dt = 0.1

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
        lin_vel = response.state.twist.linear


        # save state
        self.current_state = {
            "z": pos.z,
            "vz": lin_vel.z,
        }

        # call control loop
        self.control_loop()

    def send_control(self,T):
        """ publish thrust """
        msg = Wrench()
        msg.force = Vector3(x=0.0, y=0.0, z=T)

        self.force_publisher.publish(msg)

    def control_loop(self):
        """ only `self.current_state` not None execute """
        if self.current_state is None:
            return

        z =  self.current_state["z"]
        vz = self.current_state["vz"]

        # **Step 1: position control(outer control)**
        a = QP.get_optimal_acceleration([z, vz], [self.z_d, 0])
         # target velocity
        
        T = self.mass * (a + self.g)  # Compute required thrust
        
        # **set entity state**
        self.send_control(T)

        print(f"[DEBUG] acceleration: {a}")
        print(f"[DEBUG] pos_z: {z}")
        print(f"[DEBUG] volecity: {vz}")


def main():
    rclpy.init()
    controller = QuadrotorPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


