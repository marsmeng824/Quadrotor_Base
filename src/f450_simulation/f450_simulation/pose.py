import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from gazebo_msgs.srv import GetEntityState
import time
from f450_simulation.pid import PID  #import

class HoverController(Node):
    def __init__(self):
        super().__init__('hover_controller')

        # publish /demo/force topic
        self.force_publisher = self.create_publisher(Wrench, '/demo/force', 10)
        # Gazebo get client service
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        # parameter
        self.mass = 2  # kg
        self.g = 9.81  # m/s^2
        self.drone_name = 'F450'  # my Gazebo model name

        # target_position (x_d, y_d, z_d)
        self.target_position = (3.0, 0.0, 4.0)

        #  PID controller
        self.pid_x = PID(5.0, 0.1, 1.0, setpoint=self.target_position[0])
        self.pid_y = PID(1.0, 0.1, 0.1, setpoint=self.target_position[1])
        self.pid_z = PID(2.0, 0.5, 10.0, setpoint=self.target_position[2])

        # set times
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # get drone position
        request = GetEntityState.Request()
        request.name = self.drone_name
        request.reference_frame = 'world'

        future = self.get_entity_state_client.call_async(request)
        future.add_done_callback(self.get_entity_state_callback)

    def get_entity_state_callback(self, future):
         
        try:
            state1 = future.result()
            if state1 is None:  # 
                self.get_logger().error("❌ fail to get position")
                return

            position = state1.state.pose.position
            x, y, z = position.x, position.y, position.z


            # calculate force
            Fx = self.pid_x.compute(x)
            Fy = self.pid_y.compute(y)
            Fz = self.pid_z.compute(z) + self.mass * self.g  # keep hovering

           # generate Wrench msg
            wrench_msg = Wrench()
            wrench_msg.force.x = Fx
            wrench_msg.force.y = Fy
            wrench_msg.force.z = Fz

            # publish force information
            self.force_publisher.publish(wrench_msg)

            # FORCE information
            self.get_logger().info(f" Fx={Fx:.2f}, Fy={Fy:.2f}, Fz={Fz:.2f}")

        except Exception as e:
            self.get_logger().error(f"❌ {e}")

def main():
    rclpy.init()
    node = HoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



