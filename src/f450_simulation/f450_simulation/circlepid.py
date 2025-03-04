import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Wrench, Vector3
import time
import numpy as np
from f450_simulation.trajectorypid import TPID


class QuadrotorCircleController(Node):
    def __init__(self):
        super().__init__('quadrotor_circle_controller')
        self.radius = 2.0
        self.angular_speed = 0.5
        self.target_height = 4.0
        self.pid_x = TPID(2.0, 0.5, 1.0)
        self.pid_y = TPID(2.0, 0.5, 1.0)
        self.pid_z = TPID(2.0, 0.5, 10.0)
         # parameter
        self.mass = 2  # kg
        self.g = 9.81  # m/s^2
        self.drone_name = 'F450'  # my Gazebo model name

         # publish /demo/force topic
        self.force_publisher = self.create_publisher(Wrench, '/demo/force', 10)
        # Gazebo get client service
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /demo/get_entity_state service...')

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.control_loop)  # 0.1s control loop

    def control_loop(self):
        # get entity state
        request = GetEntityState.Request()
        request.name = self.drone_name
        future = self.get_entity_state_client.call_async(request)
        request.reference_frame = 'world'

        future = self.get_entity_state_client.call_async(request)
        future.add_done_callback(self.get_entity_state_callback)

    def get_entity_state_callback(self, future):
         
        try:
            state1 = future.result()
            if state1 is None:  # 
                self.get_logger().error("❌ fail to get position")
                return

            state1 = future.result()
            current_x = state1.state.pose.position.x
            current_y = state1.state.pose.position.y
            current_z = state1.state.pose.position.z
            current_time = time.time()
            dt = 0.1  # Control time interval

            # calculate time and target trajectory
            t = current_time - self.start_time
            target_x = self.radius * np.cos(self.angular_speed * t)
            target_y = self.radius * np.sin(self.angular_speed * t)
            target_z = self.target_height

            # calculate error
            error_x = target_x - current_x
            error_y = target_y - current_y
            error_z = target_z - current_z

            # claculate force by PID 
            Fx = self.pid_x.compute(error_x, dt)
            Fy = self.pid_y.compute(error_y, dt)
            Fz = self.pid_z.compute(error_z, dt) + self.mass * self.g  # during the hovering it should equal to gravity

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

def main(args=None):
    rclpy.init(args=args)
    node = QuadrotorCircleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
