import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import GetEntityState


class QuadrotorXController(Node):
    def __init__(self):
        super().__init__('quadrotor_x_controller')

        # set target point 
        self.target_height = 4.0  

        # PID controller
        # PID gain
        self.Kp = 2  # proportion gain
        self.Ki = 1  # Integral gain
        self.Kd = 10  # Derivation gain

        # PID variable
        self.previous_error = 0.0
        self.integral = 0.0

        
        # Current height
        self.current_height = None

        self.drone_name = 'F450'  # my Gazebo model name
        self.link_name = 'F450::link'  

        # publish a generate force Publisher
        self.force_publisher = self.create_publisher(Wrench, '/demo/force', 10)

        # create Client to get quadrotor z state
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        # set timer
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
            response = future.result()
            if response.success:
                self.current_height = response.state.pose.position.z
                self.get_logger().info(f"current positon: {self.current_height}")
               
                force_z = self.PID_controller()
            

                # generate Wrench msg
                wrench_msg = Wrench()
                wrench_msg.force.x = 0.0
                wrench_msg.force.y = 0.0
                wrench_msg.force.z = force_z

            # publish force information
                self.force_publisher.publish(wrench_msg)
            else:
                self.get_logger().warn(f"fail: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"fail: {e}")

    def PID_controller(self):
        if self.current_height is None:
            self.get_logger().warn("The current height is not obtained, skip this control cycle")
            return 0.0  # return 0

        # calculate height error
        error = self.target_height - self.current_height

        # Update integrals and derivatives
        self.integral += error*0.1 
        derivative = (error - self.previous_error)/0.1

        # calculate output(force)
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative + 9.8*2

        # update
        self.previous_error = error

        return output  


def main(args=None):
    rclpy.init(args=args)
    node = QuadrotorXController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

