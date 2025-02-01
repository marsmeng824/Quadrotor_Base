import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Twist
import math

class QuadrotorController(Node):
    def __init__(self):
        super().__init__('quadrotor_controller')

        # create a client
        self.get_state_client = self.create_client(GetEntityState, '/demo/get_entity_state')
        self.set_state_client = self.create_client(SetEntityState, '/demo/set_entity_state')

        # make sure the client available
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_entity_state service...')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_entity_state service...')

        # initialization object and Kp,kd parameter
        self.target_z = 3.0  # height (m)
        self.kp = 4.0      
        self.kd = 4.0      

        # initialization state variable
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        #initialization velocity and acceleration
        self.velocity = 0.0  # initial velocity
        self.acceleration = 0.0  # initial acceleration

        # start control loop
        self.control_loop_timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        self.get_logger().info("Starting control loop...")

        # get entity state
        get_request = GetEntityState.Request()
        get_request.name = "quadrotor"
        
        future = self.get_state_client.call_async(get_request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response is not None:
                state = response.state
                current_z = state.pose.position.z
                self.get_logger().info(f"Current height: {current_z:.3f} m")

                # get control output
                control_input = self.pd_controller(current_z)

                # set control twist
                self.set_quadrotor_state(control_input)
            else:
                self.get_logger().error('Received empty response.')
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def pd_controller(self, current_z):
        # current error
        error = self.target_z - current_z

        # current time
        current_time = self.get_clock().now()
        delta_time = (current_time - self.prev_time).nanoseconds / 1e9  # change to second

        # 误差微分
        error_derivative = (error - self.prev_error) / delta_time if delta_time > 0 else 0.0
        control_output = self.kp * error + self.kd * error_derivative
        self.acceleration = control_output
        
        
        # integral to get velocity
        self.velocity += (self.acceleration+ 9.81) * delta_time
        

        # update variables
        self.prev_error = error
        self.prev_time = current_time

        self.get_logger().info(f"twist: {self.velocity:.3f},"
                               f"Error: {error:.3f} m, "
                               )

        return self.velocity

    def set_quadrotor_state(self, control_input):
        # create SetEntityState request
        set_request = SetEntityState.Request()
        set_request.state.name = "quadrotor"

        # tae advantage of request 
        set_request.state.twist.linear.z = control_input

        # sending requests async
        future = self.set_state_client.call_async(set_request)
        future.add_done_callback(self.handle_set_response)

    def handle_set_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully applied control input.")
            else:
                self.get_logger().error(f"Failed to set entity state: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"Failed to call set_entity_state service: {e}")


def main(args=None):
    rclpy.init(args=args)

    controller = QuadrotorController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down controller.')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



