import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState

class SetEntityStateNode(Node):
    def __init__(self):
        super().__init__('set_entity_state_node')

        # create a client node and get in touch with Gazebo server
        self.client = self.create_client(SetEntityState, '/demo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /demo/set_entity_state service...')

        # initial state variable
        self.enable_service = True
        self.counter = 0.0

        # creates a 
        self.service_timer = self.create_timer(0.5, self.update_state)  # Every 0.5 second update state
        self.control_timer = self.create_timer(7.0, self.toggle_service)  # Every 5 second enable/pause service

    def update_state(self):
        # if the service is not enable
        if not self.enable_service:
            self.get_logger().info('Service calls paused.')
            return

        # create a request service
        request = SetEntityState.Request()
        request.state.name = 'quadrotor'

        # set model's pose (pose)
        request.state.pose.position.x = 2.0 * self.counter  
        request.state.pose.position.y = 0.0
        request.state.pose.position.z = 1.0+0.5*self.counter  
        request.state.pose.orientation.x = 0.0
        request.state.pose.orientation.y = 0.0
        request.state.pose.orientation.z = 0.0
        request.state.pose.orientation.w = 1.0

        # set model's twist (twist)
        request.state.twist.linear.x = 1.0  # x
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.2
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.5  # z 

        request.state.reference_frame = ''  # use global reference frame

        # async service
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

        # update counter
        self.counter += 0.1

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully updated entity state.')
            else:
                self.get_logger().warn('Failed to update entity state.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def toggle_service(self):
        # switch the service state
        self.enable_service = not self.enable_service
        if self.enable_service:
            self.get_logger().info('Service calls enabled.')
        else:
            self.get_logger().info('Service calls paused.')

            # stop the timer when the service si paused
            self.service_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = SetEntityStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

