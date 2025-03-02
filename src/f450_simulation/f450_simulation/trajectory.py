import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Point, Vector3

class CircleTrajectory(Node):
    def __init__(self):
        super().__init__('circle_trajectory')
        
        # Set Parameter
        self.model_name = 'F450'  # 
        self.radius = 3.0  # trajectory's radius
        self.linear_speed = 1.0  # drone's linear speed
        self.update_rate = 10.0  # update rate（Hz）
        
        # calculate angular speed w = v / R
        self.angular_speed = self.linear_speed / self.radius
        self.theta = 0.0  # initialization
        
        # create service client
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /gazebo/set_entity_state service...')
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /gazebo/get_entity_state service...')
        
        # get current altitude
        self.target_height = self.get_current_altitude()
        if self.target_height is None:
            self.get_logger().error('cant get current altitude')
            return
        
        self.get_logger().info(f'current altitude: {self.target_height:.2f} m')
        
        # center position
        self.center_x = 0.0  
        self.center_y = 0.0  
        
        # switch on timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_position)

    def get_current_altitude(self):
        """get current altitude"""
        request = GetEntityState.Request()
        request.name = self.model_name
        request.reference_frame = 'world'
        
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            return future.result().state.pose.position.z
        return None
    
    def update_position(self):
        """update information"""
        # claculate new position
        x = self.center_x + self.radius * math.cos(self.theta)
        y = self.center_y + self.radius * math.sin(self.theta)
        z = self.target_height  # keep the height
        
        # calculate velocity
        vx = -self.linear_speed * math.sin(self.theta)
        vy = self.linear_speed * math.cos(self.theta)
        vz = 0.0
        
        # set entity state
        state_msg = EntityState()
        state_msg.name = self.model_name
        state_msg.pose.position = Point(x=x, y=y, z=z)
        state_msg.twist.linear = Vector3(x=vx, y=vy, z=vz)
        state_msg.reference_frame = 'world'
        
        # send it to service
        request = SetEntityState.Request()
        request.state = state_msg
        future = self.set_state_client.call_async(request)
        
        # control theta
        self.theta += self.angular_speed * (1.0 / self.update_rate)
        if self.theta >= 2 * math.pi:
            self.theta -= 2 * math.pi  # cycle
        
        self.get_logger().info(f'position: x={x:.2f}, y={y:.2f}, height={z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CircleTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



 



