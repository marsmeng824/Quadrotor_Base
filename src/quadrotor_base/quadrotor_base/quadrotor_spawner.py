import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class QuadrotorSpawner(Node):
    def __init__(self):
        super().__init__('quadrotor_node')

        # 创建客户端以调用 /spawn_entity 服务
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # 准备请求数据
        request = SpawnEntity.Request()
        request.name = 'quadrotor'
        request.xml = self.load_model_sdf('/home/mars0824/ros2_ws/src/quadrotor_base/models/quadrotor/model.sdf')
        request.robot_namespace = 'quadrotor'
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.1

        # 调用服务
        self.spawn_client.call_async(request)

    def load_model_sdf(self, model_path):
        with open(model_path, 'r') as file:
            return file.read()

def main(args=None):
    rclpy.init(args=args)
    node = QuadrotorSpawner()
    rclpy.spin(node)
    rclpy.shutdown()
