#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import EntityState

class QuadBroadcast(Node):
    def __init__(self):
        super().__init__('QuadBroadcast_node')

        # 无人机名称
        self.drone_name = 'F450'  # 根据 Gazebo 中的模型名称修改
       


        # 创建服务客户端以获取实体状态
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        # 等待服务可用
        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /gazebo/get_entity_state 服务...')

        # 创建控制推力的发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 定时器，定期获取无人机状态
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_drone_pose(self):
        # 创建服务请求
        request = GetEntityState.Request()
        request.name = self.drone_name
        request.reference_frame = 'world'  # 参考坐标系

        # 调用服务
        future = self.get_entity_state_client.call_async(request)
        future.add_done_callback(self.get_entity_state_callback)

    def get_entity_state_callback(self, future):
        try:
            response = future.result()
            if response.success:
                pose = response.state.pose
                self.get_logger().info(f"Quadrotor Pose: {pose}")
            else:
                self.get_logger().warn(f"获取无人机状态失败: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"服务调用失败: {e}")

    def timer_callback(self):
        # 定期获取无人机 Pose
        self.get_drone_pose()

def main(args=None):
    rclpy.init(args=args)
    Quad_Broadcast = QuadBroadcast()
    rclpy.spin(Quad_Broadcast)
    Quad_Broadcast.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()