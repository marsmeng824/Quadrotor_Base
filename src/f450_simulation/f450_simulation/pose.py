import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import GetEntityState


class QuadrotorXController(Node):
    def __init__(self):
        super().__init__('quadrotor_x_controller')

        # set target point (设定点)
        self.target_x = 5.0  

        # PID 控制器（P、I、D 参数需要调整）
        # PID gain
        self.Kp = 4  # proportion gain
        self.Ki = 1  # Integral gain
        self.Kd = 4  # Derivation gain

        # PID variable
        self.previous_error = 0.0
        self.integral = 0.0

        
        # Current height
        self.current_height = None

        # 无人机名称和 link 名称
        self.drone_name = 'F450'  # 你的 Gazebo 模型名称
        self.link_name = 'F450::link'  

        # 发布控制力的 Publisher
        self.force_publisher = self.create_publisher(Wrench, '/demo/force', 10)

        # 创建 Client 获取 quadrotor 的 x 位置
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        # 设定定时器（50Hz）
        self.timer = self.create_timer(0.2, self.control_loop)

    def control_loop(self):
        # 获取无人机的高度
        request = GetEntityState.Request()
        request.name = self.drone_name
        request.reference_frame = 'world'

        future = self.get_entity_state_client.call_async(request)
        future.add_done_callback(self.get_entity_state_callback)

    def get_entity_state_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.current_height = response.state.pose.position.x
                self.get_logger().info(f"current positon: {self.current_height}")
               
                force_x = self.PID_controller()
            

                # 生成 Wrench 消息（仅作用于 x 轴）
                wrench_msg = Wrench()
                wrench_msg.force.x = force_x
                wrench_msg.force.y = 0.0
                wrench_msg.force.z = 0.0

            # 发布力信息
                self.force_publisher.publish(wrench_msg)
            else:
                self.get_logger().warn(f"获取无人机状态失败: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"服务调用失败: {e}")

    def PID_controller(self):
        if self.current_height is None:
            self.get_logger().warn("当前高度未获取，跳过本次控制循环")
            return 0.0  # 返回 0 避免出错

        # 计算高度误差
        error = self.target_x - self.current_height

        # 更新积分和微分
        self.integral += error*0.2 
        derivative = (error - self.previous_error)/0.2

        # 计算 PID 输出（力）
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative 

        # 更新上一次的误差
        self.previous_error = error

        return output  # 作为 Z 方向上的力



def main(args=None):
    rclpy.init(args=args)
    node = QuadrotorXController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


