import rclpy
from rclpy.node import Node
from javis_interfaces.srv import GetCoordinates

class MoveRobotArmClient(Node):
    def __init__(self):
        super().__init__('move_robot_arm_client')
        self.client = self.create_client(GetCoordinates, 'get_coordinates')

        # 서비스가 준비될 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.request = GetCoordinates.Request()

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = MoveRobotArmClient()
    response = client.send_request()
    client.get_logger().info(f'Result: {response}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
