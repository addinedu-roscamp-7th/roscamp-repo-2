import rclpy
from rclpy.node import Node
from javis_interfaces.msg import BatteryStatus

class BatteryStatusMonitor(Node):
    def __init__(self, namespace):
        super().__init__('battery_status_monitor', namespace=namespace)
        self.battery_level = 100.0
        self.publisher = self.create_publisher(BatteryStatus, 'status/battery_status', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_status)
        self.get_logger().info('Battery Status Monitor node has been started.')

    def publish_battery_status(self):
        """
        주기적으로 배터리 상태를 생성하고 발행합니다.
        """
        msg = BatteryStatus()

        # 배터리 감소 시뮬레이션
        if self.battery_level > 0:
            self.battery_level -= 0.1
        else:
            self.battery_level = 100.0 # 다시 100%로 리셋 (테스트용)

        msg.charge_percentage = self.battery_level
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Battery Status: {msg.charge_percentage:.2f}%')

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryStatusMonitor(namespace='dobby1')
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()