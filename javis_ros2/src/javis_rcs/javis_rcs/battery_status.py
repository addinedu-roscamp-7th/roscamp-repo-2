import rclpy
from rclpy.node import Node
from javis_interfaces.msg import BatteryStatus

import sys
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
    if len(sys.argv) < 2:
        # setup.py의 entry_points에 'battery_test'로 등록되어 있으므로 사용법에 반영합니다.
        print("Usage: ros2 run javis_rcs battery_test <namespace>")
        return

    namespace = sys.argv[1]
    battery_monitor = BatteryStatusMonitor(namespace=namespace)
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()