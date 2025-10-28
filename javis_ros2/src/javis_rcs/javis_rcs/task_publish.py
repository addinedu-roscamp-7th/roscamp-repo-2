import rclpy
from rclpy.node import Node
from javis_interfaces.msg import Scheduling
from datetime import datetime
import sys

class TaskPublisher(Node):
    def __init__(self, namespace=""):
        super().__init__('task_publisher', namespace=namespace)
        self.pub_ = self.create_publisher(Scheduling, 'task_publisher', 10)
        self.timer_ = self.create_timer(1.0, self.task_state_pub)

    def task_state_pub(self):
        msg = Scheduling()
        msg.no = 1
        msg.robot_name = self.get_namespace().lstrip('/')
        msg.task_id = 1
        msg.priority = 1
        msg.status = 1
        # 현재 시간을 '년-월-일 시:분:초' 형식의 문자열로 변환합니다.
        msg.task_create_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.pub_.publish(msg)
        self.get_logger().info(f"Publishing task with time: {msg.task_create_time}")
        

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run javis_rcs task_pub <namespace>")
        return

    namespace = sys.argv[1]
    node = TaskPublisher(namespace=namespace)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
