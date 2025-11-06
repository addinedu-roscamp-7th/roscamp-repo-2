import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose
from javis_interfaces.msg import TrackingStatus

class TurtlesimSubscriber(Node):

    def __init__(self):
        super().__init__('javis_status_subscriber')
        self.subscription = self.create_subscription(
            TrackingStatus, "/dobby1/ai/tracking/status", self.callback, 10
        )
        # self.subcriptions

    def callback(self, msg):
        self.get_logger().info
        (f'tracking_id: {msg.tracking_id}\n, person_detected: {msg.person_detected}\n, distance_to_person: {msg.distance_to_person}')

def main(args = None):
    rp.init(args = args)

    javis_status_subscriber = TurtlesimSubscriber()
    rp.spin(javis_status_subscriber)

    javis_status_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()