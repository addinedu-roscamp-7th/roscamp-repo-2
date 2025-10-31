import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from javis_interfaces.msg import CurrentPose

class PosePublisher(Node):
    def __init__(self):
        super().__init__("pose_publisher")
        self.publisher = self.create_publisher(CurrentPose, "/status/current_pose", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.time_callback)
        self.get_logger().info("현재 자세 발행")

    def time_callback(self):
        msg = CurrentPose()
        msg.pose = Pose2D()
        msg.pose.x = 2.0
        msg.pose.y = 1.5
        msg.pose.theta = 2.0
        
        self.publisher.publish(msg)

        x = msg.pose.x
        y = msg.pose.y
        theta = msg.pose.theta
        self.get_logger().info(f"현재 포즈 발행: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

def main(args = None):
    rp.init(args = args)

    turtlesim_publisher = PosePublisher()
    rp.spin(turtlesim_publisher)

    turtlesim_publisher.destroy_node()
    rp.shutdown()

    PosePublisher

if __name__ == "__main__":
    main()