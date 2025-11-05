#!/usr/bin/env python3
import rclpy as rp
from rclpy.node import Node
# from javis_ais_msgs.msg import TrackingStatus # import는 파일명으로 (무조건)
from javis_interfaces.msg import TrackingStatus # import는 파일명으로 (무조건)
# 코드에 숨어있지만 import는 하지 않아도 되는 항목들
# from geometry_msgs
# from std_msgs
# from builtin_interfaces

class tracking_publisher(Node):

    def __init__(self):
        super().__init__('tracking_person_publisher')
        self.publisher = self.create_publisher(TrackingStatus, '/dobby1/ai/tracking/status', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # 1st communication test dummy data setting
        msg = TrackingStatus()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tracking_person_status'
        msg.tracking_id = 'track_person'
        msg.person_detected = True
        # person_pose는 geometry_msgs/Pose message type 사용
        # geometry_msgs/Pose 내부에 선언된 message type
        # geometry_msgs/Point position -> float64 x / y / z
        # geometry_msgs/Quaternion orientation -> float64 x / y / z / w
        msg.person_pose.position.x = 2.0
        msg.person_pose.position.y = 0.7
        msg.person_pose.position.z = 2.0
        msg.person_pose.orientation.x = 0.0
        msg.person_pose.orientation.y = 0.0
        msg.person_pose.orientation.z = 0.0
        msg.person_pose.orientation.w = 1.0
        msg.distance_to_person = 7.0
        msg.confidence = 5.0
        msg.is_lost = False
        msg.time_since_last_seen = 0.1

        self.publisher.publish(msg)
        self.get_logger().info(f'"{msg.tracking_id}" 데이터 Publish', throttle_duration_sec=1.0)



def main(args = None):
    rp.init(args = args)

    tracking_people = tracking_publisher()
    rp.spin(tracking_people)

    tracking_people.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()