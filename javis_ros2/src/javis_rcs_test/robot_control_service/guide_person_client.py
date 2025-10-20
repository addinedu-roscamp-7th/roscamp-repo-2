import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import GuidePerson
from rclpy.task import Future
from geometry_msgs.msg import Pose2D


class GuidePersonClient(Node):
    def __init__(self, namespace=''):
        super().__init__('guide_person_client', namespace=namespace)
        self._client = ActionClient(self, GuidePerson, 'guide_person')

    def send_goal(self, dest_location: Pose2D) -> Future:
        goal_msg = GuidePerson.Goal()
        goal_msg.dest_location = dest_location

        self.get_logger().info("도비서버 기다리는중")
        self._client.wait_for_server()

        return self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
    
    def feedback_callback(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(f'feedback = distance_remaining:{fb.distance_remaining_m}, person_detected:{fb.person_detected}')

def main():
    rclpy.init()
    node = GuidePersonClient(namespace='dobby1/main')
    goal_future = node.send_goal(
        
        dest_location = Pose2D(x=1.2, y=5.3, theta=3.14)
    )
    def goal_response_cb(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().info('길안내 요청이 거부됨')
            return
        result_future = goal_handle.get_result_async()

        def result_cb(r_future):
            result = r_future.result().result
            node.get_logger().info(f"작업 완료 결과 =  success:{result.success}, error_code:{result.error_code}, total_distance_m:{result.total_distance_m}, totla_time_sec:{result.total_time_sec}, message: {result.message}")
            rclpy.shutdown()
        result_future.add_done_callback(result_cb)
    goal_future.add_done_callback(goal_response_cb)

    rclpy.spin(node)
