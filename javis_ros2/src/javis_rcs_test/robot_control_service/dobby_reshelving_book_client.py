import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import ReshelvingBook
from rclpy.task import Future
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D

class DobbyReshelvingBookClient(Node):
    def __init__(self, namespace=''):
        super().__init__('dobby_reshelving_book_client', namespace=namespace)
        self._client = ActionClient(self, ReshelvingBook, 'reshelving_book')

    def send_goal(self, return_desk_id:int, return_desk_location:Pose2D, return_desk_pose:Pose) -> Future :
        goal_msg = ReshelvingBook.Goal()
        goal_msg.return_desk_id = return_desk_id
        goal_msg.return_desk_location = return_desk_location
        goal_msg.return_desk_pose = return_desk_pose


        self.get_logger().info("도서 서버 기다리는 중")
        self._client.wait_for_server()

        return self._client.send_goal_async(
            goal_msg, feedback_callback=self.reshelving_callback
        )
    
    def reshelving_callback(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(f'dobby reshelving feedback:{fb.progress_percent}%')

def main():
    rclpy.init()
    node = DobbyReshelvingBookClient(namespace='dobby1/main')
    goal_future = node.send_goal( return_desk_id=2, return_desk_location=Pose2D(x=5.1, y=1.2, theta=1.7), return_desk_pose=Pose(x=4.0, y=5.0, z=6.0)
    )

    def goal_response_cb(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().info('작업 거절당함')
            return
        result_future = goal_handle.get_result_async()

        def result_cb(r_future):
            result = r_future.result().result
            node.get_logger().info(f'작업 결과, success:{result.success}, message:{result.message}, books_processed:{result.books_processed}, failed_book_ids:{result.failed_book_ids}, total_distance_m:{result.total_distance_m}m, total_time_sec:{result.total_time_sec}초')
            rclpy.shutdown()
        result_future.add_done_callback(result_cb)
    goal_future.add_done_callback(goal_response_cb)

    rclpy.spin(node)