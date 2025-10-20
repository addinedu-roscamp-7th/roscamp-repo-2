import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import CleanSeat
from rclpy.task import Future
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D

class CleanSeatClient(Node):
    def __init__(self, namespace=''):
        super().__init__('clean_seat_client', namespace=namespace)
        self._client = ActionClient(self, CleanSeat, 'clean_seat')

    def send_goal(self, seat_id: int, return_desk_location: Pose2D, return_desk_pose: Pose,  bin_location: Pose2D, bin_pose: Pose) -> Future:
        goal_msg = CleanSeat.Goal()
        goal_msg.seat_id = seat_id
        goal_msg.return_desk_location = return_desk_location
        goal_msg.return_desk_pose = return_desk_pose
        goal_msg.bin_location = bin_location
        goal_msg.bin_pose = bin_pose
        

        self.get_logger().info("도비 서버 기다리는 중...")
        self._client.wait_for_server()

        return self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
    
    def feedback_callback(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(f'feedback = 진행률: {fb.progress_percent}%')

def main():
    rclpy.init()
    node = CleanSeatClient(namespace='dobby1/main')
    goal_future = node.send_goal(
        seat_id=2,
        return_desk_location=Pose2D(x=2.5, y=1.25, theta=3.14),
        return_desk_pose=Pose(x=4.0, y=5.0, z=6.0),
        bin_location = Pose2D(x=7.0, y=8.0, theta=9.0),
        bin_pose = Pose(x=10.0, y=11.0, z=12.0)
    )

    def goal_response_cb(future):
        goal_handle = future.result()
        node = CleanSeatClient()
        if not goal_handle.accepted:
            node.get_logger().info('좌석 정리 작업 거절당함')
            return
        result_future = goal_handle.get_result_async()

        def result_cb(r_future):
            result = r_future.result().result
            node.get_logger().info(f'작업 완료 결과 =  seat_id: {result.seat_id}, success: {result.success}, trash_collected_count: {result.trash_collected_count}, trash_types: {result.trash_types}, total_distance_m: {result.total_distance_m}, total_time_sec:{result.total_time_sec}, message: {result.message}')
            rclpy.shutdown()
        result_future.add_done_callback(result_cb)
    goal_future.add_done_callback(goal_response_cb)

    rclpy.spin(node)
            


    