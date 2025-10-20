import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from javis_interfaces.action import PickupBook
from rclpy.task import Future
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose


class DobbyPickupClient(Node):
    def __init__(self, namespace=''):
        super().__init__('dobby_pickup_client', namespace=namespace)
        self.dobby_client = ActionClient(self, PickupBook, 'pickup_book')

    def pickup_send_goal(self,
                        book_id: str,
                        shelf_approach_location: Pose2D,
                        book_pick_pose: Pose,
                        storage_id: int,
                        storage_approach_location: Pose2D,
                        storage_slot_pose: Pose
                        ) -> Future :
        pickup_goal_msg = PickupBook.Goal()
        pickup_goal_msg.book_id = book_id
        pickup_goal_msg.shelf_approach_location = shelf_approach_location
        pickup_goal_msg.book_pick_pose = book_pick_pose
        pickup_goal_msg.storage_id = storage_id
        pickup_goal_msg.storage_approach_location = storage_approach_location
        pickup_goal_msg.storage_slot_pose = storage_slot_pose

        self.get_logger().info("도비 서버 기다리는 중....")
        self.dobby_client.wait_for_server()

        return self.dobby_client.send_goal_async(
            pickup_goal_msg, feedback_callback=self.dobby_pickup_callback
        )
    
    def dobby_pickup_callback(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(f'dobby pickup feedback: {fb.progress_percent}%')

def main():
    rclpy.init()
    pickup_node = DobbyPickupClient(namespace='dobby1/main')
    pickup_goal_future = pickup_node.pickup_send_goal(
        book_id = "Ga01",
        storage_id = 3,
        shelf_approach_location = Pose2D(x=1.0, y=2.0, theta=3.0),
        book_pick_pose = Pose(x=4.0, y=5.0, z=6.0),   
        storage_approach_location = Pose2D(x=7.0, y=8.0, theta=9.0),
        storage_slot_pose = Pose(x=10.0, y=11.0, z=12.0)


    )
    def pickup_goal_response_cb(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            pickup_node.get_logger().info('Goal rejected')
            return
        result_future = goal_handle.get_result_async()

        def result_cb(r_future):
            result = r_future.result().result
            pickup_node.get_logger().info(f'Result  book_id:{result.book_id}, storage_id:{result.storage_id}, success: {"성공" if result.success else "실패"}, message: {result.message}, total_distance_m: {result.total_distance_m}m, total_time_sec: {result.total_time_sec}초')
            rclpy.shutdown()
        result_future.add_done_callback(result_cb)
    pickup_goal_future.add_done_callback(pickup_goal_response_cb)

    rclpy.spin(pickup_node)
