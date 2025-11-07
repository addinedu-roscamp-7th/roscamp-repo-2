import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from javis_interfaces.action import TrackingTest
import sys

class TrackingTestClient(Node):
    def __init__(self):
        super().__init__('tracking_test_client')
        self.dobby_nav2_client = ActionClient(self, TrackingTest, 'tracking_test')

    def send_goal(self, goal_pose: PoseStamped):
        goal_msg = TrackingTest.Goal()
        goal_msg.goal_pose = goal_pose

        self.get_logger().info('dobby nav2 test 서버 기다리는 중')
        if not self.dobby_nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('dobby nav2 test 서버가 응답하지 않습니다.')
            rclpy.shutdown()
            return

        self.get_logger().info('목표 지점 전송...')
        send_goal_future = self.dobby_nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 피드백 메시지가 너무 길 경우 일부만 출력하거나, 필요한 정보만 가공하여 출력할 수 있습니다.
        self.get_logger().info(f'피드백 수신: {feedback.part_of_path}')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('목표가 거절되었습니다.')
            return
        self.get_logger().info('목표가 수락되었습니다.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'결과: {result.message}')
        rclpy.shutdown() # 작업 완료 후 노드 종료

def main(args=None):
    rclpy.init(args=args)
    
    try:
        pose_menu = int(sys.argv[1])
    except (IndexError, ValueError):
        print("0, 1, 2, 3, 5 중에 하나를 입력하세요")
        
        rclpy.shutdown()
        return

    goal_pose = PoseStamped()
    
    if pose_menu == 0:
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.1
    elif pose_menu == 1:
        goal_pose.pose.position.x = -0.34356356069590466
        goal_pose.pose.position.y = -5.856086955052448
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.009428745496879554
        goal_pose.pose.orientation.w = 0.9999555483912048

    elif pose_menu == 2:
        goal_pose.pose.position.x = -0.3683174093558657
        goal_pose.pose.position.y = -2.903880938591644
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.7326551462428521
        goal_pose.pose.orientation.w = 0.6806000563354848
    elif pose_menu == 3:
        goal_pose.pose.position.x = -1.5115693211403654
        goal_pose.pose.position.y = -0.7371961728499472
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.6690682999336836
        goal_pose.pose.orientation.w = 0.7432009217054635
    elif pose_menu == 4:
        goal_pose.pose.position.x = -0.5109752595635183
        goal_pose.pose.position.y = -4.9554330645441365
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.7482731018940546
        goal_pose.pose.orientation.w = 0.6633908086353395
    elif pose_menu == 5:
        goal_pose.pose.position.x = -5.958908382012765
        goal_pose.pose.position.y = -5.536665961475101
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.7194566842054619
        goal_pose.pose.orientation.w = 0.6945373132899931
    else:
        print(f"잘못된 메뉴 선택: {pose_menu}. 1, 2, 3 중 하나를 선택하세요.")
        rclpy.shutdown()
        return

    node = TrackingTestClient()
    try:
        node.get_logger().info('dobby_nav2_client 노드 실행')
        node.send_goal(goal_pose)
        rclpy.spin(node) # 작업이 완료되고 rclpy.shutdown()이 호출될 때까지 대기
    except KeyboardInterrupt:
        node.get_logger().info('dobby_nav2_client 노드 종료')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
