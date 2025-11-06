import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# 필요한 메시지/액션 임포트
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration

class WaypointSequencer(Node):
    def __init__(self):
        super().__init__('waypoint_sequencer')
        self.get_logger().info("Waypoint Sequencer 노드 시작.")
        
        # NavigateToPose 액션 클라이언트 생성
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )
        
        # --- 목적지 2만 포함하도록 웨이포인트 설정 수정 ---
        self.waypoints = [
            {
                "id": 2, # 목적지 ID를 2로 설정
                "x": 5.03516519,
                "y": -0.92406468,
                "z_orientation": -0.01896088,
                "w_orientation": 0.99982022,
            }
        ]

        self.get_logger().info(f"{len(self.waypoints)}개의 웨이포인트가 로드되었습니다.")

    def send_goal(self, waypoint_data):
        # 1. Nav2 액션 Goal 생성
        goal_msg = NavigateToPose.Goal()
        
        # 2. PoseStamped 메시지 설정
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = waypoint_data["x"]
        goal_pose.pose.position.y = waypoint_data["y"]
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = waypoint_data["z_orientation"]
        goal_pose.pose.orientation.w = waypoint_data["w_orientation"]
        
        goal_msg.pose = goal_pose

        self.get_logger().info(f'[{waypoint_data["id"]}] 목표 전송: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        
        # 3. Action 서버가 준비될 때까지 대기
        self._action_client.wait_for_server()
        
        # 4. Goal 전송 및 응답 대기
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        
        goal_handle = self._send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f'[{waypoint_data["id"]}] 목표가 거부되었습니다.')
            return False

        self.get_logger().info(f'[{waypoint_data["id"]}] 목표 수락. 경로 추적 시작.')
        
        # 5. 최종 결과 대기 (블로킹)
        self._get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)

        result = self._get_result_future.result()
        
        # GoalStatus 상수를 사용하여 최종 상태 확인
        if result.status == GoalStatus.STATUS_SUCCEEDED: 
            return True
        else:
            status_text = {
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED"
            }.get(result.status, "FAILURE")

            self.get_logger().error(f'[{waypoint_data["id"]}] 목표 달성 실패. 최종 상태: {status_text} ({result.status})')
            return False

    def run_sequence(self):
        """목적지 2로 이동하고 대기하는 메인 루프 (하나의 목표만 처리)"""
        # 웨이포인트가 하나만 있으므로, 첫 번째 요소를 사용합니다.
        waypoint = self.waypoints[0]
            
        self.get_logger().info(f"===== Waypoint {waypoint['id']} (단일 목표) =====")
        
        # 1. 목표 위치로 이동
        success = self.send_goal(waypoint)
        
        if success:
            # 2. 목적지 달성 메시지 출력
            self.get_logger().info(f"✅ 목적지 {waypoint['id']}에 도달했습니다.")
            
            # 3. 1분(60초) 대기
            self.get_logger().info("⏳ 60초간 대기합니다...")
            time.sleep(60) # 1분 대기
            self.get_logger().info("✅ 대기 시간 60초가 완료되었습니다.")
            
        else:
            self.get_logger().error(f"❌ Waypoint {waypoint['id']} 이동에 실패했습니다.")
            
        self.get_logger().info("====================================")
        self.get_logger().info("🎉 임무를 완료했습니다! 🎉")
        self.get_logger().info("====================================")

def main(args=None):
    rclpy.init(args=args)
    sequencer = WaypointSequencer()
    try:
        sequencer.run_sequence()
    except Exception as e:
        sequencer.get_logger().error(f"시퀀스 실행 중 오류 발생: {e}")
    finally:
        sequencer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()