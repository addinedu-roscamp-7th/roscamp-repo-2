import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from javis_interfaces.msg import ArucoDockingData, DobbyState
# 참고: 아래 import가 성공하려면 javis_ddc 패키지의 package.xml에 javis_dmc에 대한 의존성(<depend>javis_dmc</depend>)이 추가되어야 할 수 있습니다.
from javis_dmc.states.state_enums import MainState
import math

# DDC 노드 내부에서 사용하는 상태 정의
class DDCState:
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    DOCKING = 'DOCKING'

# 도킹 관련 상수
TARGET_DISTANCE = 0.15  # 마커로부터 최종 목표 거리 (15cm)
DOCKING_SPEED_LINEAR = 0.05  # 도킹 시 선형 속도
DOCKING_SPEED_ANGULAR = 0.1  # 도킹 시 각속도
DOCKING_SPEED_STRAFE = 0.03 # 도킹 시 좌/우 이동 속도

# 허용 오차
DISTANCE_TOLERANCE = 0.01  # 1cm
YAW_TOLERANCE = 1.0  # 1도
X_TOLERANCE = 0.01 # 1cm

class DDCNode(Node):
    def __init__(self):
        super().__init__('ddc_node')
        
        # 로봇의 내부 상태 초기화
        self.state = DDCState.IDLE
        self.get_logger().info(f'DDC 상태 초기화: {self.state}')

        # 전역 Dobby 상태
        self.dobby_main_state = None

        # ArUco 마커 데이터 저장을 위한 변수
        self.latest_aruco_data = None

        # --- 구독자 ---
        self.create_subscription(ArucoDockingData, 'aruco_docking_data', self.aruco_callback, 10)
        self.create_subscription(DobbyState, 'status/robot_state', self.dobby_state_callback, 10)

        # --- 발행자 ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- 액션 클라이언트 ---
        # Nav2 스택에 연결하기 위한 액션 클라이언트
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- 액션 서버 ---
        # 상위 노드(DMC)로부터 내비게이션 목표를 받기 위한 액션 서버
        self.nav_to_pose_server = ActionServer(
            self,
            NavigateToPose,
            'drive/navigate_to_pose', # dmc에서 호출하는 토픽명
            self.nav_to_pose_callback)
        
        self.get_logger().info("Dobby Drive Controller (DDC) 노드가 시작되었습니다.")

    def nav_to_pose_callback(self, goal_handle):
        """
        NavigateToPose 액션 요청을 처리하는 콜백 (DMC -> DDC).
        내비게이션과 도킹을 순차적으로 수행하고 전체 과정이 끝나면 결과를 반환합니다.
        """
        # --- 1. 전역 상태 및 내부 상태 확인 ---
        if self.dobby_main_state is None:
            msg = "Dobby 전역 상태를 수신하지 못해 작업을 거부합니다."
            self.get_logger().error(msg)
            goal_handle.abort()
            return NavigateToPose.Result()

        # 비상 정지 상태에서는 모든 작업을 거부
        if self.dobby_main_state == MainState.EMERGENCY_STOP.value:
            msg = f"비상 정지 상태에서는 내비게이션을 시작할 수 없습니다."
            self.get_logger().warn(msg)
            goal_handle.abort()
            return NavigateToPose.Result()

        if self.state != DDCState.IDLE:
            msg = f"내비게이션을 시작할 수 없습니다. DDC가 IDLE 상태가 아닙니다. 현재 상태: {self.state}"
            self.get_logger().warn(msg)
            goal_handle.abort()
            return NavigateToPose.Result()

        goal_pose = goal_handle.request.pose
        self.get_logger().info(f"DMC로부터 새로운 목표 수신: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}")
        
        # --- 2. 내비게이션 수행 ---
        self.get_logger().info("Nav2로 목표를 전달하고 내비게이션을 시작합니다.")
        self.state = DDCState.NAVIGATING
        
        self._nav2_client.wait_for_server()
        nav2_goal_msg = NavigateToPose.Goal()
        nav2_goal_msg.pose = goal_pose
        
        send_goal_future = self._nav2_client.send_goal_async(nav2_goal_msg)
        
        # MultiThreadedExecutor를 사용하므로, future.result()를 직접 호출하여 블로킹
        nav2_goal_handle = send_goal_future.result()
        if not nav2_goal_handle.accepted:
            self.get_logger().error('Nav2 목표가 거부되었습니다.')
            self.state = DDCState.IDLE
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info('Nav2 목표가 수락되었습니다. 결과 대기 중...')
        result_future = nav2_goal_handle.get_result_async()
        nav2_result = result_future.result().result
        
        # Nav2 결과 확인 (성공 여부에 따라 분기)
        if nav2_result is None: # 실제로는 status 코드를 확인해야 함
             self.get_logger().error("내비게이션 실패: Nav2로부터 결과를 받지 못했습니다.")
             self.state = DDCState.IDLE
             goal_handle.abort()
             return NavigateToPose.Result()

        self.get_logger().info(f"내비게이션 완료. DOCKING 상태로 전환합니다.")
        self.state = DDCState.DOCKING
        
        # --- 3. 도킹 수행 ---
        docking_start_time = self.get_clock().now()
        DOCKING_TIMEOUT_SEC = 30.0 # 30초 타임아웃

        while rclpy.ok():
            # 타임아웃 확인
            elapsed_time = (self.get_clock().now() - docking_start_time).nanoseconds / 1e9
            if elapsed_time > DOCKING_TIMEOUT_SEC:
                self.get_logger().error("도킹 시간 초과!")
                self.stop_robot()
                self.state = DDCState.IDLE
                goal_handle.abort()
                return NavigateToPose.Result()

            # 도킹 로직 실행
            self.perform_docking()

            # 도킹이 성공하면 perform_docking 내부에서 self.state를 IDLE로 변경
            if self.state == DDCState.IDLE:
                self.get_logger().info("내비게이션 및 도킹 작업 전체 성공!")
                goal_handle.succeed()
                return NavigateToPose.Result()
            
            # 취소 요청 확인
            if goal_handle.is_cancel_requested:
                self.get_logger().info("작업 취소 요청 수신. 도킹을 중단합니다.")
                self.stop_robot()
                self.state = DDCState.IDLE
                goal_handle.canceled()
                return NavigateToPose.Result()

            time.sleep(0.1) # 루프 주기

        # rclpy.ok()가 False가 되면 루프 종료
        self.get_logger().warn("RCLPY가 종료되어 도킹 작업을 중단합니다.")
        self.state = DDCState.IDLE
        goal_handle.abort()
        return NavigateToPose.Result()

    def aruco_callback(self, msg):
        """ArUco 마커 데이터를 수신하고 저장하는 콜백"""
        self.latest_aruco_data = msg

    def dobby_state_callback(self, msg):
        """Dobby의 전역 상태를 수신하는 콜백"""
        self.dobby_main_state = msg.main_state
        self.get_logger().info(f"Dobby 전역 상태 수신: {MainState(msg.main_state).name} => {msg.main_state}")

    def perform_docking(self):
        """정밀 도킹 로직을 수행"""
        if self.latest_aruco_data is None:
            self.get_logger().warn("DOCKING 상태이지만 ArUco 데이터를 수신하지 못했습니다.", once=True)
            return

        # 오차 계산
        x_error = self.latest_aruco_data.marker_pos_x
        z_error = self.latest_aruco_data.marker_pos_z - TARGET_DISTANCE
        yaw_error_deg = self.latest_aruco_data.marker_yaw

        # 오차가 허용 범위 내에 있는지 확인
        if abs(x_error) < X_TOLERANCE and abs(z_error) < DISTANCE_TOLERANCE and abs(yaw_error_deg) < YAW_TOLERANCE:
            self.get_logger().info("도킹 성공!")
            self.state = DDCState.IDLE
            self.stop_robot()
            self.latest_aruco_data = None # 다음 도킹을 위해 초기화
            return

        # Twist 메시지 생성
        twist_msg = Twist()

        # 간단한 P-제어 (동시 제어)
        # 1. Yaw (회전) 제어: 로봇이 마커를 정면으로 보도록 회전
        if abs(yaw_error_deg) > YAW_TOLERANCE:
            # yaw_error > 0 이면 마커가 로봇의 왼쪽에 치우쳐 있다는 의미일 수 있으므로, 왼쪽으로 회전 (양수 각속도)
            twist_msg.angular.z = DOCKING_SPEED_ANGULAR if yaw_error_deg > 0 else -DOCKING_SPEED_ANGULAR
        else:
            twist_msg.angular.z = 0.0

        # 2. X (좌/우) 제어: 로봇이 마커의 중앙에 오도록 좌/우로 이동
        if abs(x_error) > X_TOLERANCE:
            # x_error > 0 이면 마커가 카메라 오른쪽에 있으므로, 로봇을 오른쪽으로 이동 (음수 y속도)
            twist_msg.linear.y = -DOCKING_SPEED_STRAFE if x_error > 0 else DOCKING_SPEED_STRAFE
        else:
            twist_msg.linear.y = 0.0

        # 3. Z (앞/뒤) 제어: 로봇이 목표 거리에 도달하도록 앞/뒤로 이동
        if abs(z_error) > DISTANCE_TOLERANCE:
            twist_msg.linear.x = DOCKING_SPEED_LINEAR if z_error > 0 else -DOCKING_SPEED_LINEAR
        else:
            twist_msg.linear.x = 0.0
            
        self.get_logger().info(f"도킹 중... [X Err: {x_error:.3f} m, Z Err: {z_error:.3f} m, Yaw Err: {yaw_error_deg:.2f} deg] -> [Vel X: {twist_msg.linear.x:.2f}, Y: {twist_msg.linear.y:.2f}, Ang: {twist_msg.angular.z:.2f}]")
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        """로봇을 정지시키는 Twist 메시지를 발행"""
        self.get_logger().info("로봇을 정지합니다.")
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    ddc_node = DDCNode()
    # 멀티 스레드 실행기 사용 (액션 서버/클라이언트 동시 처리)
    executor = MultiThreadedExecutor()
    executor.add_node(ddc_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        ddc_node.get_logger().info("키보드 인터럽트를 수신했습니다. 노드를 종료합니다.")
    finally:
        ddc_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
