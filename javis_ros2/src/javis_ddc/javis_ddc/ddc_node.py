import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
import time

from geometry_msgs.msg import PoseStamped, Twist, Point
from nav2_msgs.action import NavigateToPose
from javis_interfaces.msg import DobbyState
from javis_dmc.states.state_enums import MainState
from rclpy.qos import QoSProfile, DurabilityPolicy

# DDC 노드 내부에서 사용하는 상태 정의
class DDCState:
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    DOCKING = 'DOCKING'

class ArucoDockingPID:
    def __init__(self, node: Node):
        self.node = node
        self.logger = self.node.get_logger()

        # PID 파라미터 선언 및 초기화
        self.node.declare_parameter('docking.target_z', 0.65)
        self.node.declare_parameter('docking.stop_tolerance_z', 0.02)
        self.node.declare_parameter('docking.stop_tolerance_x', 0.12)
        self.node.declare_parameter('docking.kp_z', 0.20)
        self.node.declare_parameter('docking.ki_z', 0.0)
        self.node.declare_parameter('docking.kd_z', 0.05)
        self.node.declare_parameter('docking.kp_x', 1.2)
        self.node.declare_parameter('docking.ki_x', 0.0)
        self.node.declare_parameter('docking.kd_x', 0.9)
        self.node.declare_parameter('docking.max_linear_speed', 0.10)
        self.node.declare_parameter('docking.max_angular_speed', 0.9)

        self.target_z = self.node.get_parameter('docking.target_z').value
        self.stop_tolerance_z = self.node.get_parameter('docking.stop_tolerance_z').value
        self.stop_tolerance_x = self.node.get_parameter('docking.stop_tolerance_x').value
        self.kp_z = self.node.get_parameter('docking.kp_z').value
        self.ki_z = self.node.get_parameter('docking.ki_z').value
        self.kd_z = self.node.get_parameter('docking.kd_z').value
        self.kp_x = self.node.get_parameter('docking.kp_x').value
        self.ki_x = self.node.get_parameter('docking.ki_x').value
        self.kd_x = self.node.get_parameter('docking.kd_x').value
        self.max_linear_speed = self.node.get_parameter('docking.max_linear_speed').value
        self.max_angular_speed = self.node.get_parameter('docking.max_angular_speed').value
        
        self.target_x = 0.0
        self.integral_z = 0.0
        self.last_error_z = 0.0
        self.integral_x = 0.0
        self.last_error_x = 0.0
        self.reached_target = False

        self.logger.info(f"Aruco Docking PID 활성화 (목표 거리={self.target_z}, 중앙정렬 x={self.target_x})")

    def compute_velocity(self, msg: Point) -> Twist:
        twist = Twist()
        if self.reached_target:
            return twist

        x, _, z = msg.x, msg.y, msg.z

        error_z = self.target_z - z
        self.integral_z += error_z
        derivative_z = error_z - self.last_error_z
        control_z = self.kp_z * error_z + self.ki_z * self.integral_z + self.kd_z * derivative_z
        self.last_error_z = error_z
        control_z = max(min(control_z, self.max_linear_speed), -self.max_linear_speed)

        error_x = x - self.target_x
        self.integral_x += error_x
        derivative_x = error_x - self.last_error_x
        control_x = self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * derivative_x
        self.last_error_x = error_x
        control_x = -control_x
        control_x = max(min(control_x, self.max_angular_speed), -self.max_angular_speed)

        if abs(error_x) < self.stop_tolerance_x:
            control_x = 0.0

        if abs(error_z) < self.stop_tolerance_z and abs(error_x) < self.stop_tolerance_x:
            self.reached_target = True
            self.logger.info(f"🎯 정렬 및 목표 거리 도달: z={z:.3f}, x={x:.3f} → 정지")
            return twist

        twist.linear.x = control_z
        twist.angular.z = control_x
        
        self.logger.info(
            f"[PID] z={z:.3f} err_z={error_z:.3f} → v={control_z:.3f} | "
            f"x={x:.3f} err_x={error_x:.3f} → w={control_x:.3f}"
        )
        return twist

    def reset(self):
        self.integral_z = 0.0
        self.last_error_z = 0.0
        self.integral_x = 0.0
        self.last_error_x = 0.0
        self.reached_target = False
        self.logger.info("🔄 PID 상태 리셋 완료")

class DDCNode(Node):
    def __init__(self):
        super().__init__('ddc_node')
        self.get_logger().info(f"Node Namespace: {self.get_namespace()}")
        
        # 로봇의 내부 상태 초기화
        self.state = DDCState.IDLE
        self.get_logger().info(f'DDC 상태 초기화: {self.state}')

        # 전역 Dobby 상태
        self.dobby_main_state = None

        # ArUco 마커 데이터 저장을 위한 변수
        self.latest_aruco_data = None

        self.pid_controller = ArucoDockingPID(self)

        self.DOCKING_TIMEOUT_SEC = 30.0

        # QoS 프로파일 생성 (TRANSIENT_LOCAL 설정)
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Point, '/ai/docking/normalized_data', self.aruco_callback, 10)
        self.create_subscription(DobbyState, 'status/robot_state', self.dobby_state_callback, qos_profile)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 액션 클라이언트 ---
        # Nav2 스택에 연결하기 위한 액션 클라이언트
        self._nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

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
        NavigateToPose 액션 요청을 동기적으로 처리하는 콜백 (DMC -> DDC).
        내비게이션과 도킹을 순차적으로 수행하고 전체 과정이 끝나면 결과를 반환.
        """
        # --- 1. 전역 상태 및 내부 상태 확인 ---
        # 작업 시작 전, 로봇이 명령을 수행할 수 있는 상태인지 확인.
        if self.dobby_main_state is None:
            msg = "Dobby 전역 상태를 수신하지 못해 작업을 진행할 수 없습니다."
            self.get_logger().error(msg)
            goal_handle.abort()
            return NavigateToPose.Result()

        # 비상 정지 상태에서는 모든 작업을 거부.
        if self.dobby_main_state == MainState.EMERGENCY_STOP.value:
            msg = f"비상 정지 상태에서는 내비게이션을 시작할 수 없습니다."
            self.get_logger().warn(msg)
            goal_handle.abort()
            return NavigateToPose.Result()

        # DDC가 다른 작업을 수행 중인 경우, 새로운 작업을 시작하지 않음.
        if self.state != DDCState.IDLE:
            msg = f"내비게이션을 시작할 수 없습니다. DDC가 IDLE 상태가 아닙니다. 현재 상태: {self.state}"
            self.get_logger().warn(msg)
            goal_handle.abort()
            return NavigateToPose.Result()

        goal_pose = goal_handle.request.pose
        self.get_logger().info(f"DMC로부터 새로운 목표 수신: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}")
        
        # --- 2. 내비게이션 수행 ---
        # Nav2 액션 클라이언트를 사용하여 목표 지점까지 자율 주행 시작.
        self.get_logger().info("Nav2로 목표를 전달하고 내비게이션을 시작합니다.")
        self.state = DDCState.NAVIGATING
        
        # Nav2 서버가 준비될 때까지 대기.
        self._nav2_client.wait_for_server()
        nav2_goal_msg = NavigateToPose.Goal()
        nav2_goal_msg.pose = goal_pose
        
        # 목표를 Nav2에 비동기적으로 전송하고, 결과가 올 때까지 동기적으로 대기.
        # rclpy.spin_until_future_complete는 대기하는 동안 다른 콜백이 처리될 수 있도록 함.
        send_goal_future = self._nav2_client.send_goal_async(nav2_goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        nav2_goal_handle = send_goal_future.result()
        if not nav2_goal_handle.accepted:
            self.get_logger().error('Nav2 목표가 거부되었습니다.')
            self.state = DDCState.IDLE
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info('Nav2 목표가 수락되었습니다. 결과 대기 중...')
        result_future = nav2_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        nav2_result = result_future.result().result
        
        if nav2_result is None:
            self.get_logger().error("내비게이션 실패: Nav2로부터 결과를 받지 못했습니다.")
            self.state = DDCState.IDLE
            goal_handle.abort()
            return NavigateToPose.Result()

        # --- 3. 도킹 준비 ---
        # 내비게이션이 성공적으로 완료되면, 상태를 DOCKING으로 전환하고 PID 제어기를 리셋.
        self.get_logger().info(f"내비게이션 완료. DOCKING 상태로 전환합니다.")
        self.state = DDCState.DOCKING
        self.pid_controller.reset()
        
        docking_start_time = self.get_clock().now()
        final_result = NavigateToPose.Result()

        # --- 4. 도킹 수행 (while 루프 사용) ---
        # 타이머 대신 while 루프를 사용하여 도킹 프로세스를 동기적으로 처리.
        while rclpy.ok():
            # 4.1. 콜백 처리를 위해 spin_once 호출
            # 이 부분이 매우 중요. 루프가 실행되는 동안에도 ArUco 마커 데이터 수신과 같은
            # 다른 ROS2 콜백들이 처리될 수 있도록 보장. timeout_sec는 블로킹을 방지.
            rclpy.spin_once(self, timeout_sec=0.1)

            # 4.2. 타임아웃 확인
            # 도킹이 너무 오래 걸리면 작업을 중단하고 실패 처리.
            elapsed_time = (self.get_clock().now() - docking_start_time).nanoseconds / 1e9
            if elapsed_time > self.DOCKING_TIMEOUT_SEC:
                self.get_logger().error("도킹 시간 초과!")
                goal_handle.abort()
                break

            # 4.3. 취소 요청 확인
            # 상위 노드(DMC)에서 액션 취소를 요청했는지 확인.
            if goal_handle.is_cancel_requested:
                self.get_logger().info("작업 취소 요청 수신. 도킹을 중단합니다.")
                goal_handle.canceled()
                break

            # 4.4. 도킹 로직 실행
            # ArUco 마커를 기반으로 PID 제어를 통해 로봇을 정밀하게 움직임.
            self.perform_docking()

            # 4.5. 성공 확인
            # PID 제어기가 목표 지점에 도달했다고 판단하면 루프 종료.
            if self.pid_controller.reached_target:
                self.get_logger().info("도킹 성공! 작업 전체 성공!")
                goal_handle.succeed()
                break

        # --- 5. 정리 ---
        # 도킹 루프가 끝나면 로봇을 정지시키고, DDC 상태를 다시 IDLE로 변경.
        self.stop_robot()
        self.state = DDCState.IDLE

        # 액션 콜백의 최종 결과를 반환.
        return final_result

    def aruco_callback(self, msg):
        """ArUco 마커 데이터를 수신하고 저장하는 콜백"""
        self.latest_aruco_data = msg

    def dobby_state_callback(self, msg):
        """Dobby의 전역 상태를 수신하는 콜백"""
        if self.dobby_main_state != msg.main_state:
            self.dobby_main_state = msg.main_state
            self.get_logger().info(f"Dobby 전역 상태 수신: {MainState(msg.main_state).name} : {msg.main_state}")

    def perform_docking(self):
        """정밀 도킹 로직을 수행"""
        if self.latest_aruco_data is None:
            self.get_logger().warn("DOCKING 상태이지만 ArUco 데이터를 수신하지 못했습니다.", once=True)
            return

        twist_msg = self.pid_controller.compute_velocity(self.latest_aruco_data)
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
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