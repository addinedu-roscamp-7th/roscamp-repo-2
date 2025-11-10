import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose2D, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from javis_interfaces.action import GuideNavigation as GuideNavigationAction
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose2D, Twist
import cv2
import math
import tf_transformations
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from javis_interfaces.msg import TrackingStatus

FACE_CASCADE_PATH = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'

# 추적 안정성 및 로봇 제어 변수
LOST_FRAMES_THRESHOLD = 30     # 30프레임 이상 추적 실패 시 재탐지 시도 (약 1초)
STOP_TIMEOUT_SECONDS = 10.0    # 추적 실패 후 로봇 정지 유지 시간


class GuideNavigation(Node):
    def __init__(self):
        super().__init__('guide_navigation')
        self.goal_pose = PoseStamped()
        self.nav2 = BasicNavigator()
        self.nav2.waitUntilNav2Active()
        initial_pose = self.init_pose()
        self.nav2.setInitialPose(initial_pose)

        self.dobby_nav2_server = ActionServer(self, GuideNavigationAction,
                                              'dobby1/drive/guide_navigation',
                                              execute_callback=self.execute_callback,
                                              goal_callback=self.goal_callback,
                                              cancel_callback=self.cancel_callback)
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        self._path_pub = self.create_publisher(
            Path,
            '/dobby_path',
            10
        )
        self._img_pub = self.create_publisher(
            Image,
            '/dobby_img',
            10
        )
        self.detect_pub = self.create_publisher(
            TrackingStatus,
            'dobbb1/ai/tracking/status',
            10
        )
        self.goal_handle = None
        self.amcl_pose = None # 초기화

        timer_period = 0.1
        self.img_timer = self.create_timer(timer_period, self.img_timer_callback)
        self.br = CvBridge()
        self.current_frame = None
        
        self.goal_handle = None
        self.amcl_pose = None # 초기화

        self.pose_publisher = self.create_publisher(Pose2D, 'tracking_person_pose', 10)
        self.timer_period = 1.0 / 30.0  # 30 FPS 처리 주기
        self.timer = self.create_timer(self.timer_period, self.process_frame_callback)
        self.stop_timer = None          # 로봇 정지 시간 타이머

        # --- 2. OpenCV 설정 ---
        self.face_cascade = cv2.CascadeClassifier(FACE_CASCADE_PATH)
        if self.face_cascade.empty():
            self.get_logger().error(f"Haar Cascade 파일을 로드할 수 없습니다. 경로를 확인하세요: {FACE_CASCADE_PATH}")
            self.destroy_node()
            return

        self.tracker = None  # CSRT 트래커 인스턴스
        # 웹캠 인덱스 2를 사용하고 있지만, 환경에 따라 0, 1 등으로 변경해야 할 수 있습니다.
        self.cap = cv2.VideoCapture(0) 

        if not self.cap.isOpened():
            self.get_logger().error("WebCam을 열 수 없습니다.")
            self.destroy_node()
            return

        # --- 3. 상태 변수 ---
        self.tracking = False
        self.lost_frames = 0
        self.is_robot_moving = True
        self.goal_sent = False

        self.get_logger().info('OpenCV CSRT Visitor Tracking Node Started successfully.')

        # <--- 추가된 부분: 로봇 정지를 위한 /cmd_vel 퍼블리셔 ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    def img_timer_callback(self):
        if self.current_frame is not None:
            ros_image_message = self.br.cv2_to_imgmsg(self.current_frame, encoding="bgr8")
            self._img_pub.publish(ros_image_message)


 

    async def execute_callback(self, goal_handle):
        self.get_logger().info("이동 명령 실행")
        self.goal_handle = goal_handle

        request = goal_handle.request
        feedback_msg = GuideNavigationAction.Feedback()
        result = GuideNavigationAction.Result()
        self.goal_sent = True
        self.is_robot_moving = True # <--- 추가된 부분: 새로운 목표 시작 시 항상 움직임 상태로 설정

        self.get_logger().info("이동 시작")

        goal_pose = self.set_goal_pose(request)
        dobby_path = Path()
        if self.amcl_pose is None:
            # amcl_pose가 없는 경우 초기 포즈를 사용하거나 에러 처리
            self.amcl_pose = self.init_pose_with_covariance()
        
        # getPath는 BasicNavigator에서 직접 PoseWithCovarianceStamped를 인수로 받지 않으므로 변환
        dobby_path = self.nav2.getPath(self.convert_to_pose_stamped(self.amcl_pose), goal_pose)
        self._path_pub.publish(dobby_path)

        # goToPose를 사용하여 목표 지점으로 이동 요청
        # nav2_simple_commander는 이미 비동기적으로 동작하므로, 이동 목표를 취소하지 않도록 주의
        self.nav2.goToPose(goal_pose)

        # 작업이 완료될 때까지 피드백 전송
        while not self.nav2.isTaskComplete():
            # 취소 요청 확인
            if self.goal_handle.is_cancel_requested:
                self.nav2.cancelTask()
                self.get_logger().info('액션 서버에서 내비게이션 취소 요청을 수락했습니다.')
                # GoalStatus.STATUS_CANCELED 상태는 nav2_result에서 처리되므로 break
                break 

            # 로봇이 멈춰야 하는 상태라면, 내비게이션 명령을 일시적으로 중단
            if not self.is_robot_moving:
                
                pass # 로봇이 멈춘 상태에서는 피드백만 계속 전송

            feedback = self.nav2.getFeedback()
            if feedback:

                pose2d = Pose2D()
                pose2d.x = self.amcl_pose.pose.pose.position.x
                pose2d.y = self.amcl_pose.pose.pose.position.y
                    
                q = self.amcl_pose.pose.pose.orientation
                _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                pose2d.theta = yaw
                    
                feedback_msg.current_location = pose2d

                feedback_msg.distance_remaining = feedback.distance_remaining
                feedback_msg.status = "길안내중"
                feedback_msg.person_detected = self.person_detected
                goal_handle.publish_feedback(feedback_msg)


        # 결과 처리
        nav2_result = self.nav2.getResult()

        if nav2_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 지점 도착 성공!')
            goal_handle.succeed()
            result.success = True
            result.final_location = self.amcl_pose
            result.termination_reason = 'Navigation succeeded'

        # <--- 수정된 부분: 외부 클라이언트에 의한 취소 처리 ---
        elif nav2_result == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('내비게이션이 외부 요청에 의해 취소되었습니다.')
            goal_handle.canceled()
            result.success = False
            result.termination_reason = 'Navigation was canceled by external request'

        else:
            self.get_logger().error(f'목표 지점 도착 실패! 상태: {nav2_result}')
            goal_handle.abort()
            result.success = False
            result.termination_reason = f'Navigation failed with status: {nav2_result}'

        # <--- 추가된 부분: 액션 종료 후 상태 변수 초기화 ---
        self.goal_sent = False
        self.is_robot_moving = True # 다음 목표를 위해 기본값으로 설정
        return result

    def init_pose_with_covariance(self):
        """임시 PoseWithCovarianceStamped를 생성합니다."""
        init_pose_cov = PoseWithCovarianceStamped()
        init_pose_cov.header.frame_id = 'map'
        init_pose_cov.header.stamp = self.nav2.get_clock().now().to_msg()
        # 나머지 필드는 0으로 기본 설정됩니다.
        return init_pose_cov

    def init_pose(self):

        # 초기 포즈의 yaw 값을 명확히 지정합니다.
        initial_yaw_degrees = -3.08823
        q = self.get_quaternion_from_yaw(initial_yaw_degrees)

        init_pose_msg = PoseStamped()
        init_pose_msg.header.frame_id = 'map'
        init_pose_msg.header.stamp = self.nav2.get_clock().now().to_msg()
        init_pose_msg.pose.position.x = 0.0
        init_pose_msg.pose.position.y = 0.0
        init_pose_msg.pose.position.z = 0.0
        init_pose_msg.pose.orientation.x = q[0]
        init_pose_msg.pose.orientation.y = q[1]
        init_pose_msg.pose.orientation.z = q[2]
        init_pose_msg.pose.orientation.w = q[3]
        return init_pose_msg

    def set_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav2.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal.destination.position.x
        goal_pose.pose.position.y = goal.destination.position.y
        goal_pose.pose.position.z = goal.destination.position.z
        goal_pose.pose.orientation.x = goal.destination.orientation.x
        goal_pose.pose.orientation.y = goal.destination.orientation.y
        goal_pose.pose.orientation.z = goal.destination.orientation.z
        goal_pose.pose.orientation.w = goal.destination.orientation.w
        return goal_pose

    def get_quaternion_from_yaw(self, yaw_degrees):
        yaw_radians = math.radians(yaw_degrees)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)
        return quaternion
    def convert_to_pose_stamped(self, cov_stamped_msg: PoseWithCovarianceStamped) -> PoseStamped:
        new_pose_stamped = PoseStamped()
        new_pose_stamped.header = cov_stamped_msg.header
        new_pose_stamped.pose = cov_stamped_msg.pose.pose
        return new_pose_stamped
    
    def handle_tracking_loss(self):
        self.lost_frames += 1

        if self.lost_frames > LOST_FRAMES_THRESHOLD:
            self.tracking = False
            self.tracker = None
            self.lost_frames = 0
            self.get_logger().info("Tracking Lost, Detecting Person...")

    
    def amcl_callback(self, msg):
        self.amcl_pose = msg
    def goal_callback(self, goal_request: GuideNavigationAction.Goal):
        self.get_logger().info(f"이동 명령 요청 {goal_request.destination}, 최대속도: {goal_request.max_speed}, 안전최소거리: {goal_request.person_follow_distance}")

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("이동 명령 취소")
        return CancelResponse.ACCEPT

    # <--- 수정된 부분: pause 함수 ---
    def pause(self):
        """로봇을 일시정지 상태로 변경합니다. 실제 정지 명령은 process_frame_callback에서 처리됩니다."""
        if self.is_robot_moving:
            self.get_logger().info("일시정지 요청. 로봇을 멈춥니다.")
            self.is_robot_moving = False

    
    def init_pose_with_covariance(self):
        """임시 PoseWithCovarianceStamped를 생성합니다."""
        init_pose_cov = PoseWithCovarianceStamped()
        init_pose_cov.header.frame_id = 'map'
        init_pose_cov.header.stamp = self.nav2.get_clock().now().to_msg()
        # 나머지 필드는 0으로 기본 설정됩니다.
        return init_pose_cov
    
    def resume_robot_moving(self, reason="Tracking Resumed"):
        """로봇을 다시 주행 상태로 변경합니다."""
        if not self.is_robot_moving:
            self.is_robot_moving = True
            self.get_logger().info(f"Robot Starts Moving, Reason: {reason}")

        if self.stop_timer:
            self.stop_timer.destroy()
            self.stop_timer = None

    # <--- 수정된 부분: process_frame_callback 함수 ---
    def process_frame_callback(self):
        """ 타이머에 의해 주기적으로 호출되는 콜백 함수: 프레임 읽기, 처리 및 로봇 이동 제어 """

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("WebCam에서 Frame을 읽을 수 없습니다!")
            return

        frame_height, frame_width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.tracking and self.tracker is not None:
            # --- 1. 추적 업데이트 ---
            success, bbox = self.tracker.update(frame)
            self.person_detected = success

            if success:
                # 추적 성공 시 로봇 이동 재개
                self.update_tracking(frame, frame_width, frame_height, bbox)
                

            else:
                # 추적 실패 시 로봇 일시정지
                self.handle_tracking_loss()
                self.pause()

            trackingStatus_msg = TrackingStatus()
            trackingStatus_msg.header.frame_id = 'map'
            trackingStatus_msg.header.stamp = self.nav2.get_clock().now().to_msg()
            trackingStatus_msg.person_detected = success
            trackingStatus_msg.tracking_id = 'visitor'
            trackingStatus_msg.person_pose.position.x = 0.0
            trackingStatus_msg.person_pose.position.y = 0.0
            trackingStatus_msg.person_pose.position.z = 0.0
            trackingStatus_msg.distance_to_person = 0.0
            trackingStatus_msg.confidence = 0.0
            trackingStatus_msg.is_lost = False
            trackingStatus_msg.time_since_last_seen = 0.0
            self.detect_pub.publish(trackingStatus_msg)
        else:
            # --- 2. 추적 중이 아닐 때: 재감지 시도 ---
            self.attempt_detection(frame, gray)
            # 감지에 성공하여 추적이 시작되지 않았다면, 로봇은 정지 상태여야 함
            if not self.tracking:
                self.pause()

        # --- 3. 로봇 이동 제어 ---
        # is_robot_moving 상태가 False이고, 내비게이션 목표가 활성화된 상태일 때만 정지 명령 전송
        # self.is_robot_moving이 True인 경우에도, process_frame_callback은 단순히 정지 명령을 보내지 않을 뿐,
        # 로봇은 Nav2에 의해 계속 움직이게 됩니다.
        if self.goal_sent and not self.is_robot_moving:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

        # cv2.imshow('OpenCV_CSRT_Tracking', frame)
        # if cv2.waitKey(1) & 0xFF == 27:
        #     self.destroy_node()
        self.current_frame = frame


    def update_tracking(self, frame, frame_width, frame_height, bbox):
        """ 추적 중일 때 트래커를 업데이트하고 결과를 처리합니다. """
        x, y, w, h = [int(v) for v in bbox]

        # 사람이 돌아왔으므로, 로봇 이동 재개
        self.resume_robot_moving(reason="Tracking Activated")

        # 추적 결과 그리기
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "Visitor", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        self.lost_frames = 0

        # 추적 결과 Pose2D 메시지로 변환 및 발행
        center_x = x + w / 2.0
        center_y = y + h / 2.0

        pose_msg = Pose2D()
        pose_msg.x = center_x - frame_width / 2.0       # 화면 중앙 기준 X 좌표
        pose_msg.y = center_y - frame_height / 2.0      # 화면 중앙 기준 Y 좌표
        pose_msg.theta = float(w)                       # 감지된 객체의 크기 (가까울수록 커짐)

        self.pose_publisher.publish(pose_msg)

    def attempt_detection(self, frame, gray):
        """ 추적에 실패했을 때 얼굴을 다시 감지하고 추적을 시작합니다. """

        # Haar Cascade 감지
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.05, # 이미지 축소 비율
            minNeighbors=8,  # 최소 이웃 수
            minSize=(80, 80) # 최소 얼굴 크기
        )

        if len(faces) > 0:
            # 가장 큰 얼굴 (가장 가까운 사람) 찾기
            max_area = 0
            best_bbox = None
            for (x, y, w, h) in faces:
                area = w * h
                if area > max_area:
                    max_area = area
                    best_bbox = (x, y, w, h)

            if best_bbox:
                # CSRT 트래커 재설정 및 추적 시작
                self.tracker = cv2.TrackerCSRT_create()
                self.tracker.init(frame, best_bbox)
                self.tracking = True
                self.get_logger().info(f"Detection Success. Starting CSRT tracking.")

                # 감지된 얼굴에 사각형 그리기
                x, y, w, h = [int(v) for v in best_bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, "Visitor", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                self.current_frame = frame


    def destroy_node(self):
        """ 노드 종료 시 자원을 해제합니다. """
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()




def main(args=None):
    rclpy.init(args=args)
    node = GuideNavigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('guide_navigation 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
