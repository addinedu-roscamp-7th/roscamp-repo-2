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
import time
import tf_transformations
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .reid_tracker_module import ReIDTracker
import numpy as np

# 추적 안정성 및 로봇 제어 변수
LOST_FRAMES_THRESHOLD = 30     # 30프레임 이상 추적 실패 시 재탐지 시도 (약 1초)


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
        self.person_detected = False
        self.initial_person_detected = False
        self.initial_pause_active = False
        self.initial_pause_timer = None
        
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

        self.person_lost_timeout_seconds = 5.0
        self.start_pose = None
        self.person_lost_timestamp = None
        self.person_lost_timeout_occured = False

        # --- 2. Re-ID Tracker 설정 ---
        self.tracker = ReIDTracker(reid_model_path="osnet_x0_25_msmt17.pt")
        self.get_logger().info('Re-ID Tracker initialized successfully.')
        self.camera_matrix = np.array([
            [694.598257, 0.000000, 300.324333],
            [0.000000, 696.536683, 237.763347],
            [0.000000, 0.000000, 1.000000],
        ], dtype=np.float32)
        self.dist_coeffs = np.array([-0.006456, -0.046534, -0.000181, 0.002512, 0.000000], dtype=np.float32)

        self.declare_parameter('video_device', '/dev/video2')
        video_device = self.get_parameter('video_device').get_parameter_value().string_value

        self.cap = None

        # --- 3. 상태 변수 ---
        self.is_robot_moving = True
        self.goal_sent = False

        self.get_logger().info('Guide Navigation Node Started successfully.')

        # <--- 추가된 부분: 로봇 정지를 위한 /cmd_vel 퍼블리셔 ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def img_timer_callback(self):
        if self.current_frame is not None:
            ros_image_message = self.br.cv2_to_imgmsg(self.current_frame, encoding="bgr8")
            self._img_pub.publish(ros_image_message)

    def execute_callback(self, goal_handle):
        self.get_logger().info("이동 명령 실행")
        
        # --- 카메라 초기화 ---
        video_device = self.get_parameter('video_device').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(0) # Using device 0 as per original code
        if not self.cap.isOpened():
            self.get_logger().error(f"WebCam을 열 수 없습니다: {video_device}")
            goal_handle.abort()
            result = GuideNavigationAction.Result()
            result.success = False
            result.termination_reason = f'Failed to open camera device: {video_device}'
            return result

        self.get_logger().info("카메라가 성공적으로 열렸습니다.")
        self.timer = self.create_timer(self.timer_period, self.process_frame_callback)
        self.goal_handle = goal_handle

        # Store the starting position
        if self.amcl_pose:
            self.start_pose = self.convert_to_pose_stamped(self.amcl_pose)
        else:
            self.start_pose = self.init_pose() 
        self.person_lost_timestamp = None
        self.person_lost_timeout_occured = False

        request = goal_handle.request
        feedback_msg = GuideNavigationAction.Feedback()
        result = GuideNavigationAction.Result()
        self.goal_sent = True
        self.is_robot_moving = True
        
        self.get_logger().info("이동 시작")

        goal_pose = self.set_goal_pose(request)
        if self.amcl_pose is None:
            self.amcl_pose = self.init_pose_with_covariance()
        
        dobby_path = self.nav2.getPath(self.convert_to_pose_stamped(self.amcl_pose), goal_pose)
        self._path_pub.publish(dobby_path)

        self.nav2.goToPose(goal_pose)

        while not self.nav2.isTaskComplete():
            if self.goal_handle.is_cancel_requested:
                self.nav2.cancelTask()
                self.get_logger().info('액션 서버에서 내비게이션 취소 요청을 수락했습니다.')
                break 

            if self.person_lost_timeout_occured:
                break

            if not self.is_robot_moving:
                pass 

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

        nav2_result = self.nav2.getResult()

        if self.person_lost_timeout_occured:
            self.get_logger().info(f"{self.person_lost_timeout_seconds}초간 사람 감지 실패. 출발 지점으로 복귀합니다.")
            
            # 복귀 전 사람 감지 타이머 중지
            if self.timer:
                self.timer.destroy()
                self.timer = None

            self.nav2.goToPose(self.start_pose)
            while not self.nav2.isTaskComplete():
                time.sleep(0.1)
            
            self.get_logger().info("출발 지점 복귀 완료.")
            goal_handle.succeed() # Abort 대신 Succeed로 변경
            result.success = True
            result.final_location = self.convert_to_pose2d(self.amcl_pose)
            result.termination_reason = f'Person lost for {self.person_lost_timeout_seconds} seconds, returned to start.'
        
        elif nav2_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 지점 도착 성공!')
            goal_handle.succeed()
            result.success = True
            result.final_location = self.convert_to_pose2d(self.amcl_pose)
            result.termination_reason = 'Navigation succeeded'
        
        elif nav2_result == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('내비게이션이 외부 요청에 의해 취소되었습니다.')
            goal_handle.canceled()
            result.success = False
            result.final_location = self.convert_to_pose2d(self.amcl_pose)
            result.termination_reason = 'Navigation was canceled by external request'
        
        else:
            self.get_logger().error(f'목표 지점 도착 실패! 상태: {nav2_result}')
            goal_handle.abort()
            result.success = False
            result.final_location = self.convert_to_pose2d(self.amcl_pose)
            result.termination_reason = f'Navigation failed with status: {nav2_result}'

        self._cleanup_goal_resources()
        return result

    def _cleanup_goal_resources(self):
        """안내 액션에 사용된 리소스를 정리합니다 (타이머, 카메라 등)."""
        self.get_logger().info("가이드 액션 리소스 정리.")
        
        if self.timer:
            self.timer.destroy()
            self.timer = None
        
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None
            self.get_logger().info("카메라가 해제되었습니다.")

        # 상태 변수 초기화
        self.goal_sent = False
        self.is_robot_moving = True
        self.person_detected = False
        self.initial_person_detected = False
        self.initial_pause_active = False
        if self.initial_pause_timer:
            self.initial_pause_timer.cancel()
            self.initial_pause_timer = None

    def init_pose_with_covariance(self):
        init_pose_cov = PoseWithCovarianceStamped()
        init_pose_cov.header.frame_id = 'map'
        init_pose_cov.header.stamp = self.nav2.get_clock().now().to_msg()
        return init_pose_cov

    def init_pose(self):
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
    
    def convert_to_pose2d(self, pose_stamped_or_cov_stamped) -> Pose2D:
        """
        PoseStamped 또는 PoseWithCovarianceStamped 메시지를 Pose2D 메시지로 변환합니다.
        """
        pose2d = Pose2D()
        pose = None

        if isinstance(pose_stamped_or_cov_stamped, PoseWithCovarianceStamped):
            pose = pose_stamped_or_cov_stamped.pose.pose
        elif isinstance(pose_stamped_or_cov_stamped, PoseStamped):
            pose = pose_stamped_or_cov_stamped.pose
        else:
            self.get_logger().warn("Pose2D로 변환할 수 없는 알 수 없는 Pose 타입입니다.")
            return pose2d # 기본값 Pose2D 반환

        pose2d.x = pose.position.x
        pose2d.y = pose.position.y
        q = pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose2d.theta = yaw
        return pose2d
    
    def amcl_callback(self, msg):
        self.amcl_pose = msg

    def goal_callback(self, goal_request: GuideNavigationAction.Goal):
        self.get_logger().info(f"이동 명령 요청 {goal_request.destination}, 최대속도: {goal_request.max_speed}, 안전최소거리: {goal_request.person_follow_distance}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("이동 명령 취소")
        return CancelResponse.ACCEPT

    def pause(self):
        if self.is_robot_moving:
            self.get_logger().info("일시정지 요청. 로봇을 멈춥니다.")
            self.is_robot_moving = False
    
    def resume_robot_moving(self, reason="Tracking Resumed"):
        if not self.is_robot_moving:
            self.is_robot_moving = True
            self.get_logger().info(f"Robot Starts Moving, Reason: {reason}")

        

    def end_initial_pause(self):
        self.get_logger().info('20초 정지가 완료되었습니다. 추적을 재개합니다.')
        self.initial_pause_active = False
        self.resume_robot_moving(reason="Initial 20s pause finished")
        if self.initial_pause_timer:
            self.initial_pause_timer.cancel()
            self.initial_pause_timer = None

    def process_frame_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("WebCam에서 Frame을 읽을 수 없습니다!")
            return

        frame_height, frame_width = frame.shape[:2]
        
        tracks = self.tracker.update(frame)
        
        self.person_detected = tracks.shape[0] > 0

        if self.person_detected:
            self.person_lost_timestamp = None # Reset timer
            primary_target = self.tracker.get_primary_target(tracks)
            if primary_target is not None:
                if not self.initial_pause_active:
                    self.resume_robot_moving(reason="Target in sight")
                
                x1, y1, x2, y2, track_id = primary_target[:5]
                x1, y1, x2, y2, track_id = map(int, [x1, y1, x2, y2, track_id])

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"visitor", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                w = x2 - x1

                pose_msg = Pose2D()
                pose_msg.x = center_x - frame_width / 2.0
                pose_msg.y = center_y - frame_height / 2.0
                pose_msg.theta = float(w)

                if not self.initial_person_detected:
                    self.initial_person_detected = True
                    self.get_logger().info('사람이 처음 감지되었습니다. 20초간 정지합니다.')
                    self.initial_pause_active = True
                    self.pause()
                    self.initial_pause_timer = self.create_timer(20.0, self.end_initial_pause)

                self.pose_publisher.publish(pose_msg)
            else:
                self.pause()
        else: # Person not detected
            self.pause()
            self.get_logger().info("사람 감지 실패")
            if self.person_lost_timestamp is None:
                self.person_lost_timestamp = time.monotonic()
                self.get_logger().info(f"사람 감지 실패. {self.person_lost_timeout_seconds}초 타이머를 시작합니다.")
            elif time.monotonic() - self.person_lost_timestamp > self.person_lost_timeout_seconds:
                if not self.person_lost_timeout_occured:
                    self.get_logger().info(f"{self.person_lost_timeout_seconds}초 동안 사람 감지 실패. 복귀 절차를 시작합니다.")
                    self.person_lost_timeout_occured = True
                    self.nav2.cancelTask() # This will break the loop in execute_callback

        if self.goal_sent and not self.is_robot_moving:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            if not self.initial_person_detected:
                self.get_logger().info("일시정지 중")

        if cv2.waitKey(1) & 0xFF == 27:
            self.destroy_node()
        self.current_frame = frame

    def destroy_node(self):
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
