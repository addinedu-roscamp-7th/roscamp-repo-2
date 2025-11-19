import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose2D, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from javis_interfaces.action import GuideNavigation as GuideNavigationAction
import math
import tf_transformations
import time
from .reid_tracker_module import ReIDTracker
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav2_msgs.srv import ManageLifecycleNodes

REID_PATH = "osnet_x0_25_msmt17.pt"
COMMAND_PAUSE = 1
COMMAND_RESUME = 2

class GuideNavigation(Node):
    def __init__(self):
        super().__init__('guide_navigation')
        self.goal_pose = PoseStamped()
        self.nav2 = BasicNavigator()
        self.nav2.waitUntilNav2Active()
        self.current_frame = None
        self.amcl_pose = self.init_pose_with_covariance()

        self.is_pause = False
        self.goal_pose = None

        self.dobby_nav2_server = ActionServer(self, GuideNavigationAction,
                                              'dobby1/drive/guide_navigation',
                                              execute_callback=self.execute_callback,
                                              goal_callback=self.goal_callback,
                                              cancel_callback=self.cancel_callback)
        self.timer_period = 1.0/30.0
        self.img_pub_timer = self.create_timer(self.timer_period, self.img_pub_timer_callback)

        qos_profile =  QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            qos_profile
        )

        

        self.get_logger().info("Waiting for /amcl_pose...")
        while self.amcl_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Received /amcl_pose.")

        self.nav2.waitUntilNav2Active()
        initial_pose = self.init_pose()
        self.nav2.setInitialPose(initial_pose)

        self._img_pub = self.create_publisher(
            Image,
            'dobby/image',
            10
        )


        self.manager_cli = self.create_client(
            ManageLifecycleNodes, 
            '/lifecycle_manager_navigation/manage_nodes'
        )

        self.get_logger().info('Lifecycle Manager 서비스 대기 중...')
        # 서비스가 준비될 때까지 대기
        if not self.manager_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Lifecycle Manager 서비스를 찾을 수 없습니다. Nav2가 실행 중인지 확인하세요.')
            # 서비스를 찾지 못하면 즉시 종료
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info('Lifecycle Manager 서비스 준비 완료.')

        self.tracker = ReIDTracker(reid_model_path=REID_PATH)

        self.br = CvBridge()
        
        self.img_timer_period = 1.0 /30.0

        self.is_goal_sent = False
        self.is_moving = False
        self.cap = None
        self._img_timer = self.create_timer(self.img_timer_period, self.img_timer_callback)
        
    
    def _manager_service_callback(self, future, command_name):
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info(f'[{command_name}] 호출 성공. 네비게이션 상태 변경 완료.')
            else:
                self.get_logger().error(f'[{command_name}] 호출 실패. 결과: {result}')
        except Exception as e:
            self.get_logger().error(f'서비스 콜백 중 예외 발생: {e}')

    def call_manager_service(self, command_value, command_name):
        """ManageLifecycleNodes 서비스를 호출하는 함수"""
        if not self.manager_cli.service_is_ready():
            self.get_logger().warn(f'서비스가 준비되지 않았습니다. [{command_name}] 호출을 건너뜁니다.')
            return

        req = ManageLifecycleNodes.Request()
        req.command = command_value

        self.get_logger().info(f'[{command_name}] 서비스 호출 시도...')
        
        future = self.manager_cli.call_async(req)
        
        from functools import partial
        callback = partial(self._manager_service_callback, command_name=command_name)
        future.add_done_callback(callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("이동 명령 실행")
        request = goal_handle.request
        self.goal_pose = self.set_goal_pose(request)

        self.cap = cv2.VideoCapture(0)
        self.initial_person_detected = False
        if not self.cap.isOpened():
            self.get_logger().error("WebCam을 열 수 없습니다.")
            goal_handle.abort()
            return GuideNavigationAction.Result(success=False, termination_reason="WebCam 열기 실패")

        feedback_msg = GuideNavigationAction.Feedback()
        
        
        self.nav2.goToPose(self.goal_pose)

        while rclpy.ok():
            if self.cap is None:
                self.cap = cv2.VideoCapture(0)

            if goal_handle.is_cancel_requested:
                self.nav2.cancelTask()
                goal_handle.canceled()
                self.get_logger().info('Goal canceled by client')
                
                self.is_goal_sent = False
                return GuideNavigationAction.Result()
            if self.is_pause:
                if self.cap is None:
                    self.cap = cv2.VideoCapture(0)

                
                self.get_logger().info("Navigation paused. Waiting for person to reappear.")
                while self.is_pause and rclpy.ok() and not goal_handle.is_cancel_requested:
                    if self.amcl_pose:
                        current_pose_2d = self.convert_to_pose2d(self.convert_to_pose_stamped(self.amcl_pose))
                        feedback_msg.current_location = current_pose_2d
                            
                        dist_x = self.goal_pose.pose.position.x - current_pose_2d.x
                        dist_y = self.goal_pose.pose.position.y - current_pose_2d.y
                        feedback_msg.distance_remaining = math.sqrt(dist_x**2 + dist_y**2)

                    feedback_msg.status = "일시중지"
                    feedback_msg.person_detected = False
                    goal_handle.publish_feedback(feedback_msg)
                    time.sleep(0.2)
                    if not rclpy.ok() or goal_handle.is_cancel_requested:
                        break

                    self.img_timer_callback()
                    self.get_logger().info("Resuming navigation monitoring.")
                    time.sleep(0.1) # Allow navigator to reset state after new goal is sent
                    continue
            

            if self.nav2.isTaskComplete():
                nav2_result = self.nav2.getResult()
                # Task completed for a real reason
                self.get_logger().info(f'nav2_result : {nav2_result}')
                result = GuideNavigationAction.Result()
                if self.amcl_pose:
                    self.destroy_cap()
                    result.final_location = self.convert_to_pose2d(self.convert_to_pose_stamped(self.amcl_pose))

                if nav2_result == TaskResult.SUCCEEDED:
                    goal_handle.succeed()
                    result.success = True
                    result.termination_reason = "도착"
                    self.is_goal_sent = False
                elif nav2_result == TaskResult.CANCELED:
                    # Underlying nav task was canceled. This is not from our client. Abort.
                    result.success = False
                    
                    
                else: # FAILED
                    goal_handle.abort()
                    result.success = False
                    result.termination_reason = "실패"
                    self.is_goal_sent = False
                
                
                
                return result

            # Publish feedback while navigating
            feedback = self.nav2.getFeedback()
            if feedback and self.amcl_pose:
                feedback_msg.current_location = self.convert_to_pose2d(self.convert_to_pose_stamped(self.amcl_pose))
                feedback_msg.distance_remaining = feedback.distance_remaining
                feedback_msg.status = "길안내중"
                feedback_msg.person_detected = not self.is_pause
                goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.2)
        
        # This part is reached if rclpy is shut down or goal is canceled during pause
        goal_handle.abort()
        self.destroy_cap()
        self.is_goal_sent = False
        return GuideNavigationAction.Result(success=False, termination_reason="Aborted")
    
    def amcl_callback(self, msg):
        self.amcl_pose = msg

        
    def goal_callback(self, goal_request: GuideNavigationAction.Goal):
        self.get_logger().info(f"이동 명령 요청 {goal_request.destination}, 최대속도: {goal_request.max_speed}, 안전최소거리: {goal_request.person_follow_distance}")
        self.is_goal_sent = True
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info("이동 명령 취소")
        return CancelResponse.ACCEPT

    
    def init_pose_with_covariance(self):
        init_pose_cov = PoseWithCovarianceStamped()
        init_pose_cov.header.frame_id = 'map'
        init_pose_cov.header.stamp = self.nav2.get_clock().now().to_msg()
        return init_pose_cov

    def init_pose(self):
        init_pose_msg = self.convert_pose_with_covariance_stamped_to_pose_stamped(self.amcl_pose)
        init_pose_msg.header.stamp = self.nav2.get_clock().now().to_msg()
        self.get_logger().info(f'초기 위치 : {init_pose_msg}')
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
    
    def convert_pose_with_covariance_stamped_to_pose_stamped(
            self,
            pose_with_cov_stamped: PoseWithCovarianceStamped
            ) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header = pose_with_cov_stamped.header
        pose_stamped.pose = pose_with_cov_stamped.pose.pose
        return pose_stamped

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

    def end_initial_pause(self):
        pass

    def img_timer_callback(self):
        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if frame is None:
            self.cap.release()
            self.cap = cv2.VideoCapture(0)
            ret, frame = self.cap.read()

        tracks = self.tracker.update(frame)
        track = self.tracker.get_primary_target(tracks=tracks)
        
        # Draw bounding boxes for all tracks
        for t in tracks :
            x1, y1, x2, y2, track_id = map(int, t[:5])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{track_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Pause/resume logic
        person_detected = track is not None
        self.get_logger().info(f'person_detected: {person_detected}')
        if self.is_goal_sent: # Only do this if we are navigating
            if person_detected:
                if self.is_pause:
                    self.get_logger().info("피안내자 감지됨, 길안내를 재개합니다.")
                    self.nav2.goToPose(self.goal_pose)
                    self.is_pause = False
            else: # person not detected
                if not self.is_pause:
                    self.get_logger().info("피안내자 없음, 길안내를 일시중지합니다.")
                    self.nav2.cancelTask()
                    self.is_pause = True

        if cv2.waitKey(1) & 0xFF == 27 :
            self.destroy_node()
            self.destroy_cap()

        self.current_frame = frame
    def img_pub_timer_callback(self):
        if self.current_frame is not None:
            ros_image_message = self.br.cv2_to_imgmsg(self.current_frame, encoding="bgr8")
            
            self._img_pub.publish(ros_image_message)

    def destroy_cap(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info('cap destroyed')

        

def main(args=None):
    rclpy.init(args=args)
    node = GuideNavigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('guide_navigation 종료')
    finally:
        node.destroy_node()
        node.destroy_cap()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
