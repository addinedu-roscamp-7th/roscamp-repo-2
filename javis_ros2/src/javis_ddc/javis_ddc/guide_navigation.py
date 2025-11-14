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

REID_PATH = "osnet_x0_25_msmt17.pt"

class GuideNavigation(Node):
    def __init__(self):
        super().__init__('guide_navigation')
        self.goal_pose = PoseStamped()
        self.nav2 = BasicNavigator()

        self.amcl_pose = None
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
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
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.tracker = ReIDTracker(reid_model_path=REID_PATH)

        self.br = CvBridge()
        self.current_frame = None
        self.img_timer_period = 1.0 /30.0
        

        self.dobby_nav2_server = ActionServer(self, GuideNavigationAction,
                                              'dobby1/drive/guide_navigation',
                                              execute_callback=self.execute_callback,
                                              goal_callback=self.goal_callback,
                                              cancel_callback=self.cancel_callback)
        self.timer_period = 1.0/30.0
        self.img_pub_timer = self.create_timer(self.timer_period, self.img_pub_timer_callback)
        

    def execute_callback(self, goal_handle):
        self.get_logger().info("이동 명령 실행")
        request = goal_handle.request

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("WebCam을 열 수 없습니다.")
            self.destroy_node()
            return

        self._img_timer = self.create_timer(self.img_timer_period, self.img_timer_callback)
        
        goal_pose = self.set_goal_pose(request)
        self.nav2.goToPose(goal_pose)

        feedback_msg = GuideNavigationAction.Feedback()
        
        while not self.nav2.isTaskComplete():
            if goal_handle.is_cancel_requested:
                self.nav2.cancelTask()
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return GuideNavigationAction.Result()

            feedback = self.nav2.getFeedback()
            if feedback and self.amcl_pose:
                current_pose = self.convert_to_pose2d(self.convert_to_pose_stamped(self.amcl_pose))
                feedback_msg.current_location = current_pose
                feedback_msg.distance_remaining = feedback.distance_remaining
                feedback_msg.status = "길안내중"
                feedback_msg.person_detected = True
                goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.2)

        result = GuideNavigationAction.Result()
        nav2_result = self.nav2.getResult()
        self.get_logger().info(f'nav2_result : {nav2_result}')

        if self.amcl_pose:
            current_pose = self.convert_to_pose2d(self.convert_to_pose_stamped(self.amcl_pose))
            result.final_location = current_pose

        if nav2_result == TaskResult.SUCCEEDED:
            goal_handle.succeed()
            result.success = True
            result.termination_reason = "도착"
            self.destroy_cap()
        elif nav2_result == TaskResult.CANCELED:
            goal_handle.canceled()
            result.success = False
            result.termination_reason = "취소됨"
            self.destroy_cap()
        else:
            goal_handle.abort()
            result.success = False
            result.termination_reason = "실패"
            self.destroy_cap()

        
        
        return result
    
    def amcl_callback(self, msg):
        self.amcl_pose = msg

        
    def goal_callback(self, goal_request: GuideNavigationAction.Goal):
        self.get_logger().info(f"이동 명령 요청 {goal_request.destination}, 최대속도: {goal_request.max_speed}, 안전최소거리: {goal_request.person_follow_distance}")
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
    
    def img_timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('ret is none')
            return

        tracks = self.tracker.update(frame)

        self.person_detected = tracks.shape[0] > 0

        # primary_target = self.tracker.get_primary_target(tracks)

        # self.get_logger().info(f'{tracks}')
        
        # for track in tracks :
        #     x1, y1, x2, y2, track_id = track[:5]
        #     x1, y1, x2, y2, track_id = map(int, [x1, y1, x2, y2, track_id])
        #     cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #     cv2.putText(frame, f"{track_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if self.person_detected:
            track = self.tracker.get_primary_target(tracks=tracks)
            x1, y1, x2, y2, track_id = track[:5]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{track_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0

            self._cmd_vel_pub.publish(stop_msg)
            self.get_logger().info("일시정지 중")
        if cv2.waitKey(1) & 0xFF == 27 :
            self.destroy_node()
        self.current_frame = frame

    def img_pub_timer_callback(self):
        if self.current_frame is not None:
            ros_image_message = self.br.cv2_to_imgmsg(self.current_frame, encoding="bgr8")
            self._img_pub.publish(ros_image_message)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

    def destroy_cap(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

        

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
