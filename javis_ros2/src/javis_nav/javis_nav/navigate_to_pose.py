#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf_transformations
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Point


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ArucoDockingPID(Node):
    def __init__(self):
        super().__init__('aruco_docking_pid')

        # ---------------------------
        # 🎯 제어 목표값 설정
        # ---------------------------
        self.target_x = 0.0       # 마커 중심 (좌우 정렬)
        self.target_z = 0.65       # 목표 거리
        self.stop_tolerance_z = 0.02  # 거리 오차 허용
        self.stop_tolerance_x = 0.12  # 좌우 오차 허용

        # ---------------------------
        # ⚙️ PID 파라미터
        # ---------------------------
        # 거리 제어 (linear.x)
        self.kp_z = 0.20
        self.ki_z = 0.0
        self.kd_z = 0.05

        # 회전 제어 (angular.z)
        self.kp_x = 1.2
        self.ki_x = 0.0
        self.kd_x = 0.9

        # ---------------------------
        # ⚙️ 내부 상태
        # ---------------------------
        self.integral_z = 0.0
        self.last_error_z = 0.0
        self.integral_x = 0.0
        self.last_error_x = 0.0
        self.max_linear_speed = 0.10
        self.max_angular_speed = 0.9
        self.reached_target = False

        # ---------------------------
        # 🔌 ROS 통신 설정
        # ---------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Point, '/ai/docking/normalized_data', self.callback, 10)

        self.get_logger().info("✅ Aruco Docking PID 활성화 완료 (목표 거리=0.4, 중앙정렬 x=0.0)")

    def callback(self, msg: Point):
        if self.reached_target:
            return

        # 수신된 normalized 데이터
        x = msg.x  # -1 ~ 1 (왼쪽/오른쪽)
        y = msg.y  # pitch (필요시 사용)
        z = msg.z  # 정규화 거리 (0~1)

        # ---------------------------
        # 1️⃣ 거리 제어 (Z축 → linear.x)
        # ---------------------------
        error_z = self.target_z - z
        self.integral_z += error_z
        derivative_z = error_z - self.last_error_z
        control_z = self.kp_z * error_z + self.ki_z * self.integral_z + self.kd_z * derivative_z
        self.last_error_z = error_z
        # 속도 제한
        control_z = max(min(control_z, self.max_linear_speed), -self.max_linear_speed)

        # ---------------------------
        # 2️⃣ 방향 제어 (X축 → angular.z)
        # ---------------------------
        error_x = x - self.target_x
        self.integral_x += error_x
        derivative_x = error_x - self.last_error_x
        control_x = self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * derivative_x
        self.last_error_x = error_x
        # 회전 방향 반전 (로봇이 오른쪽을 +로 보는 경우)
        control_x = -control_x
        control_x = max(min(control_x, self.max_angular_speed), -self.max_angular_speed)


        # ---------------------------
        # x축 오차가 허용 범위 안이면 회전 멈추고 직진만
        # ---------------------------
        if abs(error_x) < self.stop_tolerance_x:
            control_x = 0.0
            self.get_logger().info(f"➡ x축 정렬 완료, 직진만 수행: error_x={error_x:.3f}")

        # ---------------------------
        # 3️⃣ 정지 조건
        # ---------------------------
        if abs(error_z) < self.stop_tolerance_z and abs(error_x) < self.stop_tolerance_x:
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.reached_target = True
            self.get_logger().info(
                f"🎯 정렬 및 목표 거리 도달: z={z:.3f}, x={x:.3f} → 정지"
            )
            return

        # ---------------------------
        # 4️⃣ Twist 메시지 생성 및 퍼블리시
        # ---------------------------
        twist = Twist()
        twist.linear.x = control_z
        twist.angular.z = control_x
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"[PID] z={z:.3f} err_z={error_z:.3f} → v={control_z:.3f} | "
            f"x={x:.3f} err_x={error_x:.3f} → w={control_x:.3f}"
        )

    def reset(self):
        self.integral_z = 0.0
        self.last_error_z = 0.0
        self.integral_x = 0.0
        self.last_error_x = 0.0
        self.reached_target = False
        self.get_logger().info("🔄 PID 상태 리셋 완료")



class RobotPose(Node):
    def __init__(self):
        super().__init__('robot_pose_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.get_pose)

    def get_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.orientation = trans.transform.rotation

            # yaw(heading) 계산
            quat = pose.pose.orientation
            (_, _, yaw) = tf_transformations.euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w]
            )

            self.current_pose = pose 
            self.get_logger().info(
                f"현재 위치 -> x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
            )

        except Exception as e:
            self.get_logger().warn(f"Transform lookup 실패: {e}")


# Shelf positions for picking
shelf_positions = {
    "shelf_A": [4.258,-1.236], #4.858 | 4.358, -1.153, 
}

# Shipping destination for picked products
shipping_destinations = {
    "recycling": [4.888, -1.153],
}

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''

def main():
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_item_location = 'shelf_A'
    request_destination = 'recycling'
    ####################

    rclpy.init()
    navigator = BasicNavigator()

    node = RobotPose() 
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        if hasattr(node, 'current_pose'):
            break
    node.get_logger().info("현재 위치를 가져왔습니다.")
    current_pose = node.current_pose
    node.destroy_node()

    yaw = math.radians(-10)  # Facing forward along the x-axis  -13  

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = node.current_pose.pose.position.x
    initial_pose.pose.position.y = node.current_pose.pose.position.y
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 0.1
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = math.sin(yaw / 2)
    shelf_item_pose.pose.orientation.w = math.cos(yaw / 2)
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination + ')...')
        # shipping_destination = PoseStamped()
        # shipping_destination.header.frame_id = 'map'
        # shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        # shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
        # shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
        # shipping_destination.pose.orientation.z = math.sin(yaw / 2)
        # shipping_destination.pose.orientation.w = math.cos(yaw / 2)
        # navigator.goToPose(shipping_destination)
        
        print("PID 제어 시작")
        apc = ArucoDockingPID()       # ✅ 클래스 이름 수정
        navigator.cancelTask()         # 네비게이션 중단

        # ✅ 먼저 정지 명령 전송
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        apc.cmd_pub.publish(stop_twist)
        apc.reset()

        # ✅ ArUco 기반 PID로 0.4m까지 접근
        while rclpy.ok() and not apc.reached_target:
            rclpy.spin_once(apc, timeout_sec=0.1)

        # ✅ 목표 도달 후 완전 정지
        final_twist = Twist()
        final_twist.linear.x = 0.0
        final_twist.angular.z = 0.0
        apc.cmd_pub.publish(final_twist)
        apc.get_logger().info("✅ 0.4m 거리 및 정렬 완료 → 완전 정지.")


    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()