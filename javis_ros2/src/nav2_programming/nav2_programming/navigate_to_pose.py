#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf_transformations
from sensor_msgs.msg import LaserScan
import math
# from javis_interfaces.msg import ArucoDockingData


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# class LidarPIDController(Node):
#     def __init__(self):
#         super().__init__('aruco_pid_controller')

#         # 목표 거리 (10cm)
#         self.target_distance = 0.10  # m
#         self.kp = 0.20
#         self.ki = 0.0
#         self.kd = 0.10

#         self.last_error = 0.0
#         self.integral = 0.0
#         self.max_speed = 0.1  # m/s
#         self.stop_tolerance = 0.02  # ±2 cm

#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.create_subscription(String, '/aruco_pid_controller', self.aruco_callback, 10)

#         self.reached_target = False
#         self.get_logger().info("Aruco PID Controller (10 cm) 활성화 완료")

#     def aruco_callback(self, msg):
#         if self.reached_target:
#             return

#         text = msg.data.strip()
#         try:
#             # 예: "DDC (ID: 11) | x: -0.038, z: 0.303, pitch: -8.846"
#             parts = text.split('|')
#             x_str = parts[1].split(':')[1].split(',')[0].strip()
#             z_str = parts[2].split(':')[1].split(',')[0].strip()

#             x = float(x_str)
#             z = float(z_str)
#         except Exception as e:
#             self.get_logger().warn(f"파싱 실패: {e} | 데이터: {text}")
#             return

#         dist = z
#         error = dist - self.target_distance

#         # 정지 조건
#         if abs(error) <= self.stop_tolerance:
#             twist = Twist()
#             twist.linear.x = 0.0
#             self.cmd_pub.publish(twist)
#             self.reached_target = True
#             self.get_logger().info(f" 목표 거리 도달 ({dist:.3f} m) → 정지")
#             return

#         #PID 계산
#         self.integral += error
#         derivative = error - self.last_error
#         control = self.kp * error + self.ki * self.integral + self.kd * derivative
#         self.last_error = error

#         control = max(min(control, self.max_speed), -self.max_speed)

#         twist = Twist()
#         twist.linear.x = control
#         self.cmd_pub.publish(twist)

#         self.get_logger().info(f" 거리={dist:.3f} m | 제어속도={control:.2f}")

#     def reset(self):
#         self.last_error = 0.0
#         self.integral = 0.0
#         self.reached_target = False



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
    "shelf_A": [4.358,-1.053], #4.858 , -1.153
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
        # print("PID 제어 시작")
        # apc = ArucoPIDController()      # 기존 LidarPIDController() → ArucoPIDController()
        # navigator.cancelTask()          # 네비게이션 중단

        # # 먼저 정지 명령 전송
        # stop_twist = Twist()
        # stop_twist.linear.x = 0.0
        # apc.cmd_pub.publish(stop_twist)
        # apc.reset()

        # # ArUco 감지 기반으로 10cm까지 접근
        # while rclpy.ok() and not apc.reached_target:
        #     rclpy.spin_once(apc, timeout_sec=0.1)

        # # 목표 도달 후 완전 정지
        # final_twist = Twist()
        # final_twist.linear.x = 0.0
        # apc.cmd_pub.publish(final_twist)
        # apc.get_logger().info("✅ 10cm 거리 도달 후 완전 정지 완료.")

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