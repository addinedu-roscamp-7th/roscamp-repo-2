import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from javis_interfaces.action import PerformTask
from javis_kc.move_robot_arm_client import MoveRobotArmClient
from pymycobot.mycobot280 import MyCobot280
import time
import math
import threading
import numpy as np
import subprocess
from enum import Enum

# TF2 라이브러리 임포트
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy.duration

class RobotState(Enum):
    WAITING_FOR_OBJECT = 1
    LOWERING_TO_PICK = 3
    GRIPPING = 4
    RAISING_AFTER_PICK = 5
    RETURNING_HOME = 6

class Point:
    def __init__(self, x, y, z): self.x, self.y, self.z = x, y, z

class PerformTaskActionServer(Node):
    def __init__(self):
        super().__init__('perform_task_action_server')
        
        # 'busy' 상태를 관리하기 위한 플래그
        self._is_busy = False

        tf_cmd = [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0.375", "0.505", "0.275", "0.0", "3.1415", "1.5707",
            "base_link", "camera_link"
        ]
        subprocess.Popen(tf_cmd)

        self.get_logger().info('myCobot 동적 TF 발행')

        self._action_server = ActionServer(
            self,
            PerformTask,
            '/kreacher/action/perform_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback  # 목표 수락/거절을 위한 콜백 추가
        )

        self.feedback_msg = PerformTask.Feedback()

        self.mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
        self.mc.thread_lock = False
        self.mc.send_angles([0.0,0.0,0.0,-90.0,0.0,-45.0], 30)
        self.mc.set_gripper_value(100, 50)
        self.get_logger().info('MyCobot 연결 완료')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.state = RobotState.WAITING_FOR_OBJECT
        self.target_coords = [0.0, 0.0, 0.0]
        self.approach_height = 225
        
        self.get_logger().info('액션서버 시작. 목표를 기다립니다...')

    def goal_callback(self, goal_request):
        """새로운 목표가 들어왔을 때 수락할지 거절할지 결정합니다."""
        if self._is_busy:
            self.get_logger().info('서버가 이미 다른 작업을 수행 중이므로 새 목표를 거절합니다.')
            return GoalResponse.REJECT
        
        self.get_logger().info('새로운 목표를 수락합니다.')
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """목표를 수락했을 때 실제 작업을 수행하는 함수."""
        self.get_logger().info(f'작업 실행: "{goal_handle.request.member_id}"')
        
        # 작업 시작을 알리고 busy 플래그 설정
        self._is_busy = True
        
        # 작업이 실패하거나 성공하더라도 항상 busy 플래그를 해제하도록 try-finally 사용
        try:
            # 1. 클라이언트를 통해 목표 좌표를 '한 번만' 요청합니다.
            move_robot_arm_client = MoveRobotArmClient()
            self.get_logger().info('목표 객체의 좌표를 요청합니다...')
            res = move_robot_arm_client.send_request()

            # 2. 유효한 좌표인지 확인 (0,0,0이 아닌지)
            if res.x == 0.0 and res.y == 0.0 and res.z == 0.0:
                error_msg = '유효하지 않은 좌표(0,0,0)를 수신하여 작업을 중단합니다.'
                self.get_logger().error(error_msg)
                goal_handle.abort()
                return PerformTask.Result(success=False, message=error_msg)

            # 3. TF 변환을 수행합니다.
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'camera_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )
                transform_xyz = self.transform_point(Point(res.x, res.y, res.z), transform)
                self.get_logger().info(f'변환된 좌표: x: {transform_xyz[0]}, y: {transform_xyz[1]}, z: {transform_xyz[2]}')
                self.target_coords = [transform_xyz[0], transform_xyz[1], transform_xyz[2]]

                self.mc.send_angles([-0.08, 2.81, -102.12, 6.76, -0.96, -40.16], 30)
                time.sleep(2.0)
            except tf2_ros.TransformException as ex:
                error_msg = f'TF 변환 실패: {ex}'
                self.get_logger().error(error_msg)
                goal_handle.abort()
                return PerformTask.Result(success=False, message=error_msg)

            # 4. 상태 머신을 시작하고 작업 완료까지 순차적으로 실행
            self.state = RobotState.LOWERING_TO_PICK
            
            # 상태가 RETURNING_HOME이 될 때까지 control_loop를 주기적으로 실행
            while self.state != RobotState.RETURNING_HOME and rclpy.ok():
                self.control_loop()
                self.feedback_msg.progress_percentage = self.get_progress_by_state()
                goal_handle.publish_feedback(self.feedback_msg)
                time.sleep(1) # 각 상태 전환 후 안정화를 위한 대기

            # 마지막 상태(RETURNING_HOME) 실행
            if self.state == RobotState.RETURNING_HOME and rclpy.ok():
                 self.get_logger().info("마지막 단계: 홈으로 복귀 중...")
                 self.control_loop()

            # 작업 완료 처리
            goal_handle.succeed()
            result = PerformTask.Result()
            result.success = True
            result.pick_up_num = 1
            result.message = f'Task "{goal_handle.request.member_id}" completed successfully'
            self.get_logger().info('목표 수행 성공!')
            return result

        finally:
            # 작업이 성공하든 실패하든, 마지막에는 항상 busy 플래그를 해제하고 초기 상태로 복귀
            self.get_logger().info("작업 완료. 다음 목표를 받을 준비가 되었습니다.")
            self._is_busy = False
            self.state = RobotState.WAITING_FOR_OBJECT

    def get_progress_by_state(self):
        """현재 상태에 따라 진행률을 반환합니다."""
        if self.state == RobotState.LOWERING_TO_PICK: return 30.0
        if self.state == RobotState.GRIPPING: return 60.0
        if self.state == RobotState.RAISING_AFTER_PICK: return 80.0
        if self.state == RobotState.RETURNING_HOME: return 95.0
        return 0.0

    def control_loop(self):
        """상태에 따라 로봇을 제어하는 메인 루프 (기존 코드와 유사, 상태 전환 로직 강화)"""
        if self.state == RobotState.LOWERING_TO_PICK:
            self.get_logger().info("상태: [LOWERING_TO_PICK]")
            current_coords = self.mc.get_coords()
            if not current_coords: return

            pick_coords = [
                float(self.target_coords[0] * 1000 - 70), float(self.target_coords[1] * 1000 -15),
                current_coords[2],
                current_coords[3], current_coords[4], current_coords[5]
            ]
            self.mc.sync_send_coords(pick_coords, 40, 1)  # 속도, 타임아웃
            time.sleep(3)
            current_coords = self.mc.get_coords()
            pick_coords = [
                current_coords[0], current_coords[1],
                float(self.target_coords[2] * 1000 + 30), # Z값 오프셋
                current_coords[3], current_coords[4], current_coords[5]
            ]
            self.mc.sync_send_coords(pick_coords, 40, 1) # 속도, 타임아웃
            time.sleep(1.5)
            self.state = RobotState.GRIPPING

        elif self.state == RobotState.GRIPPING:
            self.get_logger().info("상태: [GRIPPING]")
            self.mc.set_gripper_value(0, 50)
            time.sleep(1.5)

            self.state = RobotState.RAISING_AFTER_PICK

        elif self.state == RobotState.RAISING_AFTER_PICK:
            self.get_logger().info("상태: [RAISING_AFTER_PICK]")
            current_coords = self.mc.get_coords()
            if not current_coords: return

            raise_angles = [-1.14, 10.89, -63.63, -40.25, -2.02, -45.87]
            self.mc.send_angles(raise_angles, 30)
            time.sleep(1.5)

            reverse_cup = [-1.14, 10.89, -63.63, -40.25, 178.0, -45.87]
            self.mc.send_angles(reverse_cup, 30)
            self.state = RobotState.RETURNING_HOME

        elif self.state == RobotState.RETURNING_HOME:
            self.get_logger().info("상태: [RETURNING_HOME]")
            self.mc.send_angles([0.0,0.0,0.0,-90.0, 0.0,-45.0], 30)
            time.sleep(2.5)
            # 이 상태는 루프를 종료시키는 역할이므로, 여기서 상태를 바꾸지 않음

    # --- 나머지 유틸리티 함수들 (변경 없음) ---
    def rpy_to_quaternion(self, roll, pitch, yaw):
        roll_rad, pitch_rad, yaw_rad = map(math.radians, [roll, pitch, yaw])
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z
            cr * cp * cy + sr * sp * sy   # w
        ]

    def quaternion_to_rotation_matrix(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
        ])

    def transform_point(self, point, transform):
        R = self.quaternion_to_rotation_matrix(transform.transform.rotation)
        t = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        p = np.array([point.x, point.y, point.z])
        return R @ p + t

    def broadcast_timer_callback(self):
        coords = self.mc.get_coords()
        if not coords: return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'end_effector_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = [c / 1000.0 for c in coords[:3]]
        q = self.rpy_to_quaternion(coords[3], coords[4], coords[5])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q[0], q[1], q[2], q[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    action_server = PerformTaskActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()