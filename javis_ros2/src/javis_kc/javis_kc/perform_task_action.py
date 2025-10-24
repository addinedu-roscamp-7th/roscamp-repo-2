import rclpy
from rclpy.action import ActionServer
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
from enum import Enum  # 상태 관리를 위해 Enum 추가

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
    # 필요에 따라 PLACE 관련 상태 추가 가능
    RETURNING_HOME = 6


class PerformTaskActionServer(Node):
    def __init__(self):
        super().__init__('perform_task_action_server')
        tf_cmd = [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0.4", "-0.245", "0.07", "-1.5707", "3.1415", "1.5707",
            "base_link", "camera_link"
        ]
        subprocess.Popen(tf_cmd)

        self.get_logger().info('myCobot 동적 TF 발행')

        self._action_server = ActionServer(
            self,
            PerformTask,
            '/kreacher/action/perform_task',
            self.execute_callback
        )

        self.feedback_msg = PerformTask.Feedback()

        self.mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
        self.mc.thread_lock = False
        self.mc.send_angles([0, 0, 0, -90, 0, 45], 50)
        self.get_logger().info('MyCobot 연결 완료')
        # TF 리스너 및 버퍼 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF 브로드캐스터 초기화 (엔드 이펙터 TF 발행용)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.broadcast_timer_callback)

        # --- 상태 관리 변수 추가 ---
        self.state = RobotState.WAITING_FOR_OBJECT
        self.target_coords = [0.0, 0.0, 0.0]  # 목표 좌표 저장 변수
        self.approach_height = 225  # 접근 높이 (mm)
        self.pick_height_offset = 10  # 물체 높이 + 추가 오프셋 (mm)

        self.coordinate_received = False
        self.lock = threading.Lock()

        # 상태 머신을 실행할 메인 제어 루프 타이머
        self.get_logger().info('액션서버 시작')

    def execute_callback(self, goal_handle):
        move_robot_arm_client = MoveRobotArmClient()

        self.feedback_msg.progress_percentage = 0.0
        while RobotState.RETURNING_HOME != 6:
            self.get_logger().info('로봇이 이동중입니다. 잠시만 기다려주세요')
            res = move_robot_arm_client.send_request()
            x, y, z = res.x, res.y, res.z
            self.get_logger().info(f'x: {x}, y: {y}, z: {z}')
            if x != 0.0 and y != 0.0 and z != 0.0:
                self.target_coords = [x, y, z]
                if self.state == RobotState.WAITING_FOR_OBJECT:
                    self.state = RobotState.LOWERING_TO_PICK
            goal_handle.publish_feedback(self.feedback_msg)
            self.control_loop()
            time.sleep(2)

        # 작업 완료
        goal_handle.succeed()

        result = PerformTask.Result()
        result.success = True
        result.pick_up_num = 1
        result.message = f'Task "{goal_handle.request.member_id}" completed successfully'
        self.get_logger().info('Goal succeeded!')
        return result

    def rpy_to_quaternion(self, roll, pitch, yaw):
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        q = [0.0] * 4
        q[3] = cr * cp * cy + sr * sp * sy  # w
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        return q

    def quaternion_to_rotation_matrix(self, q):
        """쿼터니언을 3x3 회전 행렬로 변환"""
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
        ])

    def transform_point(self, point, transform):
        """TransformStamped를 사용해 점을 수동으로 변환"""
        # 회전 행렬 생성
        R = self.quaternion_to_rotation_matrix(transform.transform.rotation)

        # 이동 벡터
        t = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # 원본 점
        p = np.array([point.x, point.y, point.z])

        # 변환: p' = R * p + t
        transformed = R @ p + t

        return transformed

    def control_loop(self):
        """상태에 따라 로봇을 제어하는 메인 루프"""
        if self.state == RobotState.LOWERING_TO_PICK:
            self.get_logger().info("상태: [LOWERING_TO_PICK]")
            self.feedback_msg.progress_percentage = 30.0
            current_coords = self.mc.get_coords()
            if not current_coords: return

            # z축만 목표 높이로 하강
            pick_coords = [
                self.target_coords[0],
                self.target_coords[1],
                self.target_coords[2] + 100,  # 물체 높이 + 오프셋
                current_coords[3],
                current_coords[4],
                current_coords[5]
            ]
            self.get_logger().info(pick_coords)
            result = self.mc.sync_send_coords(pick_coords, 30, 0)
            time.sleep(2)
            if result == 1:
                self.state = RobotState.GRIPPING

        elif self.state == RobotState.GRIPPING:
            self.get_logger().info("상태: [GRIPPING]")
            self.feedback_msg.progress_percentage = 60.0
            self.mc.set_gripper_value(0, 50)
            time.sleep(1)
            self.state = RobotState.RAISING_AFTER_PICK

        elif self.state == RobotState.RAISING_AFTER_PICK:
            self.get_logger().info("상태: [RAISING_AFTER_PICK]")
            self.feedback_msg.progress_percentage = 80.0
            current_coords = self.mc.get_coords()
            if not current_coords: return

            # z축만 다시 접근 높이로 상승
            raise_coords = [
                current_coords[0],
                current_coords[1],
                self.approach_height,
                current_coords[3],
                current_coords[4],
                current_coords[5]
            ]
            self.mc.send_coords(raise_coords, 20, 0)
            time.sleep(2)
            self.state = RobotState.RETURNING_HOME

        elif self.state == RobotState.RETURNING_HOME:
            self.get_logger().info("상태: [RETURNING_HOME]")
            self.feedback_msg.progress_percentage = 100.0
            self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
            self.mc.set_gripper_value(100, 50)
            time.sleep(2)

    def broadcast_timer_callback(self):
        coords = self.mc.get_coords()
        if not coords: return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'end_effector_link'
        t.transform.translation.x = coords[0] / 1000.0
        t.transform.translation.y = coords[1] / 1000.0
        t.transform.translation.z = coords[2] / 1000.0
        q = self.rpy_to_quaternion(coords[3], coords[4], coords[5])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q[0], \
        q[1], q[2], q[3]
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
