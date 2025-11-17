import rclpy
from rclpy.action import ActionServer, GoalResponse, GoalStatus
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
from javis_interfaces.action import PerformTask
from javis_kc.move_robot_arm_client import MoveRobotArmClient
from pymycobot.mycobot280 import MyCobot280
import time
import math
import numpy as np
import subprocess
from enum import Enum
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy.duration
from typing import List, Tuple

# 로봇 기본 자세
HOME_ANGLES = [0.0, 0.0, 0.0, -90.0, 0.0, -45.0]
PICKUP_HOME_ANGLES = [1.23, 4.13, 0.61, 83.58, -0.43, -43.15]

# 핫아메리카노
M1_RAISE_ANGLES = [-1.14, 10.89, -63.63, -40.25, -2.02, -45.87]
M1_REVERSE_CUP = [-1.14, 10.89, -63.63, 140.0, -2.02, -45.87]
M1_READY_TO_PLACE = [91.14, -81.73, 92.54, 82.44, -0.08, -42.62]
M1_CLOSE_TO_DISPENSE = [89.47, -53.78, 105.46, 58.27, -2.37, -45.08]
M1_FINISH_TO_DISPENSE = [85.07, -93.33, 130.42, 52.99, 1.4, -36.91]
M1_FINISH_TO_DISPENSE_1 = [87.36, -1.31, 6.85, 79.54, 0.96, -39.63]
M1_FINISH_TO_DISPENSE_2 = [-88.41, -0.79, -1.14, 81.91, -4.39, -45.0]
M1_FINISH_TO_DISPENSE_3 = [-92.19, 135.0, -83.84, 45.7, -4.74, -43.41]
M1_FINISH_TO_DISPENSE_4 = [-89.38, 109.07, -95.97, 74.61, -2.81, -45.79]


# 아이스아메리카노
M2_ICE_PREPROCESS_1 = [86.57, -10.54, -4.39, -92.72, 1.31, -52.03]
M2_ICE_PREPROCESS_2 = [92.1, -66.35, -38.32, -92.28, 96.76, -64.59]
M2_READY_TO_PLACE = [47.72, 3.86, -2.81, -87.53, -2.63, 79.89]
M2_CLOSE_TO_DISPENSE = [-21.18, -1.66, -2.81, -86.74, 17.31, 18.72]
M2_FINISH_TO_DISPENSE_1 = [92.63, 5.62, -16.52, -76.81, 0.52, -37.7]
M2_FINISH_TO_DISPENSE_2 = [96.5, -38.14, -59.32, 4.39, -2.54, -65.12]
M2_FINISH_TO_DISPENSE_3 = [93.69, -7.91, -47.46, -35.41, -5.44, -38.32]


GRIPPER_OPEN = 100
GRIPPER_CLOSE = 0
GRIPPER_SPEED = 50

GRIPPER_TIP_OFFSET_IN_EE = np.array([0.06, -0.06, 0.005])


class RobotState(Enum):
    """로봇 동작 상태를 명확히 정의합니다. (주석용)"""
    WAITING_FOR_OBJECT = 0
    APPROACHING = 1
    PICKING = 2
    GRIPPING = 3
    RAISING = 4
    DISPENSING = 5
    RETURNING_HOME = 6

class Point:
    def __init__(self, x, y, z): self.x, self.y, self.z = x, y, z

class PerformTaskActionServer(Node):
    
#진행단계
    PROGRESS_STEPS = {
        RobotState.APPROACHING: 20.0,
        RobotState.PICKING: 40.0,
        RobotState.GRIPPING: 60.0,
        RobotState.RAISING: 70.0,
        RobotState.DISPENSING: 90.0,
        RobotState.RETURNING_HOME: 100.0,
    }

    def __init__(self):
        super().__init__('perform_task_action_server')
        

        self._is_busy = False
        self.object_rotation_angle = 0.0 
        self.state = RobotState.WAITING_FOR_OBJECT
        self.target_coords = [0.0, 0.0, 0.0]
        self.approach_height_m = 0.225
        self.pick_orientation_rpy = [-179.0, 0.0, -45.0] # 기본 RPY [Roll, Pitch, Yaw]
        self.current_menu_id = None
        
        # 액션 서버 받는 곳
        self._action_server = ActionServer(
            self, PerformTask, '/kreacher/action/perform_task',
            execute_callback=self.execute_callback, goal_callback=self.goal_callback
        )
        #opencv이용한 물체 회전값 구하는거
        self.feedback_msg = PerformTask.Feedback()
        self.angle_subscriber = self.create_subscription(
            Float64, 'rotation_angle', self.angle_callback, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.broadcast_timer_callback) # 손수 TF 발행


        self.mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
        self.mc.thread_lock = True
        self._move_to_angles(HOME_ANGLES, 30, log=False)
        self.mc.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
        self.get_logger().info('MyCobot 연결 완료 및 홈 위치 이동.')

        self._start_static_tf_publisher()

        self.get_logger().info('액션서버 시작. 목표를 기다립니다...')

    def _start_static_tf_publisher(self):
        tf_cmd = [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0.31", "0.505", "0.280", "0.0", "3.1415", "1.5707",
            "base_link", "camera_link"
        ]
        subprocess.Popen(tf_cmd)
        self.get_logger().info('정적 TF 발행기(카메라) 시작.')
        
    #각도 움직임
    def _move_to_angles(self, angles: List[float], speed: int = 30, delay: float = 3.0, log: bool = True):
        if log:
            self.get_logger().info(f"로봇 이동: Angles {angles}")
        self.mc.send_angles(angles, speed)
        time.sleep(delay)

    #좌표 보내기
    def _move_to_coords(self, coords: List[float], speed: int = 40, model: int = 1, delay: float = 3.0, log: bool = True):
        if log:
            self.get_logger().info(f"로봇 이동: Coords {coords}")
        self.mc.send_coords(coords, speed, model)
        time.sleep(delay)
    


    def angle_callback(self, msg):
        self.object_rotation_angle = msg.data
        self.get_logger().debug(f"수신된 객체 회전 각도: {self.object_rotation_angle:.2f} degrees")

    def goal_callback(self, goal_request):
        if self._is_busy:
            self.get_logger().info('서버가 이미 다른 작업을 수행 중이므로 새 목표를 거절합니다.')
            return GoalResponse.REJECT
        
        self.get_logger().info('새로운 목표를 수락합니다.')
        return GoalResponse.ACCEPT

    #명령 받는 함수
    def execute_callback(self, goal_handle):
        self.get_logger().info(f'작업 실행 시작 - order_id: {goal_handle.request.order_id}, menu_id: {goal_handle.request.menu_id}')
        
        self._is_busy = True
        self.current_menu_id = goal_handle.request.menu_id
        
        try:
            target_ee_coords, pick_orientation_rpy = self._get_and_transform_target(goal_handle)
            if target_ee_coords is None:
                return PerformTask.Result(success=False, message="좌표 획득/변환 실패")

            self.state = RobotState.APPROACHING
            self._move_to_approach(target_ee_coords, pick_orientation_rpy)
            self._publish_feedback(goal_handle, self.state)

            self.state = RobotState.PICKING
            self._perform_picking(pick_orientation_rpy)
            self._publish_feedback(goal_handle, self.state)
            
            self.state = RobotState.RAISING
            self._publish_feedback(goal_handle, self.state)

            self.state = RobotState.DISPENSING
            if self.current_menu_id == 1:
                self._dispense_menu1()
            elif self.current_menu_id == 2:
                self._dispense_menu2(target_ee_coords, pick_orientation_rpy) # 얼음 픽업을 위한 좌표 전달
            else:
                error_msg = f'지원하지 않는 menu_id: {self.current_menu_id}'
                self.get_logger().error(error_msg)
                goal_handle.abort()
                return PerformTask.Result(success=False, message=error_msg)
            
            self._publish_feedback(goal_handle, self.state)


            self.state = RobotState.RETURNING_HOME
            self._return_home()
            self._publish_feedback(goal_handle, self.state)

            goal_handle.succeed()
            result = PerformTask.Result()
            result.order_id = goal_handle.request.order_id
            result.success = True
            result.pick_up_num = 1 if self.current_menu_id == 1 else 2
            result.message = f'메뉴 {self.current_menu_id}번 음료 제조 완료. 픽업 번호: {result.pick_up_num}'
            return result

        except Exception as e:
            error_msg = f'작업 실행 중 예외 발생: {e}'
            self.get_logger().error(error_msg)
            goal_handle.abort()
            return PerformTask.Result(success=False, message=error_msg)

        finally:
            self.get_logger().info("작업 완료. 다음 목표를 받을 준비가 되었습니다.")
            self._is_busy = False
            self.state = RobotState.WAITING_FOR_OBJECT

    def _get_and_transform_target(self, goal_handle) -> Tuple[List[float], List[float]] | Tuple[None, None]:
        """객체 좌표를 획득하고 base_link 기준 end_effector 좌표로 변환"""
        move_robot_arm_client = MoveRobotArmClient()
        self.get_logger().info('목표 객체의 좌표를 요청합니다...')
        res = move_robot_arm_client.send_request()
        
        if res.x == 0.0 and res.y == 0.0 and res.z == 0.0:
            self.get_logger().error('유효하지 않은 좌표(0,0,0)를 수신했습니다.')
            goal_handle.abort()
            return None, None
        
        # 2. 객체 각도에 따른 Yaw 각도 계산
        calculated_yaw = self.pick_orientation_rpy[2]

        if 0.0 <= self.object_rotation_angle <= 45.0:
            calculated_yaw -= self.object_rotation_angle
        elif 45.0 < self.object_rotation_angle <= 90.0:
            calculated_yaw -= (self.object_rotation_angle - 90.0)
        else:
             self.get_logger().warn(f"객체 각도({self.object_rotation_angle:.2f}도)가 0~90도 범위를 벗어났습니다. 기본 Yaw 각도를 사용합니다.")

        pick_orientation_rpy = [
            self.pick_orientation_rpy[0],  # -179.0
            self.pick_orientation_rpy[1],  # 0.0
            calculated_yaw                  # 계산된 값
        ]
        
        # TF 변환 (camera_link -> base_link)
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            self.target_coords = self.transform_point(Point(res.x, res.y, res.z), transform)
            self.get_logger().info(f'변환된 gripper_tip 목표 좌표(m): {self.target_coords}')

            approach_pos_gripper = [
                self.target_coords[0], 
                self.target_coords[1], 
                self.approach_height_m 
            ]
            
            approach_coords_ee = self._calculate_end_effector_pose(
                approach_pos_gripper,
                pick_orientation_rpy 
            )
            
            self.get_logger().info(f"계산된 end_effector 접근 좌표(mm, deg): {approach_coords_ee}")
            return approach_coords_ee, pick_orientation_rpy
            
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'TF 변환 실패: {ex}')
            goal_handle.abort()
            return None, None

    # 물체 접근
    def _move_to_approach(self, approach_coords_ee: List[float], pick_orientation_rpy: List[float]):
        self.get_logger().info(f"동작: 접근 위치로 이동 (z={approach_coords_ee[2]:.2f} mm)")
        self._move_to_coords(approach_coords_ee, speed=30, delay=3.0)
        self.pick_orientation_rpy = pick_orientation_rpy # 최종 픽업 자세 업데이트

    # 물체 집기 수행
    def _perform_picking(self, pick_orientation_rpy: List[float]):
        self.get_logger().info("동작: 물체 잡기 위해 하강")
        
        # 잡기 직전의 gripper_tip 위치
        final_gripper_tip_pos = [
            self.target_coords[0],
            self.target_coords[1],
            self.target_coords[2] + 0.030  # 30mm 오프셋 
        ]
        
        # 최종 잡기 위치에 도달하기 위한 end_effector 좌표 계산
        final_pick_coords_ee = self._calculate_end_effector_pose(
            final_gripper_tip_pos,
            pick_orientation_rpy
        )
        
        # 계산된 좌표로 이동
        self.get_logger().info(f"동작: 최종 잡기 위치로 이동 (z={final_pick_coords_ee[2]:.2f} mm)")
        self._move_to_coords(final_pick_coords_ee, speed=40, delay=1.5)
        
        self.state = RobotState.GRIPPING
        self.get_logger().info("동작: 그리퍼 닫기 (GRIPPING)")
        self.mc.set_gripper_value(GRIPPER_CLOSE, GRIPPER_SPEED)
        time.sleep(1.5)

    def _dispense_menu1(self):
        self.get_logger().info("동작: Menu 1 - 디스펜스 시퀀스 시작")
        
        self._move_to_angles(M1_RAISE_ANGLES, 30, 5.0)
        self._move_to_angles(M1_REVERSE_CUP, 30, 5.0)
        self._move_to_angles(M1_READY_TO_PLACE, 30, 5.5)
        self._move_to_angles(M1_CLOSE_TO_DISPENSE, 30, 9.5)
        self._move_to_angles(M1_FINISH_TO_DISPENSE, 30, 5.0)
        self._move_to_angles(M1_FINISH_TO_DISPENSE_1, 30, 5.0)
        self._move_to_angles(M1_FINISH_TO_DISPENSE_2, 30, 5.0)
        self._move_to_angles(M1_FINISH_TO_DISPENSE_3, 20, 5.0)
        
        self.get_logger().info("동작: 그리퍼 열기")
        self.mc.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
        time.sleep(5.0)
        
        self._move_to_angles(M1_FINISH_TO_DISPENSE_4, 30, 3.0)
    
    def _dispense_menu2(self, target_ee_coords: List[float], pick_orientation_rpy: List[float]):
        self.get_logger().info("동작: Menu 2 - 얼음 픽업 및 디스펜스 시퀀스 시작")
        
        # 얼음 픽업 및 배치
        self.get_logger().info("동작: 얼음 픽업 및 컵에 배치")
        self._move_to_angles(M2_ICE_PREPROCESS_1, 30, 2.0)
        self._move_to_angles(M2_ICE_PREPROCESS_2, 30, 2.0)
        self.mc.set_gripper_value(GRIPPER_CLOSE, GRIPPER_SPEED) # 얼음 집기
        time.sleep(1.5)
        self._move_to_angles(M2_ICE_PREPROCESS_1, 30, 1.5)
        
        # 컵 위 접근 위치로 이동
        cup_approach_coords_ice_drop = [
            target_ee_coords[0],
            target_ee_coords[1],
            self.approach_height_m * 1000.0,
            target_ee_coords[3],
            target_ee_coords[4],
            target_ee_coords[5]
        ]
        self._move_to_coords(cup_approach_coords_ice_drop, speed=40, delay=1.5)
        self.mc.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED) # 얼음 놓기
        time.sleep(1.5)
        
        # 컵 픽업 및 디스펜스
        self.get_logger().info("동작: 컵 픽업 및 디스펜스")
        
        # 물체를 들어올리는 동작
        self._move_to_angles(M1_RAISE_ANGLES, 30, 5.0)
        
        # 두 번째 디스펜스 위치로 이동
        self._move_to_angles(M2_READY_TO_PLACE, 30, 5.5)
        self._move_to_angles(M2_CLOSE_TO_DISPENSE, 30, 9.5)
        self._move_to_angles(M2_READY_TO_PLACE, 30, 5.0) # 복귀
        self._move_to_angles(M2_FINISH_TO_DISPENSE_1, 30, 5.0)
        self._move_to_angles(M2_FINISH_TO_DISPENSE_2, 30, 5.0)
        
        self.get_logger().info("동작: 그리퍼 열기")
        self.mc.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
        time.sleep(5.0)
        
        self._move_to_angles(M2_FINISH_TO_DISPENSE_3, 30, 3.0)

    def _return_home(self):
        self.get_logger().info("동작: 홈으로 복귀")
        self._move_to_angles(PICKUP_HOME_ANGLES, 30, 2.5)
        self._move_to_angles(HOME_ANGLES, 30, 2.0)

    def _publish_feedback(self, goal_handle, state: RobotState):
        self.feedback_msg.order_id = goal_handle.request.order_id
        self.feedback_msg.progress_percentage = self.PROGRESS_STEPS.get(state, 0.0)
        goal_handle.publish_feedback(self.feedback_msg)


    def _rpy_deg_to_rotation_matrix(self, rpy_deg):
        return self._rpy_rad_to_rotation_matrix(list(map(math.radians, rpy_deg)))

    def _rpy_rad_to_rotation_matrix(self, rpy_rad):
        roll, pitch, yaw = rpy_rad[0], rpy_rad[1], rpy_rad[2]
        Rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def _calculate_end_effector_pose(self, gripper_tip_target_pos_m: List[float], end_effector_target_rpy_deg: List[float]) -> List[float]:
        # 목표 end_effector 자세에 대한 회전 행렬 계산
        R_base_to_ee = self._rpy_deg_to_rotation_matrix(end_effector_target_rpy_deg)
        
        # 오프셋 벡터를 base_link 좌표계로 변환
        offset_in_base_frame = R_base_to_ee @ GRIPPER_TIP_OFFSET_IN_EE
        
        # gripper_tip 목표 위치에서 변환된 오프셋을 빼서 end_effector 목표 위치 계산
        P_gripper_tip = np.array(gripper_tip_target_pos_m)
        P_end_effector = P_gripper_tip - offset_in_base_frame
        
        # myCobot의 send_coords 형식(mm, deg)으로 변환하여 반환
        return [
            P_end_effector[0] * 1000.0 + 10.0, # 보정값
            P_end_effector[1] * 1000.0 - 40.0, # 보정값
            P_end_effector[2] * 1000.0,
            float(end_effector_target_rpy_deg[0]),
            float(end_effector_target_rpy_deg[1]),
            float(end_effector_target_rpy_deg[2])
        ]

    def rpy_to_quaternion(self, roll, pitch, yaw):
        roll_rad, pitch_rad, yaw_rad = map(math.radians, [roll, pitch, yaw])
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]
    
    def rotation_matrix_to_quaternion(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return [qx, qy, qz, qw]


    def quaternion_to_rotation_matrix(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
        ])

    def transform_point(self, point, transform):
        R = self.quaternion_to_rotation_matrix(transform.transform.rotation)
        t = np.array([
            transform.transform.translation.x, 
            transform.transform.translation.y, 
            transform.transform.translation.z
        ])
        p = np.array([point.x, point.y, point.z])
        return R @ p + t

    def broadcast_timer_callback(self, *args, **kwargs):
        coords = self.mc.get_coords()
        if not coords: return

        current_time = self.get_clock().now().to_msg()

        t_ee = TransformStamped()
        t_ee.header.stamp = current_time
        t_ee.header.frame_id = 'base_link'
        t_ee.child_frame_id = 'end_effector_link'
        t_ee.transform.translation.x = coords[0] / 1000.0
        t_ee.transform.translation.y = coords[1] / 1000.0
        t_ee.transform.translation.z = coords[2] / 1000.0
        q_ee = self.rpy_to_quaternion(coords[3], coords[4], coords[5])
        t_ee.transform.rotation.x, t_ee.transform.rotation.y, t_ee.transform.rotation.z, t_ee.transform.rotation.w = q_ee
        self.tf_broadcaster.sendTransform(t_ee)

        t_gripper_in_ee = GRIPPER_TIP_OFFSET_IN_EE # x, y, z (m)
        rpy_gripper_in_ee_rad = [0.0, 0.0, -0.7854]
        R_gripper_in_ee = self._rpy_rad_to_rotation_matrix(rpy_gripper_in_ee_rad)

        t_ee_in_base = np.array([t_ee.transform.translation.x, t_ee.transform.translation.y, t_ee.transform.translation.z])
        R_ee_in_base = self._rpy_deg_to_rotation_matrix([coords[3], coords[4], coords[5]])

        t_gripper_in_base = t_ee_in_base + R_ee_in_base @ t_gripper_in_ee
        R_gripper_in_base = R_ee_in_base @ R_gripper_in_ee
        
        q_gripper = self.rotation_matrix_to_quaternion(R_gripper_in_base)

        t_gripper = TransformStamped()
        t_gripper.header.stamp = current_time
        t_gripper.header.frame_id = 'base_link'
        t_gripper.child_frame_id = 'gripper_tip_link'
        t_gripper.transform.translation.x = t_gripper_in_base[0]
        t_gripper.transform.translation.y = t_gripper_in_base[1]
        t_gripper.transform.translation.z = t_gripper_in_base[2]
        t_gripper.transform.rotation.x, t_gripper.transform.rotation.y, t_gripper.transform.rotation.z, t_gripper.transform.rotation.w = q_gripper
        self.tf_broadcaster.sendTransform(t_gripper)


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