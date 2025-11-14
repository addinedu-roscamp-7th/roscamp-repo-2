import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
# 필요한 메시지 타입 import
from std_msgs.msg import Float64 # <--- Float64 메시지 타입 추가
from javis_interfaces.action import PerformTask
from javis_kc.move_robot_arm_client import MoveRobotArmClient
from pymycobot.mycobot280 import MyCobot280
import time
import math
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
    MOVING_TO_APPROACH = 2
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
        
        # 객체 인식 노드로부터 받은 각도를 저장할 변수 (초기값 0.0)
        self.object_rotation_angle = 0.0 
        
        # ROS 구독자 선언: '/rotation_angle' 토픽 구독
        self.angle_subscriber = self.create_subscription(
            Float64,
            'rotation_angle',
            self.angle_callback, # <--- 콜백 함수 연결
            10
        )

        tf_cmd = [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0.31", "0.505", "0.280", "0.0", "3.1415", "1.5707",
            "base_link", "camera_link"
        ]
        subprocess.Popen(tf_cmd)

        self.get_logger().info('정적 TF 발행기(카메라) 시작. Gripper Tip은 동적으로 계산됩니다.')


        self._action_server = ActionServer(
            self,
            PerformTask,
            '/kreacher/action/perform_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )

        self.feedback_msg = PerformTask.Feedback()

        self.mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
        self.mc.thread_lock = True
        self.mc.send_angles([0.0,0.0,0.0,-90.0,0.0,-45.0], 30)
        self.mc.set_gripper_value(100, 50)
        self.get_logger().info('MyCobot 연결 완료')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.state = RobotState.WAITING_FOR_OBJECT
        # 목표 좌표 (gripper_tip_link 기준, base_link 좌표계)
        self.target_coords = [0.0, 0.0, 0.0]
        # 물체에 접근할 때의 Z축 높이 (mm)
        self.approach_height_mm = 225.0
        # 물체를 잡을 때 사용할 엔드 이펙터의 목표 자세 (RPY, degrees)
        # R, P는 고정, Y값은 아래 execute_callback에서 덮어쓰기 위해 초기값 설정
        self.pick_orientation_rpy = [-179.0, 0.0, -45.0] 
        
        # 현재 처리 중인 메뉴 ID를 저장
        self.current_menu_id = None
        
        self.get_logger().info('액션서버 시작. 목표를 기다립니다...')

    def angle_callback(self, msg):
        """'/rotation_angle' 토픽으로부터 각도 값을 수신합니다."""
        # rotation.py에서 보낸 각도는 0~90도 절대값입니다.
        self.object_rotation_angle = msg.data
        self.get_logger().debug(f"수신된 객체 회전 각도: {self.object_rotation_angle:.2f} degrees")


    def goal_callback(self, goal_request):
        """새로운 목표가 들어왔을 때 수락할지 거절할지 결정합니다."""
        if self._is_busy:
            self.get_logger().info('서버가 이미 다른 작업을 수행 중이므로 새 목표를 거절합니다.')
            return GoalResponse.REJECT
        
        self.get_logger().info('새로운 목표를 수락합니다.')
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """목표를 수락했을 때 실제 작업을 수행하는 함수."""
        self.get_logger().info(f'작업 실행 - order_id: {goal_handle.request.order_id}, menu_id: {goal_handle.request.menu_id}')
        
        self._is_busy = True
        
        try:
            # menu_id에 따른 처리
            if goal_handle.request.menu_id not in [1, 2]:
                error_msg = f'지원하지 않는 menu_id: {goal_handle.request.menu_id}'
                self.get_logger().error(error_msg)
                goal_handle.abort()
                return PerformTask.Result(order_id=goal_handle.request.order_id, success=False, pick_up_num=0, message=error_msg)
            
            self.get_logger().info(f'메뉴 {goal_handle.request.menu_id}번 제조를 시작합니다.')
            
            # 현재 처리할 메뉴 ID 설정
            self.current_menu_id = goal_handle.request.menu_id
            
            # 1. 클라이언트를 통해 목표 좌표를 요청
            move_robot_arm_client = MoveRobotArmClient()
            self.get_logger().info('목표 객체의 좌표를 요청합니다...')
            res = move_robot_arm_client.send_request()
            
            # --- 객체 각도에 따른 로봇 Yaw 각도 계산 로직 ---
            calculated_yaw = self.pick_orientation_rpy[2] # 초기 Z축(Yaw) 각도. -45.0도

            if 0.0 <= self.object_rotation_angle <= 45.0:
                # 0 ~ 45도 사이면 '현재각도'를 Yaw 값에 더함
                calculated_yaw -= self.object_rotation_angle
                self.get_logger().info(f"객체 각도({self.object_rotation_angle:.2f}도) -> Yaw에 그대로 반영: {calculated_yaw:.2f}도")
            elif 45.0 < self.object_rotation_angle <= 90.0:
                # 45 ~ 90도 사이면 '현재각도 - 90'을 Yaw 값에 더함
                angle_offset = self.object_rotation_angle - 90.0
                calculated_yaw -= angle_offset
                self.get_logger().info(f"객체 각도({self.object_rotation_angle:.2f}도) -> Yaw에 (각도-90) 반영: {calculated_yaw:.2f}도")
            else:
                 # 예외 처리 (각도가 범위를 벗어난 경우)
                 self.get_logger().warn(f"객체 각도({self.object_rotation_angle:.2f}도)가 0~90도 범위를 벗어났습니다. 기본 Yaw 각도를 사용합니다.")


            # 최종 계산된 Yaw 각도로 pick_orientation_rpy 업데이트
            # [R, P, Y] = [roll, pitch, yaw]
            pick_orientation_for_coords = [
                self.pick_orientation_rpy[0],  # Roll: -179.0
                self.pick_orientation_rpy[1],  # Pitch: 0.0
                calculated_yaw                  # Yaw: 계산된 값
            ]
            # -----------------------------------------------


            # 2. 유효한 좌표인지 확인
            if res.x == 0.0 and res.y == 0.0 and res.z == 0.0:
                error_msg = '유효하지 않은 좌표(0,0,0)를 수신하여 작업을 중단합니다.'
                self.get_logger().error(error_msg)
                goal_handle.abort()
                return PerformTask.Result(success=False, message=error_msg)

            # 3. TF 변환을 수행합니다. (camera_link -> base_link)
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'camera_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )
                # transform_point의 결과는 gripper_tip_link가 가야 할 base_link 기준 좌표
                transform_xyz = self.transform_point(Point(res.x, res.y, res.z), transform)
                self.get_logger().info(f'변환된 gripper_tip 목표 좌표: x: {transform_xyz[0]}, y: {transform_xyz[1]}, z: {transform_xyz[2]}')
                self.target_coords = [transform_xyz[0], transform_xyz[1], transform_xyz[2]]

                self.get_logger().info("계산 시작: gripper_tip 목표 지점 도달을 위한 end_effector 좌표 계산")
                
                # 1. Gripper가 객체 바로 위(접근 높이)에 위치할 좌표 계산
                approach_pos_gripper = [
                    self.target_coords[0], 
                    self.target_coords[1], 
                    self.approach_height_mm / 1000.0  #미터 단위로 변환
                ]
                
                # 2. 접근 위치로 가기 위한 end_effector의 좌표와 자세 계산
                # 계산된 각도 (pick_orientation_for_coords)를 사용하여 자세 계산
                approach_coords_ee = self._calculate_end_effector_pose(
                    approach_pos_gripper,
                    pick_orientation_for_coords # <--- 수정된 RPY 자세 사용
                )
                
                self.get_logger().info(f"계산된 end_effector 접근 좌표: {approach_coords_ee}")
                
                # 3. 계산된 좌표로 로봇 팔 이동 (기존 send_angles 대체)
                self.mc.send_coords(approach_coords_ee, 30)
                time.sleep(3.0)
                
                # --- 최종 피킹 자세에 계산된 각도를 반영하기 위해 멤버 변수 업데이트 ---
                self.pick_orientation_rpy[2] = calculated_yaw


            except tf2_ros.TransformException as ex:
                error_msg = f'TF 변환 실패: {ex}'
                self.get_logger().error(error_msg)
                goal_handle.abort()
                return PerformTask.Result(success=False, message=error_msg)

            # 4. 상태 머신 시작
            self.state = RobotState.LOWERING_TO_PICK
            
            while self.state != RobotState.RETURNING_HOME and rclpy.ok():
                self.control_loop()
                self.feedback_msg.order_id = goal_handle.request.order_id
                self.feedback_msg.progress_percentage = self.get_progress_by_state()
                goal_handle.publish_feedback(self.feedback_msg)
                time.sleep(1)

            if self.state == RobotState.RETURNING_HOME and rclpy.ok():
                 self.get_logger().info("마지막 단계: 홈으로 복귀 중...")
                 self.control_loop()

            goal_handle.succeed()
            result = PerformTask.Result()
            result.order_id = goal_handle.request.order_id
            result.success = True
            result.pick_up_num = 1 if goal_handle.request.menu_id == 1 else 2  # menu_id에 따라 다른 픽업 번호 할당
            result.message = f'메뉴 {goal_handle.request.menu_id}번 음료 제조 완료. 픽업 번호: {result.pick_up_num}'
            self.get_logger().info('목표 수행 성공!')
            return result

        finally:
            self.get_logger().info("작업 완료. 다음 목표를 받을 준비가 되었습니다.")
            self._is_busy = False
            self.state = RobotState.WAITING_FOR_OBJECT
            # 작업이 끝난 후 혹시 모를 다음 목표를 위해 Yaw 각도를 초기값으로 되돌릴 수 있음 (선택 사항)


    def _rpy_deg_to_rotation_matrix(self, rpy_deg):
        """RPY 각도(degree)를 회전 행렬(numpy array)로 변환합니다."""
        return self._rpy_rad_to_rotation_matrix(list(map(math.radians, rpy_deg)))

    def _rpy_rad_to_rotation_matrix(self, rpy_rad):
        """RPY 각도(radian)를 회전 행렬(numpy array)로 변환합니다."""
        roll, pitch, yaw = rpy_rad[0], rpy_rad[1], rpy_rad[2]
        
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
                       
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
                       
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        # ZYX 순서로 회전 적용
        return Rz @ Ry @ Rx

    def _calculate_end_effector_pose(self, gripper_tip_target_pos_m, end_effector_target_rpy_deg):
        """
        목표 gripper_tip 위치와 end_effector 자세를 기반으로
        myCobot에 전달할 end_effector의 좌표(mm, deg)를 계산합니다.
        """
        # end_effector_link 기준 gripper_tip_link의 상대 위치 (m)
        offset_in_ee_frame = np.array([0.06, -0.06, 0.005])

        # 목표 end_effector 자세에 대한 회전 행렬 계산
        R_base_to_ee = self._rpy_deg_to_rotation_matrix(end_effector_target_rpy_deg)
        
        # 오프셋 벡터를 base_link 좌표계로 변환
        offset_in_ee_frame = np.array([0.06, -0.06, 0.005])
        offset_in_base_frame = R_base_to_ee @ offset_in_ee_frame
        
        # gripper_tip 목표 위치에서 변환된 오프셋을 빼서 end_effector 목표 위치 계산
        P_gripper_tip = np.array(gripper_tip_target_pos_m)
        P_end_effector = P_gripper_tip - offset_in_base_frame
        
        # myCobot의 send_coords 형식(mm, deg)으로 변환하여 반환
        return [
            P_end_effector[0] * 1000.0 + 10.0,
            P_end_effector[1] * 1000.0 - 40.0,
            P_end_effector[2] * 1000.0,
            float(end_effector_target_rpy_deg[0]),
            float(end_effector_target_rpy_deg[1]),
            float(end_effector_target_rpy_deg[2])
        ]

    def get_progress_by_state(self):
        """현재 상태에 따라 진행률을 반환합니다."""
        if self.state == RobotState.LOWERING_TO_PICK: return 30.0
        if self.state == RobotState.GRIPPING: return 60.0
        if self.state == RobotState.RAISING_AFTER_PICK: return 80.0
        if self.state == RobotState.RETURNING_HOME: return 95.0
        return 0.0

    # control_loop1, control_loop2, rpy_to_quaternion, rotation_matrix_to_quaternion, 
    # quaternion_to_rotation_matrix, transform_point, broadcast_timer_callback 
    # 함수는 변경 없이 유지합니다.
    # ... (생략된 기존 코드)

    def control_loop(self):
        """상태에 따라 로봇을 제어하는 메인 루프"""
        if self.current_menu_id == 1:
            self.control_loop1()
        elif self.current_menu_id == 2:
            self.control_loop2()
        else:
            self.get_logger().error(f"지원하지 않는 메뉴 ID: {self.current_menu_id}")

    def control_loop1(self):
        """메뉴 1번을 처리하는 control loop"""
        if self.state == RobotState.LOWERING_TO_PICK:
            self.get_logger().info("상태: [LOWERING_TO_PICK]")
            
            # 잡기 직전의 gripper_tip 위치 (z축으로 약간의 오프셋 추가)
            final_gripper_tip_pos = [
                self.target_coords[0],
                self.target_coords[1],
                self.target_coords[2] + 0.030  # 30mm 오프셋 (미터 단위)
            ]
            
            # 최종 잡기 위치에 도달하기 위한 end_effector 좌표 계산
            # 여기서 self.pick_orientation_rpy의 Yaw는 이미 execute_callback에서 계산된 각도로 업데이트됨
            final_pick_coords_ee = self._calculate_end_effector_pose(
                final_gripper_tip_pos,
                self.pick_orientation_rpy # <--- 업데이트된 Yaw 각도 사용
            )
            
            self.get_logger().info(f"계산된 end_effector 최종 잡기 좌표: {final_pick_coords_ee}")
            
            # 계산된 좌표로 이동
            self.mc.sync_send_coords(final_pick_coords_ee, 40, 1)
            time.sleep(1.5)
            self.state = RobotState.GRIPPING

        elif self.state == RobotState.GRIPPING:
            self.get_logger().info("상태: [GRIPPING]")
            self.mc.set_gripper_value(0, 50)
            time.sleep(1.5)
            self.state = RobotState.RAISING_AFTER_PICK

        elif self.state == RobotState.RAISING_AFTER_PICK:
            self.get_logger().info("상태: [RAISING_AFTER_PICK]")
            
            # 물체를 들어올리는 동작 (기존 로직 유지)
            raise_angles = [-1.14, 10.89, -63.63, -40.25, -2.02, -45.87]
            self.mc.send_angles(raise_angles, 30)
            time.sleep(5.0)

            # 이후 디스펜스 동작 (기존 로직 유지)
            reverse_cup = [-1.14, 10.89, -63.63, 140.0, -2.02, -45.87]
            self.mc.send_angles(reverse_cup, 30)
            time.sleep(5.0)
            ready_to_place = [91.14, -81.73, 92.54, 82.44, -0.08, -42.62]
            self.mc.send_angles(ready_to_place, 30)
            time.sleep(5.5)
            close_to_dispense = [47.72, -71.19, 118.82, 51.94, -4.48, -7.64]
            self.mc.send_angles(close_to_dispense, 30)
            time.sleep(9.5)
            finish_to_dispense = [85.07, -93.33, 130.42, 52.99, 1.4, -36.91]
            self.mc.send_angles(finish_to_dispense, 30)
            time.sleep(5.0)
            finish_to_dispense1 = [87.36, -1.31, 6.85, 79.54, 0.96, -39.63]
            self.mc.send_angles(finish_to_dispense1, 30)
            time.sleep(5.0)
            finish_to_dispense2 = [-88.41, -0.79, -1.14, 81.91, -4.39, -45.0]
            self.mc.send_angles(finish_to_dispense2, 30)
            time.sleep(5.0)
            finish_to_dispense3 = [-92.19, 135.0, -83.84, 45.7, -4.74, -43.41]
            self.mc.send_angles(finish_to_dispense3, 20)
            time.sleep(5.0)
            self.mc.set_gripper_value(100, 50)
            time.sleep(5.0)
            finish_to_dispense4 = [-89.38, 109.07, -95.97, 74.61, -2.81, -45.79]
            self.mc.send_angles(finish_to_dispense4, 30)
            time.sleep(3.0)
            self.state = RobotState.RETURNING_HOME

        elif self.state == RobotState.RETURNING_HOME:
            self.get_logger().info("상태: [RETURNING_HOME]")
            self.mc.send_angles([1.23, 4.13, 0.61, 83.58, -0.43, -43.15], 30)
            time.sleep(2.5)

    def control_loop2(self):
        """메뉴 2번을 처리하는 control loop - 기본 동작은 1번과 동일하지만 다른 위치에 배출"""
        if self.state == RobotState.LOWERING_TO_PICK:
            self.get_logger().info("상태: [LOWERING_TO_PICK]")
            
            # 잡기 직전의 gripper_tip 위치 (z축으로 약간의 오프셋 추가)
            final_gripper_tip_pos = [
                self.target_coords[0],
                self.target_coords[1],
                self.target_coords[2] + 0.020  # 30mm 오프셋 (미터 단위)
            ]
            
            # 최종 잡기 위치에 도달하기 위한 end_effector 좌표 계산
            # 여기서 self.pick_orientation_rpy의 Yaw는 이미 execute_callback에서 계산된 각도로 업데이트됨
            final_pick_coords_ee = self._calculate_end_effector_pose(
                final_gripper_tip_pos,
                self.pick_orientation_rpy # <--- 업데이트된 Yaw 각도 사용
            )

            #얼음 집으러가기
            preprocess_of_picking_ice_angles = [91.4, -45.52, -38.32, -103.79, 91.58, -52.29]
            self.mc.send_angles(preprocess_of_picking_ice_angles, 30)
            time.sleep(2.0)
            preprocess_of_picking_ice_angles2 = [92.46, -62.4, -37.44, -99.31, 92.72, -63.01]
            self.mc.send_angles(preprocess_of_picking_ice_angles2, 30)
            time.sleep(2.0)
            #얼음집기
            self.mc.set_gripper_value(0, 50)
            time.sleep(1.5)
            self.mc.send_angles(preprocess_of_picking_ice_angles, 30)
            time.sleep(1.5)

            
            self.get_logger().info(f"계산된 end_effector 최종 잡기 좌표: {final_pick_coords_ee}")
            # 계산된 위 좌표
            self.mc.sync_send_coords([final_pick_coords_ee[0],final_pick_coords_ee[1],225.0,final_pick_coords_ee[3],final_pick_coords_ee[4],final_pick_coords_ee[5]], 40, 1)
            time.sleep(1.5)
            #얼음놓기
            self.mc.set_gripper_value(100, 50)
            time.sleep(1.5)
            # 계산된 좌표로 이동
            self.mc.sync_send_coords(final_pick_coords_ee, 40, 1)
            time.sleep(1.5)
            self.state = RobotState.GRIPPING

        elif self.state == RobotState.GRIPPING:
            self.get_logger().info("상태: [GRIPPING]")
            self.mc.set_gripper_value(0, 50)
            time.sleep(1.5)
            self.state = RobotState.RAISING_AFTER_PICK

        elif self.state == RobotState.RAISING_AFTER_PICK:
            self.get_logger().info("상태: [RAISING_AFTER_PICK]")
            
            # 물체를 들어올리는 동작 (기존 로직 유지)
            raise_angles = [-1.14, 10.89, -63.63, -40.25, -2.02, -45.87]
            self.mc.send_angles(raise_angles, 30)
            time.sleep(5.0)

            # 이후 디스펜스 동작 (두 번째 위치에 배출하도록 각도 수정)
            ready_to_place = [-6.32, 7.73, -42.53, -38.93, 7.64, 66.53]
            self.mc.send_angles(ready_to_place, 30)
            time.sleep(5.5)
            # 두 번째 픽업 위치를 위한 각도 수정
            close_to_dispense = [-37.88, 14.32, -42.53, -67.32, 10.45, 18.45]
            self.mc.send_angles(close_to_dispense, 30)
            time.sleep(9.5)
            self.mc.send_angles(ready_to_place, 30)
            time.sleep(5.0)
            # 두 번째 픽업 위치를 위한 각도 수정
            finish_to_dispense1 = [92.63, 5.62, -16.52, -76.81, 0.52, -37.7]
            self.mc.send_angles(finish_to_dispense1, 30)
            time.sleep(5.0)
            finish_to_dispense2 = [93.69, -67.14, -16.69, -4.3, -2.63, -39.02]
            self.mc.send_angles(finish_to_dispense2, 30)
            time.sleep(5.0)
            self.mc.set_gripper_value(100, 50)
            time.sleep(5.0)
            finish_to_dispense3 = [93.69, -7.91, -47.46, -35.41, -5.44, -38.32]
            self.mc.send_angles(finish_to_dispense3, 30)
            time.sleep(3.0)
            self.state = RobotState.RETURNING_HOME

        elif self.state == RobotState.RETURNING_HOME:
            self.get_logger().info("상태: [RETURNING_HOME]")
            self.mc.send_angles([1.23, 4.13, 0.61, 83.58, -0.43, -43.15], 30)
            time.sleep(2.5)
            self.mc.send_angles([0.0,0.0,0.0,-90.0,0.0,-45.0], 30)

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
        """회전 행렬을 ROS 쿼터니언 [x, y, z, w]으로 변환합니다."""
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

    def broadcast_timer_callback(self):
        #로봇에서 현재 end_effector_link 좌표 가져오기
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

        # 3. 'gripper_tip_link'의 TF를 동적으로 계산하여 발행
        # 수식: T(base->gripper) = T(base->ee) * T(ee->gripper)

        # 3.1. T(ee->gripper) 정의: end_effector_link 기준 gripper_tip_link의 정적 변환
        t_gripper_in_ee = np.array([0.06, -0.06, 0.005]) # x, y, z (m)
        rpy_gripper_in_ee_rad = [0.0, 0.0, -0.7854] # roll, pitch, yaw (rad)
        R_gripper_in_ee = self._rpy_rad_to_rotation_matrix(rpy_gripper_in_ee_rad)

        # 3.2. T(base->ee) 정의: base_link 기준 end_effector_link의 실시간 변환
        t_ee_in_base = np.array([t_ee.transform.translation.x, t_ee.transform.translation.y, t_ee.transform.translation.z])
        R_ee_in_base = self._rpy_deg_to_rotation_matrix([coords[3], coords[4], coords[5]])

        # 3.3. T(base->gripper) 계산
        # 최종 위치 = t(base->ee) + R(base->ee) * t(ee->gripper)
        t_gripper_in_base = t_ee_in_base + R_ee_in_base @ t_gripper_in_ee
        # 최종 자세 = R(base->ee) * R(ee->gripper)
        R_gripper_in_base = R_ee_in_base @ R_gripper_in_ee
        
        q_gripper = self.rotation_matrix_to_quaternion(R_gripper_in_base)

        # 3.4. 계산된 gripper_tip_link TF 발행
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