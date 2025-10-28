import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from pymycobot.mycobot import MyCobot
import math
import socket
import threading
import numpy as np
import time
from enum import Enum # 상태 관리를 위해 Enum 추가

# TF2 라이브러리 임포트
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy.duration

# 상태 정의
class RobotState(Enum):
    WAITING_FOR_OBJECT = 1
    MOVING_TO_APPROACH_POSITION = 2
    LOWERING_TO_PICK = 3
    GRIPPING = 4
    RAISING_AFTER_PICK = 5
    # 필요에 따라 PLACE 관련 상태 추가 가능
    RETURNING_HOME = 6


def rpy_to_quaternion(roll, pitch, yaw):
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    cy = math.cos(yaw_rad * 0.5); sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5); sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5); sr = math.sin(roll_rad * 0.5)
    q = [0.0] * 4
    q[3] = cr * cp * cy + sr * sp * sy  # w
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    return q

def quaternion_to_rotation_matrix(q):
    """쿼터니언을 3x3 회전 행렬로 변환"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])

def transform_point(point, transform):
    """TransformStamped를 사용해 점을 수동으로 변환"""
    # 회전 행렬 생성
    R = quaternion_to_rotation_matrix(transform.transform.rotation)
    
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

class MyCobotController(Node):
    def __init__(self):
        super().__init__('mycobot_controller')
        
        try:
            self.mc = MyCobot('/dev/ttyJETCOBOT', 1000000)
            self.mc.send_angles([0, 0, 0, 0, -90, -45], 50) # 초기 자세 설정
            time.sleep(2)
            self.get_logger().info('myCobot is connected.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to myCobot: {e}')
            rclpy.shutdown()
            return

        # TF 리스너 및 버퍼 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF 브로드캐스터 초기화 (엔드 이펙터 TF 발행용)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.broadcast_timer_callback)

        # UDP 수신기 설정
        self.udp_ip = "0.0.0.0"
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"UDP 수신기 시작. {self.udp_ip}:{self.udp_port}에서 데이터를 기다립니다...")

        # --- 상태 관리 변수 추가 ---
        self.state = RobotState.WAITING_FOR_OBJECT
        self.target_coords = None # 목표 좌표 저장 변수
        self.approach_height = 225 # 접근 높이 (mm)
        self.pick_height_offset = 10 # 물체 높이 + 추가 오프셋 (mm)
        
        self.coordinate_received = False
        self.lock = threading.Lock()

        # UDP 수신을 위한 별도 스레드 시작
        self.udp_thread = threading.Thread(target=self.udp_listener_loop, daemon=True)
        self.udp_thread.start()
        
        # 상태 머신을 실행할 메인 제어 루프 타이머
        self.control_timer = self.create_timer(0.5, self.control_loop)

    def control_loop(self):
        """상태에 따라 로봇을 제어하는 메인 루프"""
        if self.state == RobotState.WAITING_FOR_OBJECT:
            # UDP 스레드가 좌표를 받으면 상태가 변경될 때까지 대기
            return

        if self.state == RobotState.MOVING_TO_APPROACH_POSITION:
            self.get_logger().info("상태: [MOVING_TO_APPROACH_POSITION]")

            time.sleep(3) # 이동 대기
            self.state = RobotState.LOWERING_TO_PICK # 다음 상태로 전환
        
        elif self.state == RobotState.LOWERING_TO_PICK:
            self.get_logger().info("상태: [LOWERING_TO_PICK]")
            current_coords = self.mc.get_coords()
            if not current_coords: return
            
            # z축만 목표 높이로 하강
            pick_coords = [
                self.target_coords[0],
                self.target_coords[1],
                self.target_coords[2] + 100, # 물체 높이 + 오프셋
                current_coords[3],
                current_coords[4],
                current_coords[5]
            ]
            approach_test1 = self.mc.send_coords(pick_coords, 30, 0)
            if not approach_test1:
                self.get_logger().info("이동안했음.")
            time.sleep(2)
            self.state = RobotState.GRIPPING

        elif self.state == RobotState.GRIPPING:
            self.get_logger().info("상태: [GRIPPING]")
            self.mc.set_gripper_value(0, 50)
            time.sleep(1)
            self.state = RobotState.RAISING_AFTER_PICK

        elif self.state == RobotState.RAISING_AFTER_PICK:
            self.get_logger().info("상태: [RAISING_AFTER_PICK]")
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
            self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
            self.mc.set_gripper_value(100, 50)
            time.sleep(2)
            self.get_logger().info("작업 완료. 노드를 종료합니다.")
            rclpy.shutdown()

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
        q = rpy_to_quaternion(coords[3], coords[4], coords[5])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q[0], q[1], q[2], q[3]
        self.tf_broadcaster.sendTransform(t)

    def udp_listener_loop(self):
        while rclpy.ok():
            try:
                # --- 이미 좌표를 받았다면 스레드를 안전하게 종료 ---
                with self.lock:
                    if self.coordinate_received:
                        self.get_logger().info("좌표를 이미 수신했으므로 UDP 리스너 스레드를 종료합니다.")
                        break # while 루프를 빠져나가 스레드를 종료

                data, addr = self.sock.recvfrom(1024)
                
                # coordinate_received 플래그가 True로 바뀌는 것을 방지하기 위해 
                # 데이터 처리 중에도 플래그를 다시 한번 확인
                with self.lock:
                    if self.coordinate_received:
                        continue

                message = data.decode('utf-8')
                
                parts = message.split(',')
                if len(parts) == 3:
                    try:
                        x_cam = float(parts[0])
                        y_cam = float(parts[1])
                        z_cam = float(parts[2])

                        self.get_logger().info(f"'camera_link' -> 'base_link' 좌표 변환 시도...")
                        
                        transform = self.tf_buffer.lookup_transform(
                            'base_link', 'camera_link', rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=1.0)
                        )
                        
                        class Point:
                            def __init__(self, x, y, z): self.x, self.y, self.z = x, y, z
                        
                        transformed_xyz = transform_point(Point(x_cam, y_cam, z_cam), transform)
                        
                        self.get_logger().info(
                            f"좌표 변환 성공: X={transformed_xyz[0]:.3f}, Y={transformed_xyz[1]:.3f}, Z={transformed_xyz[2]:.3f}"
                        )

                        x_base_mm = float(transformed_xyz[0] * 1000.0)
                        y_base_mm = float(transformed_xyz[1] * 1000.0)
                        z_base_mm = float(transformed_xyz[2] * 1000.0)
                        
                        # --- 여기서부터 동기화 처리 ---
                        with self.lock:
                            # 변환된 좌표를 클래스 변수에 저장하고 상태 변경
                            self.target_coords = [x_base_mm, y_base_mm, z_base_mm]
                            self.state = RobotState.MOVING_TO_APPROACH_POSITION
                            
                            # --- 플래그를 True로 설정하여 더 이상 좌표를 받지 않도록 함 ---
                            self.coordinate_received = True 
                            self.get_logger().info("좌표 수신 완료. 픽앤플레이스 작업을 시작합니다.")


                    except (ValueError, tf2_ros.TransformException) as e:
                        self.get_logger().error(f"좌표 변환 또는 처리 실패: {e}")
                else:
                    self.get_logger().warn(f"잘못된 형식의 메시지 수신: '{message}'")
            except Exception as e:
                # coordinate_received 플래그가 True가 되어 루프가 종료될 때 sock.recvfrom에서 예외가 발생할 수 있음
                # 이 경우, 정상적인 종료이므로 에러 메시지를 출력하지 않음
                with self.lock:
                    if not self.coordinate_received and rclpy.ok():
                        self.get_logger().error(f"UDP 수신 루프 에러: {e}")

    def destroy_node(self):
        self.get_logger().info("노드 종료. UDP 소켓을 닫습니다.")
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotController()
    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()