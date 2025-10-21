import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from pymycobot.mycobot import MyCobot
import math
import numpy as np

# TF2 라이브러리 임포트
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy.duration

# Service 메시지 임포트
from javis_interfaces.srv import GetCoordinates

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

        # Service 클라이언트 생성
        self.cli = self.create_client(GetCoordinates, 'get_coordinates')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_coordinates 서비스를 기다리는 중...')
        
        self.get_logger().info('get_coordinates 서비스에 연결되었습니다.')
        
        # 좌표 요청 타이머 생성 (1초마다 한 번씩 서비스 호출)
        self.coordinate_timer = self.create_timer(1.0, self.request_coordinates)
        
        # 좌표 수신 플래그
        self.coordinate_received = False

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

    def request_coordinates(self):
        """주기적으로 좌표 서비스를 호출"""
        # 이미 좌표를 받았으면 더 이상 요청하지 않음
        if self.coordinate_received:
            return
        
        req = GetCoordinates.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_coordinates_response)

    def handle_coordinates_response(self, future):
        """서비스 응답 처리"""
        try:
            response = future.result()
            
            # 좌표가 모두 0이면 아직 유효한 데이터가 없는 것으로 간주
            if response.x == 0.0 and response.y == 0.0 and response.z == 0.0:
                self.get_logger().info('좌표 대기 중... (0, 0, 0 수신)')
                return
            
            # 수신된 좌표 (camera_link 기준)
            x_cam = response.x
            y_cam = response.y
            z_cam = response.z

            self.get_logger().info(
                f"Service 수신 원본 좌표 (camera_link): X={x_cam:.3f}, Y={y_cam:.3f}, Z={z_cam:.3f}"
            )
            self.get_logger().info(f"'camera_link' -> 'base_link' 좌표 변환 시도...")
            
            # TF 변환 행렬 가져오기
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 수동으로 좌표 변환
            class Point:
                def __init__(self, x, y, z):
                    self.x = x
                    self.y = y
                    self.z = z
            
            point_cam = Point(x_cam, y_cam, z_cam)
            transformed_xyz = transform_point(point_cam, transform)
            
            self.get_logger().info(
                f"좌표 변환 성공: X={transformed_xyz[0]:.3f}, "
                f"Y={transformed_xyz[1]:.3f}, Z={transformed_xyz[2]:.3f}"
            )

            # 변환된 좌표로 로봇 제어 (미터 -> 밀리미터)
            x_base_mm = float(transformed_xyz[0] * 1000.0)
            y_base_mm = float(transformed_xyz[1] * 1000.0)
            z_base_mm = float(transformed_xyz[2] * 1000.0)

            # 좌표 범위 체크 (MyCobot 280 작업 공간)
            if (abs(x_base_mm) > 281.45 or abs(y_base_mm) > 281.45 or 
                z_base_mm < -70 or z_base_mm > 412.67):
                self.get_logger().warn(
                    f"변환된 좌표가 로봇 작업 공간을 벗어났습니다: "
                    f"X={x_base_mm:.1f}mm (범위: -281.45~281.45), "
                    f"Y={y_base_mm:.1f}mm (범위: -281.45~281.45), "
                    f"Z={z_base_mm:.1f}mm (범위: -70~412.67)"
                )
                self.get_logger().warn("카메라 좌표계 설정 또는 단위를 확인하세요.")
                return

            current_coords = self.mc.get_coords()
            if current_coords:
                # 현재 자세(rx, ry, rz)는 유지하고 위치만 변경
                rx, ry, rz = current_coords[3], current_coords[4], current_coords[5]
                target_coords = [x_base_mm, y_base_mm, z_base_mm, rx, ry, rz]
                
                self.get_logger().info(
                    f"로봇 이동 명령: X={x_base_mm:.1f}, Y={y_base_mm:.1f}, "
                    f"Z={z_base_mm:.1f}, RX={rx:.1f}, RY={ry:.1f}, RZ={rz:.1f}"
                )
                
                # mode=0: 직선 보간 이동, speed=20
                self.mc.send_coords(target_coords, 20, 0)
                
                # 좌표를 성공적으로 처리했으므로 플래그 설정
                self.coordinate_received = True
                self.get_logger().info("좌표 처리 완료. 더 이상 새로운 좌표를 요청하지 않습니다.")
            else:
                self.get_logger().warn("로봇의 현재 좌표를 읽을 수 없어 이동 명령을 건너뜁니다.")

        except tf2_ros.TransformException as e:
            self.get_logger().error(f"좌표 변환 실패: {e}")
        except Exception as e:
            self.get_logger().error(f"서비스 응답 처리 중 에러: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

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