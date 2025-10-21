#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
# from camera_info_manager import CameraInfoManager # 보정 데이터 처리용 (필요시 활성화)
import cv_bridge
import numpy as np
import time

# 기존 카메라 클래스와 설정 함수 import
# 패키지 구조에 맞게 경로를 확인하세요
from .astra_camera_driver import setup_openni2_environment, OpenNI2Camera

# --- CameraInfo 생성 함수 ---
def create_camera_info(width, height, fx, fy, cx, cy, distortion_model="plumb_bob", D=None):
    """CameraInfo 메시지를 생성합니다."""
    # 임시 노드를 사용하여 현재 시간 가져오기
    temp_node = Node('__temp_camera_info_node__')
    now = temp_node.get_clock().now().to_msg()
    temp_node.destroy_node() # 임시 노드 즉시 제거

    camera_info_msg = CameraInfo()
    camera_info_msg.header.stamp = now # 현재 시간 사용
    camera_info_msg.header.frame_id = "camera_link" # 적절한 frame_id 설정 필요
    camera_info_msg.width = width
    camera_info_msg.height = height

    # 내부 카메라 행렬 K (Intrinsic camera matrix)
    camera_info_msg.k = [fx, 0.0, cx,
                         0.0, fy, cy,
                         0.0, 0.0, 1.0]

    # 왜곡 계수 D (Distortion coefficients)
    if D is None:
        # 제공되지 않으면 왜곡 없음으로 가정
        D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info_msg.d = D
    camera_info_msg.distortion_model = distortion_model

    # Rectification 행렬 R (단일 카메라이므로 단위 행렬)
    camera_info_msg.r = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]

    # 투영/카메라 행렬 P (Projection/camera matrix)
    camera_info_msg.p = [fx, 0.0, cx, 0.0,
                         0.0, fy, cy, 0.0,
                         0.0, 0.0, 1.0, 0.0]

    # Binning 및 ROI (사용하지 않으면 0으로 설정)
    camera_info_msg.binning_x = 0
    camera_info_msg.binning_y = 0
    camera_info_msg.roi.x_offset = 0
    camera_info_msg.roi.y_offset = 0
    camera_info_msg.roi.height = 0
    camera_info_msg.roi.width = 0
    camera_info_msg.roi.do_rectify = False

    return camera_info_msg

# --- ROS 2 노드 클래스 ---
class OpenNI2CameraNode(Node):
    def __init__(self):
        super().__init__('openni2_camera_node')
        self.get_logger().info("OpenNI2 카메라 노드 초기화 중...")

        # --- 파라미터 선언 ---
        self.declare_parameter('camera_name', 'camera') # 카메라 이름 (토픽 네임스페이스)
        self.declare_parameter('color_frame_id', 'camera_color_optical_frame') # 컬러 이미지 TF 프레임 ID
        self.declare_parameter('depth_frame_id', 'camera_depth_optical_frame') # 뎁스 이미지 TF 프레임 ID
        self.declare_parameter('publish_rate_hz', 30.0) # 발행 빈도 (Hz)
        # 필요시 파라미터 추가 (예: 카메라 내부 파라미터 파일 경로 등)

        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.color_frame_id = self.get_parameter('color_frame_id').get_parameter_value().string_value
        self.depth_frame_id = self.get_parameter('depth_frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.01 # 타이머 주기 (초)

        # --- OpenNI2 환경 설정 ---
        if not setup_openni2_environment():
            self.get_logger().fatal("OpenNI2 환경 설정 실패. 노드를 종료합니다.")
            raise RuntimeError("OpenNI2 환경 설정 실패.")
        self.get_logger().info("OpenNI2 환경 설정 완료.")

        # --- 카메라 초기화 ---
        self.camera = OpenNI2Camera(self.get_logger())
        if not self.camera.initialize():
             self.get_logger().fatal("OpenNI2 카메라 초기화 실패. 노드를 종료합니다.")
             raise RuntimeError("OpenNI2 카메라 초기화 실패.")
        self.get_logger().info("OpenNI2 카메라 초기화 성공.")

        # --- ROS 인터페이스 설정 ---
        self.bridge = cv_bridge.CvBridge() # OpenCV <-> ROS 이미지 메시지 변환기
        base_topic = self.camera_name # 기본 토픽 경로 (예: "camera")

        # 퍼블리셔 생성
        self.color_pub = self.create_publisher(Image, f'{base_topic}/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, f'{base_topic}/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, f'{base_topic}/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, f'{base_topic}/depth/camera_info', 10)

        # --- 카메라 정보 (CameraInfo) 생성 ---
        # 주의: OpenNI2Camera 클래스에 하드코딩된 내부 파라미터 사용 중
        # 이상적으로는 보정 파일에서 로드하거나 장치 API에서 가져와야 함
        self.depth_width = 640 # 코드 로그 기반 추정
        self.depth_height = 480
        self.color_width = 640 # 추정
        self.color_height = 480

        # CameraInfo 메시지 생성 (현재 코드에는 뎁스 파라미터만 있음)
        self.depth_cam_info = create_camera_info(
            self.depth_width, self.depth_height,
            self.camera.depth_fx, self.camera.depth_fy, # 클래스에서 가져옴
            self.camera.depth_cx, self.camera.depth_cy
        )
        self.depth_cam_info.header.frame_id = self.depth_frame_id # 올바른 프레임 ID 설정

        # !! 컬러 카메라 정보 임시 설정 - 뎁스 파라미터 사용 중 !!
        # !! 실제 컬러 카메라 값 (fx, fy, cx, cy)으로 교체 필요 !!
        self.get_logger().warning("컬러 CameraInfo에 뎁스 내부 파라미터를 사용합니다. 보정하거나 설정해주세요!")
        self.color_cam_info = create_camera_info(
            self.color_width, self.color_height,
            self.camera.depth_fx, self.camera.depth_fy, # 뎁스 값 사용
            self.camera.depth_cx, self.camera.depth_cy  # 뎁스 값 사용
        )
        self.color_cam_info.header.frame_id = self.color_frame_id # 올바른 프레임 ID 설정

        # --- 프레임 발행 타이머 ---
        self.timer = self.create_timer(timer_period, self.publish_frames)
        self.get_logger().info(f"발행 빈도: 약 {publish_rate} Hz.")
        self.get_logger().info(f"컬러 이미지 토픽: {self.color_pub.topic_name}")
        self.get_logger().info(f"뎁스 이미지 토픽: {self.depth_pub.topic_name}")
        self.get_logger().info(f"컬러 정보 토픽: {self.color_info_pub.topic_name}")
        self.get_logger().info(f"뎁스 정보 토픽: {self.depth_info_pub.topic_name}")


    def publish_frames(self):
        """카메라에서 프레임을 가져와 발행합니다."""
        try:
            timestamp = self.get_clock().now().to_msg() # 현재 시간
            depth_image, color_image = self.camera.get_frames() # 카메라에서 프레임 가져오기

            # 컬러 이미지 및 정보 발행
            if color_image is not None:
                # OpenCV 이미지를 ROS Image 메시지로 변환 (BGR8 형식)
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_msg.header.stamp = timestamp
                color_msg.header.frame_id = self.color_frame_id
                self.color_pub.publish(color_msg)

                # CameraInfo 타임스탬프 업데이트 후 발행
                self.color_cam_info.header.stamp = timestamp
                self.color_info_pub.publish(self.color_cam_info)
            # else:
            #     self.get_logger().debug("컬러 프레임 없음.")

            # 뎁스 이미지 및 정보 발행
            if depth_image is not None:
                # OpenNI 뎁스는 보통 16비트 unsigned integer (mm 단위) -> 16UC1 인코딩 사용
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = self.depth_frame_id
                self.depth_pub.publish(depth_msg)

                # CameraInfo 타임스탬프 업데이트 후 발행
                self.depth_cam_info.header.stamp = timestamp
                self.depth_info_pub.publish(self.depth_cam_info)
            # else:
            #     self.get_logger().debug("뎁스 프레임 없음.")

        except RuntimeError as e:
            self.get_logger().error(f"프레임 가져오기 실패: {e}")
            # 필요시 재연결 시도 또는 노드 종료 로직 추가
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f"CvBridge 변환 오류: {e}")
        except Exception as e:
            self.get_logger().error(f"발행 루프 중 예상치 못한 오류: {e}", include_traceback=True)

    def destroy_node(self):
        """노드 종료 시 리소스 정리."""
        self.get_logger().info("OpenNI2 카메라 노드 종료 중...")
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
        if hasattr(self, 'camera') and self.camera:
            self.camera.cleanup() # 카메라 리소스 정리 함수 호출
        super().destroy_node()

# --- 메인 실행 함수 ---
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenNI2CameraNode() # 노드 생성 및 초기화
        rclpy.spin(node) # 노드 실행 (콜백 대기)
    except (RuntimeError, KeyboardInterrupt) as e: # 초기화 실패 또는 Ctrl+C
        if isinstance(e, RuntimeError):
            print(f"노드 초기화 실패: {e}")
        else: # KeyboardInterrupt
            print("사용자에 의해 노드가 중단되었습니다.")
    except Exception as e: # 그 외 예외 처리
        if node:
            node.get_logger().fatal(f"처리되지 않은 예외 발생: {e}", include_traceback=True)
        else:
            print(f"노드 생성 중 처리되지 않은 예외 발생: {e}")
    finally:
        # 노드 및 rclpy 종료 처리
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("노드 종료 완료.")

if __name__ == '__main__':
    main()