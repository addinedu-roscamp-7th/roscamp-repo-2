# javis_dvs_node.py (일부 수정 예시)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
import cv2
import threading

# ... (기존의 다른 import 들은 그대로 유지) ...

# ❌ 이제 이 부분들은 필요 없습니다! 모두 삭제하세요.
# from .astra_camera_driver import setup_openni2_environment, OpenNI2Camera

class DVSNode(Node): # 기존 노드 클래스 이름이 VSNode 였다면 그대로 사용
    def __init__(self):
        super().__init__('dvs_node') # 노드 이름
        self.bridge = cv_bridge.CvBridge()
        self.color_image = None
        self.depth_image = None
        self.frame_lock = threading.Lock() # 여러 스레드에서 이미지에 접근할 때 필요

        # ... (기존의 다른 서비스, 퍼블리셔, 액션 서버 초기화는 그대로 유지) ...

        # --- 👇 카메라 직접 제어 코드를 토픽 구독 코드로 변경 👇 ---

        # 1. 컬러 이미지 토픽 구독자 생성
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # astra_camera 노드가 발행하는 토픽 이름
            self.color_image_callback,
            10)

        # 2. 뎁스 이미지 토픽 구독자 생성
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # astra_camera 노드가 발행하는 토픽 이름
            self.depth_image_callback,
            10)

        # 3. (필요시) 카메라 정보 토픽 구독자 생성
        self.color_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.color_info_callback,
            10)
        # ... depth_info_sub 도 동일하게 ...

        self.get_logger().info("카메라 토픽 구독 준비 완료. /camera/color/image_raw, /camera/depth/image_raw 대기 중...")
        # ❌ OpenNI2Camera, setup_openni2_environment 관련 초기화 코드는 모두 삭제!

    # --- 👇 토픽 메시지를 받았을 때 실행될 콜백 함수들 추가 👇 ---

    def color_image_callback(self, msg):
        """컬러 이미지 토픽을 받으면 OpenCV 이미지로 변환하여 저장"""
        try:
            # ROS Image 메시지를 OpenCV BGR 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                self.color_image = cv_image
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CvBridge 변환 오류: {e}')

    def depth_image_callback(self, msg):
        """뎁스 이미지 토픽을 받으면 OpenCV 이미지로 변환하여 저장"""
        try:
            # ROS Image 메시지를 OpenCV 16UC1 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            with self.frame_lock:
                self.depth_image = cv_image
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CvBridge 변환 오류: {e}')

    def color_info_callback(self, msg):
        # 카메라 내부 파라미터가 필요할 때 여기서 처리
        # 예: self.camera_fx = msg.k[0]
        pass

    # ... (기존의 다른 서비스 콜백, 액션 콜백 등은 그대로 유지) ...

    # 예시: 기존의 메인 루프나 타이머에서 이미지를 사용하던 부분
    def main_processing_loop(self): # 타이머 콜백 함수라고 가정
        with self.frame_lock:
            # 클래스 변수에서 최신 이미지를 복사해서 사용
            local_color_image = self.color_image
            local_depth_image = self.depth_image

        if local_color_image is None or local_depth_image is None:
            self.get_logger().info("아직 카메라 이미지를 수신하지 못했습니다...", throttle_duration_sec=2)
            return

        # --- 이제 local_color_image 와 local_depth_image 를 가지고 ---
        # --- 기존의 YOLO, OCR, 장애물 감지 등의 로직을 수행하면 됩니다 ---
        # 예: detected_objects = self.model_detector.detect_objects(local_color_image, local_depth_image, ...)
        # ...