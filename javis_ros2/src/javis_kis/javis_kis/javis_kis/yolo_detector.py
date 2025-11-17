import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading # 스레드 간의 데이터 공유를 안전하게 하기 위해 추가

# 메시지 동기화를 위한 message_filters 임포트
import message_filters

# PointCloud2 데이터를 다루기 위한 sensor_msgs_py.point_cloud2 임포트
from sensor_msgs_py import point_cloud2

# Ultralytics YOLOv8 라이브러리 임포트
from ultralytics import YOLO

# ★★★ 1. 생성한 서비스 인터페이스 임포트 ★★★
# 'your_package_name'은 실제 패키지 이름으로 변경해야 합니다.
from javis_interfaces.srv import GetCoordinates


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # weights_path = '/home/addinedu/roscamp-repo-2/javis_ros2/best.pt'
        weights_path = '/home/addinedu/roscamp-repo-2/javis_ros2/best.pt'
        # YOLOv8 모델 로드
        self.get_logger().info(f"YOLOv8 모델 로드 중: {weights_path}")
        try:
            self.yolo_model = YOLO(weights_path)
            self.get_logger().info("YOLOv8 모델 로드 완료.")
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            raise e
            
        # CvBridge 초기화
        self.bridge = CvBridge()
        


        self.last_detected_coords = None
        self.lock = threading.Lock()

        # 서비스 서버 생성
        self.srv = self.create_service(
            GetCoordinates, 
            '/get_coordinates', 
            self.get_coordinates_callback
        )
        self.get_logger().info("'/get_coordinates' 서비스 서버가 준비되었습니다.")

        # Subscriber를 message_filters를 사용하도록 설정
        self.image_sub = message_filters.Subscriber(self, Image, '/oak/rgb/image_rect')
        self.pointcloud_sub = message_filters.Subscriber(self, PointCloud2, '/oak/points')
        
        # ApproximateTimeSynchronizer로 두 토픽을 동기화
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pointcloud_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.time_synchronizer.registerCallback(self.synced_callback)
        
        # 객체 인식 결과 이미지 발행
        self.publisher_ = self.create_publisher(Image, '/yolo_detections_image', 10)
        
        self.get_logger().info('YOLO Detector 노드 시작. 이미지와 포인트 클라우드 토픽 동기화 대기 중...')

    # 노드 종료 메서드에서 소켓 관련 코드 삭제
    def destroy_node(self):
        self.get_logger().info("노드 종료.")
        super().destroy_node()

    # 서비스 요청이 왔을 때 호출될 콜백 함수
    def get_coordinates_callback(self, request, response):
        with self.lock:
            if self.last_detected_coords:
                response.x = self.last_detected_coords['x'] / 1.0
                response.y = self.last_detected_coords['y'] / 1.0
                response.z = self.last_detected_coords['z'] / 1.0

            else:
                response.x = 0.0
                response.y = 0.0
                response.z = 0.0
                self.get_logger().warn("서비스 요청이 있었으나, 최근 감지된 사과가 없습니다.")
        return response

    def synced_callback(self, image_msg, pc_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        results = self.yolo_model(cv_image, conf=0.85, verbose=False) 
        annotated_img = results[0].plot()

        image_height, image_width, _ = cv_image.shape
        pc_height = pc_msg.height
        pc_width = pc_msg.width

        if image_width == 0 or image_height == 0 or pc_width == 0 or pc_height == 0:
            return

        width_ratio = pc_width / image_width
        height_ratio = pc_height / image_height
        
        try:
            points_array = np.frombuffer(pc_msg.data, dtype=np.float32)
            points_array = points_array.reshape((pc_height, pc_width, -1))
        except Exception as e:
            self.get_logger().error(f'Point Cloud 변환 실패: {e}')
            return
        
        found_object_in_frame = False
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            center_u = (x1 + x2) // 2
            center_v = (y1 + y2) // 2
            
            scaled_u = int(center_u * width_ratio)
            scaled_v = int(center_v * height_ratio)

            if not (0 <= scaled_u < pc_width and 0 <= scaled_v < pc_height):
                continue

            try:
                x = points_array[scaled_v, scaled_u, 0]
                y = points_array[scaled_v, scaled_u, 1]
                z = points_array[scaled_v, scaled_u, 2]
                
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    continue
                
                class_name = self.yolo_model.names[int(box.cls)]
                self.get_logger().info(
                    f"'{class_name}' 탐지됨 - 중심 좌표: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m"
                )

                # 감지된 좌표를 클래스 변수에 저장
                with self.lock:
                    self.last_detected_coords = {'x': x, 'y': y, 'z': z}
                
                found_object_in_frame = True

                coord_text = f"Z: {z:.2f}m"
                cv2.putText(annotated_img, coord_text, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.circle(annotated_img, (center_u, center_v), 5, (0, 0, 255), -1)

                break
                
            except Exception as e:
                self.get_logger().error(f'3D 좌표 추출 실패: {e}')
                continue
        
        # 프레임에서 객체를 찾지 못했다면 저장된 좌표 초기화
        if not found_object_in_frame:
            with self.lock:
                self.last_detected_coords = None

        try:
            ros_image = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = image_msg.header.frame_id
            self.publisher_.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'결과 이미지 발행 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()