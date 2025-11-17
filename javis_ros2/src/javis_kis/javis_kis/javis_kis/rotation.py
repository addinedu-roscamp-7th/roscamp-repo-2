import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class RotationDetectorNode(Node):
    def __init__(self):
        super().__init__('rotation_detector')
        
        # 최종 처리된 이미지 퍼블리셔
        self.image_publisher = self.create_publisher(Image, 'rotation_image', 10)
        # 마스크 이미지 퍼블리셔
        self.mask_publisher = self.create_publisher(Image, 'rotation_mask_image', 10)
        # 각도 값 퍼블리셔
        self.angle_publisher = self.create_publisher(Float64, 'rotation_angle', 10)
        
        # 0.033초마다 (약 30Hz) timer_callback 함수를 실행하는 타이머 설정
        timer_period = 0.033
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
            rclpy.shutdown()
            
        # OpenCV 이미지와 ROS 메시지를 변환하기 위한 CvBridge 초기화
        self.bridge = CvBridge()
        
        self.get_logger().info("Rotation Detector 노드가 시작되었습니다.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn("프레임을 수신할 수 없습니다.")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_color = np.array([90, 49, 93])
        upper_color = np.array([180, 90, 172])

        # 정의한 HSV 범위에 해당하는 영역을 마스크로 만듦
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # 내부 노이즈(구멍) 메우기
        kernel = np.ones((7, 7), np.uint8)
        mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 노이즈가 메꿔진 마스크에서 컨투어를 찾음
        contours, _ = cv2.findContours(mask_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 객체가 감지되지 않았을 경우를 대비한 기본 각도 값
        current_angle = 0.0

        # 찾은 컨투어들을 순회
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 1000:
                rect = cv2.minAreaRect(largest_contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                
                angle = rect[2]
                rotation_angle = abs(angle)
                
                current_angle = rotation_angle
                angle_text = f"Angle: {rotation_angle:.2f}"
                text_pos = (int(rect[0][0]), int(rect[0][1]))
                cv2.putText(frame, angle_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)


        try:
            # 1. 최종 처리된 이미지 퍼블리시
            # OpenCV 이미지(numpy.ndarray)를 ROS Image 메시지로 변환
            processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_publisher.publish(processed_image_msg)
            
            # 2. 마스크 이미지 퍼블리시
            # 마스크는 단일 채널(grayscale) 이미지이므로 "mono8"로 인코딩
            mask_image_msg = self.bridge.cv2_to_imgmsg(mask_closed, "mono8")
            self.mask_publisher.publish(mask_image_msg)

            # 3. 각도 값 퍼블리시
            angle_msg = Float64()
            angle_msg.data = current_angle
            self.angle_publisher.publish(angle_msg)

        except Exception as e:
            self.get_logger().error(f"퍼블리시 중 에러 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    rotation_detector_node = RotationDetectorNode()
    
    try:
        rclpy.spin(rotation_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료 시 카메라 자원 해제
        rotation_detector_node.cap.release()
        rotation_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()