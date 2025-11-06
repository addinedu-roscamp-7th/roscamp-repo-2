import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import cv2
import numpy as np

# Haar Cascade XML 파일 경로 설정 (사용자 환경에 맞게 반드시 수정하세요!)
FACE_CASCADE_PATH = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'

# 추적 안정성 및 로봇 제어 변수
LOST_FRAMES_THRESHOLD = 30     # 30프레임 이상 추적 실패 시 재탐지 시도 (약 1초)
STOP_TIMEOUT_SECONDS = 10.0    # 추적 실패 후 로봇 정지 유지 시간

class DetectTrackingNode(Node):
    """
    OpenCV Haar Cascade & CSRT를 사용하여 가장 가까운 Visitor를 추적하고,
    추적 상태에 따라 로봇의 정지/이동을 제어합니다.
    """
    def __init__(self):
        super().__init__('detect_tracking_node')
        self.get_logger().info('OpenCV CSRT Visitor Tracking Node Initializing...')

        # --- 1. ROS 2 설정 ---
        self.pose_publisher = self.create_publisher(Pose2D, 'tracking_person_pose', 10)
        self.timer_period = 1.0 / 30.0  # 30 FPS 처리 주기  
        self.timer = self.create_timer(self.timer_period, self.process_frame_callback)
        self.stop_timer = None          # 로봇 정지 시간 타이머

        # --- 2. OpenCV 설정 ---
        self.face_cascade = cv2.CascadeClassifier(FACE_CASCADE_PATH)
        if self.face_cascade.empty():
            self.get_logger().error(f"Haar Cascade 파일을 로드할 수 없습니다. 경로를 확인하세요: {FACE_CASCADE_PATH}")
            self.destroy_node()
            return
            
        self.tracker = None  # CSRT 트래커 인스턴스
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("WebCam을 열 수 없습니다.")
            self.destroy_node() 
            return

        # --- 3. 상태 변수 ---
        self.tracking = False
        self.lost_frames = 0
        self.is_robot_moving = True 
        
        self.get_logger().info('OpenCV CSRT Visitor Tracking Node Started successfully.')

    def stop_timeout_callback(self):
        """ 로봇 정지 후 10초가 지나면 호출됩니다. (사람이 돌아오지 않음) """
        self.get_logger().error(f"Stop timeout reached ({STOP_TIMEOUT_SECONDS}s). Visitor not returned.")
        if self.stop_timer:
            self.stop_timer.destroy()
            self.stop_timer = None

    def resume_robot_movement(self, reason="Tracking Resumed"):
        """ 로봇을 이동 상태로 복귀시키고 정지 타이머를 취소합니다. """
        if not self.is_robot_moving:
            self.is_robot_moving = True
            self.get_logger().info(f"Robot Starts Moving, Reason: {reason}")
            if self.stop_timer:
                self.stop_timer.destroy()
                self.stop_timer = None
            
    def handle_tracking_loss(self):
        """ 추적 실패 확정 시 로봇 정지 및 10초 카운트 로직을 실행합니다. """
        self.lost_frames += 1

        if self.lost_frames > LOST_FRAMES_THRESHOLD:
            # 추적 실패 확정
            self.tracking = False
            self.tracker = None
            self.lost_frames = 0
            self.get_logger().info("Tracking Lost. Attempting Re-Detection...")

            # 로봇 제어 로직: 추적을 완전히 잃으면 로봇 정지 및 10초 카운트 시작
            if self.is_robot_moving:
                self.is_robot_moving = False
                # 로봇 정지 신호 발행 (수신 노드에서 Pose2D(0,0,0)을 정지 신호로 처리해야 함)
                self.pose_publisher.publish(Pose2D()) 
                self.get_logger().warn(f"Dobby Stopped. Starting {STOP_TIMEOUT_SECONDS}s TIMEOUT!")

                if self.stop_timer:
                    self.stop_timer.destroy()
                self.stop_timer = self.create_timer(STOP_TIMEOUT_SECONDS, self.stop_timeout_callback)

    def is_bbox_out_of_frame(self, frame_width, frame_height, bbox):
        """ 바운딩 박스가 화면 경계를 벗어났거나 신뢰도가 낮아졌는지 확인합니다. """
        x, y, w, h = [int(v) for v in bbox]
        
        # 1. 크기가 너무 작아지면 신뢰도 낮음으로 판단 (추적 실패 기준 강화)
        if w < 40 or h < 40: 
            self.get_logger().debug("BBox too small, confidence low.")
            return True
        
        # 2. 박스의 절반 이상이 화면 경계를 벗어났는지 확인 (화면 이탈 감지 강화)
        if x + w * 0.7 < 0 or x + w * 0.7 > frame_width or \
           y + h * 0.7 < 0 or y + h * 0.7 > frame_height:
            self.get_logger().debug("BBox center outside frame.")
            return True
            
        return False

    def process_frame_callback(self):
        """ 주기적으로 호출: 프레임 읽기 및 추적/감지 수행 """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("WebCam에서 Frame을 읽을 수 없습니다!") 
            return

        frame_height, frame_width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.tracking and self.tracker is not None:
            # 추적 중인 경우: 트래커 업데이트
            success, bbox = self.tracker.update(frame)
            
            # 트래커 성공 여부와 화면 이탈 여부 동시 확인
            if success and not self.is_bbox_out_of_frame(frame_width, frame_height, bbox):
                self.update_tracking(frame, frame_width, frame_height, bbox)
            else:
                # 트래커 실패, 화면 이탈, 또는 박스 크기가 너무 작아짐 -> 추적 실패 처리
                self.handle_tracking_loss() # 이 함수에서 lost_frames를 증가시키고 30프레임 후 정지 로직 실행

        else:
            # 추적 실패 또는 미시작: 가장 가까운 사람 재감지 시도
            self.attempt_detection(frame, gray)
        
        cv2.imshow('Visitor_Tracking', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.destroy_node()

    def update_tracking(self, frame, frame_width, frame_height, bbox):
        """ 추적 성공 시 화면 표시 및 로봇 이동 명령 발행. """
        # 사람이 돌아왔으므로 로봇 이동 재개 (10초 타이머 자동 취소)
        self.resume_robot_movement(reason="Tracking active")

        x, y, w, h = [int(v) for v in bbox]
        
        # 화면 표시
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "Visitor (Tracking)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        self.lost_frames = 0
        
        # 로봇이 이동 중일 때만 Pose2D 메시지 발행
        if self.is_robot_moving:
            center_x = x + w / 2.0
            center_y = y + h / 2.0
            
            pose_msg = Pose2D()
            pose_msg.x = center_x - frame_width / 2.0
            pose_msg.y = center_y - frame_height / 2.0
            pose_msg.theta = float(w)
            
            self.pose_publisher.publish(pose_msg)

    def attempt_detection(self, frame, gray):
        """ Haar Cascade를 사용하여 가장 가까운 얼굴을 감지하고 추적을 시작합니다. """
        
        # 인식 정밀도 강화를 위한 파라미터 튜닝
        faces = self.face_cascade.detectMultiScale(
            gray, 
            scaleFactor=1.05, # 감지 정밀도 증가
            minNeighbors=10,  # 오탐(False Positive)을 극도로 줄임
            minSize=(100, 100) # 최소 크기를 높여 오인식 방지
        )

        if len(faces) > 0:
            # 1. 가장 큰 얼굴 (가장 가까운 사람) 찾기
            max_area = 0
            best_bbox = None
            for (x, y, w, h) in faces:
                area = w * h
                if area > max_area:
                    max_area = area
                    best_bbox = (x, y, w, h)
            
            if best_bbox:
                # 2. CSRT 트래커 초기화 및 추적 시작
                self.tracker = cv2.TrackerCSRT_create()
                self.tracker.init(frame, best_bbox)
                self.tracking = True
                self.get_logger().info("Detection Success. Starting CSRT tracking of largest face (Visitor).")

                # 로봇 제어 로직: 감지 성공 시 로봇 재가동
                self.resume_robot_movement(reason="Re-detection success")

                # 화면 표시
                x, y, w, h = [int(v) for v in best_bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, "Visitor Detected!", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
    
    def destroy_node(self):
        """ 노드 종료 시 자원을 해제합니다. """
        if self.stop_timer:
            self.stop_timer.destroy()
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """ ROS 2 엔트리 포인트 """
    rclpy.init(args=args)
    node = DetectTrackingNode()
    
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()