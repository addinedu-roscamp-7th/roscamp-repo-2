import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import cv2
import numpy as np

# Haar Cascade XML 파일 경로 설정
FACE_CASCADE_PATH = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'

# 30프레임 이상 추적 실패 시 재탐지 시도
LOST_FRAMES_THRESHOLD = 30 

# 추적 실패 후 로봇 정지 유지 시간
STOP_TIMEOUT_SECONDS = 10.0

# OpenCV Haar Cascade로 얼굴을 감지하고, CSRT Tracker로 추적하는 ROS2 Node
class DetectTrackingNode(Node):

    def __init__(self):
        super().__init__('detect_tracking_node')
        self.get_logger().info('OpenCV CSRT Person Tracking Node Initializing...')

        # --- 1. ROS2 설정 ---
        self.pose_publisher = self.create_publisher(Pose2D, 'tracking_person_pose', 10)
        self.timer_period = 1.0 / 30.0  # 30 FPS 처리 주기  
        self.timer = self.create_timer(self.timer_period, self.process_frame_callback)
        self.stop_timer = None          # 로봇 정지 시간 타이머(10s)

        # --- 2. OpenCV 설정 ---
        self.face_cascade = cv2.CascadeClassifier(FACE_CASCADE_PATH)
        if self.face_cascade.empty():
            self.get_logger().error(f"Haar Cascade 파일을 로드할 수 없습니다. 경로를 확인하세요: {FACE_CASCADE_PATH}")
            self.destroy_node()
            return
            
        # CSRT 트래커는 추적 시작 시 초기화    
        self.tracker = None  
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("WebCam을 열 수 없습니다. 카메라 인덱스(0) 또는 연결을 확인하십시오.")
            self.destroy_node() 
            return

        # --- 3. 추적 상태 변수 ---
        self.tracking = False
        self.lost_frames = 0
        self.is_robot_moving = True # 로봇 초기 상태는 '이동 중'
        
        self.get_logger().info('OpenCV CSRT Person Tracking Node Started successfully.')
    
    # 로봇 정지 후 10초가 지나면 호출
    def stop_timeout_callback(self):
        self.get_logger().warn(f"Please Come Back in ({STOP_TIMEOUT_SECONDS}s), There's No Person Detected!")

        # 10초 후에도 돌아오지 않으면, 타이머 중단
        if self.stop_timer:
            self.stop_timer.destroy()
            self.stop_timer = None

    # 여기서 로봇을 '다시 동작' 상태로 전환시키거나, 다른 기본 업무를 수행할 수 있도록 Logic 확장
    # 현재는 Pose2D msg 발행을 중단하는 것 외 추가 제어는 없음

    # 추적 실패 시, 카운트를 증가시키고 재탐지 필요 여부를 확인
    def handle_tracking_loss(self):
        self.lost_frames += 1

        if self.lost_frames > LOST_FRAMES_THRESHOLD:
            self.tracking = False
            self.tracker = None
            self.lost_frames = 0
            self.get_logger().info("Tracking Lost, Detecting Person...")

            # 로봇 제어 Logic(추적을 잃으면 로봇이 정지)
            if self.is_robot_moving:
                self.is_robot_moving = False
                self.pose_publisher.publish(Pose2D()) # 없는 msg 발행으로 정지 신호 (수신 Node에서 처리 필요)
                self.get_logger().warn(f"Dobby Stopped, Starting {STOP_TIMEOUT_SECONDS}s TIMEOUT!")

                # 10초 타이머 시작 (10초 동안 사람이 돌아오지 않으면 호출)
                if self.stop_timer:
                    self.stop_timer.destroy()
                self.stop_timer = self.create_timer(STOP_TIMEOUT_SECONDS, self.stop_timeout_callback)

    # 로봇을 이동 상태로 복귀
    def resume_robot_moving(self, reason="Tracking Resumed"):
        if not self.is_robot_moving:
            self.is_robot_moving = True
            self.get_logger().info(f"Robot Starts Moving, Reason: {reason}")

        # 정지 타이머가 실행 중이었다면, 즉시 취소
        if self.stop_timer:
            self.stop_timer.destroy()
            self.stop_timer = None

    def process_frame_callback(self):
        """ 타이머에 의해 주기적으로 호출되는 콜백 함수: 프레임 읽기 및 처리 """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("WebCam에서 Frame을 읽을 수 없습니다!") 
            return

        frame_height, frame_width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.tracking and self.tracker is not None:
            # --- 1. 추적 업데이트 ---
            success, bbox = self.tracker.update(frame)
            
            if success:
                self.update_tracking(frame, frame_width, frame_height, bbox)
            else:
                self.handle_tracking_loss()
                
                # 추적 이탈/실패 확인
                if self.lost_frames > LOST_FRAMES_THRESHOLD:
                    self.tracking = False
                    self.tracker = None
                    self.lost_frames = 0 
                    self.get_logger().info("Tracking Lost. Re-detecting...")

        else:
            # --- 2. 재감지 시작 (가장 가까운 사람 = 가장 큰 얼굴) ---
            self.attempt_detection(frame, gray)
        
        cv2.imshow('OpenCV_CSRT_Tracking', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.destroy_node()

    def update_tracking(self, frame, frame_width, frame_height, bbox):
        """ 추적 중일 때 트래커를 업데이트하고 결과를 처리합니다. """
        x, y, w, h = [int(v) for v in bbox]
        
        # 사람이 돌아왔으므로, 로봇 이동 재개
        self.resume_robot_moving(reason="Tracking Activated")

        # 추적 결과 그리기
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "Visitor", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        self.lost_frames = 0
        
        # 추적 결과 Pose2D 메시지로 변환 및 발행
        center_x = x + w / 2.0
        center_y = y + h / 2.0
        
        pose_msg = Pose2D()
        pose_msg.x = center_x - frame_width / 2.0       # 화면 중앙 기준 X 좌표
        pose_msg.y = center_y - frame_height / 2.0      # 화면 중앙 기준 Y 좌표
        pose_msg.theta = float(w)                       # 감지된 객체의 크기 (가까울수록 커짐)
        
        self.pose_publisher.publish(pose_msg)

    def attempt_detection(self, frame, gray):
        """ 추적에 실패했을 때 얼굴을 다시 감지하고 추적을 시작합니다. """
        
        # Haar Cascade 감지
        faces = self.face_cascade.detectMultiScale(
            gray, 
            scaleFactor=1.05, # 이미지 축소 비율
            minNeighbors=8,  # 최소 이웃 수
            minSize=(80, 80) # 최소 얼굴 크기
        )

        if len(faces) > 0:
            # 가장 큰 얼굴 (가장 가까운 사람) 찾기
            max_area = 0
            best_bbox = None
            for (x, y, w, h) in faces:
                area = w * h
                if area > max_area:
                    max_area = area
                    best_bbox = (x, y, w, h)
            
            if best_bbox:
                # CSRT 트래커 재설정 및 추적 시작
                self.tracker = cv2.TrackerCSRT_create()
                self.tracker.init(frame, best_bbox)
                self.tracking = True
                self.get_logger().info(f"Detection Success. Starting CSRT tracking.")

                # 감지된 얼굴에 사각형 그리기
                x, y, w, h = [int(v) for v in best_bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, "Visitor", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
    
    def destroy_node(self):
        """ 노드 종료 시 자원을 해제합니다. """
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