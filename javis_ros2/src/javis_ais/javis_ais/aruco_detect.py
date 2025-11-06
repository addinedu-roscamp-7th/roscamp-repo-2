import rclpy
import cv2
import cv2.aruco as aruco
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# Pose 메시지 및 쿼터니언 변환을 위한 라이브러리
from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation as R
from javis_interfaces.msg import ArucoDockingData

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/dobbycam/image_raw',
            self.image_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/dobbycam/camera_info',
            self.info_callback,
            10
        )
        # rviz2에서 아래 publish 내용 확인 가능 (fixed frame 'camera'로 변경)
        self.pose_pub = self.create_publisher(
            PoseArray, 
            '/aruco_poses',
            10
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            '/aruco_debug_image',
            10
        )

        self.aruco_docking_pub = self.create_publisher(
            ArucoDockingData,
            '/aruco_docking_data',
            10
        )

        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.05
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 팝업창(Preview) 표시가 필요할 경우 아래 주석 해제
        # self.preview_window_name = "Dobby Aruco Preview"
        # cv2.namedWindow(self.preview_window_name, cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info("ArUco Detector (Pose Publishing) a-node started.")


    def info_callback(self, msg):
        if self.camera_matrix is None:
            # Segfault 방지를 위해 numpy 타입 명시
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            # dist_coeffs가 비어있는 경우(왜곡 없음) 처리
            if msg.d and len(msg.d) > 0:
                self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            else:
                self.get_logger().warn('Camera distortion coefficients (D) are empty. Assuming zero distortion.')
                self.dist_coeffs = np.zeros(5, dtype=np.float32)
                
            self.get_logger().info('Camera info received and set.')
            # CameraInfo는 한 번만 받으면 되므로 구독 중지
            self.destroy_subscription(self.info_sub)

    def image_callback(self, msg):
        # dist_coeffs도 함께 확인
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn("Waiting for camera info...", once=True)
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        corners, ids, _ = self.detector.detectMarkers(cv_image) # 'rejected'를 '_'로 변경

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            # 발행할 PoseArray 메시지 초기화
            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = msg.header.stamp
            pose_array_msg.header.frame_id = msg.header.frame_id # 카메라 프레임
            
            pose_list = [] # Pose 메시지들을 담을 Python 리스트

            for i in range(len(ids)):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # --- rvec, tvec을 ROS Pose 메시지로 변환 ---
                pose_msg = Pose()

                # 1. Position (tvec과 동일)
                pose_msg.position.x = tvec[0]
                pose_msg.position.y = tvec[1]
                pose_msg.position.z = tvec[2]

                # 2. Orientation (rvec -> 쿼터니언 변환)
                rotation = R.from_rotvec(rvec)
                quat = rotation.as_quat() # (x, y, z, w) 순서

                pose_msg.orientation.x = quat[0]
                pose_msg.orientation.y = quat[1]
                pose_msg.orientation.z = quat[2]
                pose_msg.orientation.w = quat[3]
                
                # 변환된 Pose 메시지를 리스트에 추가
                pose_list.append(pose_msg)

                # 디버그 이미지에 축 그리기
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

            # Python 리스트를 PoseArray 메시지의 'poses' 필드에 할당
            pose_array_msg.poses = pose_list
            
            # 완성된 PoseArray 메시지 발행
            self.pose_pub.publish(pose_array_msg)

            # pose_array_msg.poses는 리스트이므로, for loop로
            self.get_logger().info('===== Publishing ArUco Poses =====')
            
            for i, pose_msg in enumerate(pose_array_msg.poses):
                # pose_msg는 리스트 안의 개별 Pose 메시지입니다.
                # ids는 [[10], [20]] 같은 형식이므로 ids[i][0]으로 ID를 가져옵니다.
                marker_id = ids[i][0]

                # marker id < 10은 버림
                if marker_id < 10:
                    self.get_logger().info(f"Ignoring marker ID: {marker_id} (using 10+)")
                    continue  # 현재 반복을 중단하고 다음 마커로 넘어갑니다.

                # [추가] 로그를 찍기 위해 Pose 메시지의 쿼터니언으로부터 Euler 각도 계산
                quat_from_msg = [
                    pose_msg.orientation.x,
                    pose_msg.orientation.y,
                    pose_msg.orientation.z,
                    pose_msg.orientation.w
                ]
                rotation = R.from_quat(quat_from_msg)

                # OpenCV/Aruco의 표준 축 순서 'xyz' (Roll, Pitch, Yaw) 사용
                euler_angles = rotation.as_euler('xyz', degrees=True)
                roll_deg = euler_angles[0]
                pitch_deg = euler_angles[1]
                yaw_deg = euler_angles[2]
                
                self.get_logger().info(f'--- Marker ID: {marker_id} ---')
                # 위치 (Position)
                self.get_logger().info(f'  pos_x (좌우): {pose_msg.position.x:.3f} m')
                self.get_logger().info(f'  pos_y (상하): {pose_msg.position.y:.3f} m')
                self.get_logger().info(f'  pos_z (거리): {pose_msg.position.z:.3f} m')
                # 회전 (Euler Angles)
                self.get_logger().info(f'  >> Roll (끄덕임) [X]: {roll_deg:.2f} 도')
                self.get_logger().info(f'  >> Pitch (갸웃함) [Y]: {pitch_deg:.2f} 도')
                self.get_logger().info(f'  >> YAW (틀어짐) [Z]: {yaw_deg:.2f} 도')

                marker_pose_msg = ArucoDockingData()
                marker_pose_msg.marker_pos_z = pose_msg.position.z
                marker_pose_msg.marker_yaw = yaw_deg
                self.get_logger().info(f'Send DDC (ID: {marker_id}) | z: {marker_pose_msg.marker_pos_z:.3f}, yaw: {marker_pose_msg.marker_yaw:.3f}')
                self.aruco_docking_pub.publish(marker_pose_msg)

            self.get_logger().info('=====================================')


        # Rviz/Rqt용 디버그 이미지 발행
        if self.debug_image_pub.get_subscription_count() > 0:
            debug_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            debug_image_msg.header = msg.header
            debug_image_msg.header.frame_id = msg.header.frame_id
            self.debug_image_pub.publish(debug_image_msg)

        # 팝업창(Preview) 표시가 필요할 경우 아래 주석 해제
        # cv2.imshow(self.preview_window_name, cv_image)
        # cv2.waitKey(1)
            

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()