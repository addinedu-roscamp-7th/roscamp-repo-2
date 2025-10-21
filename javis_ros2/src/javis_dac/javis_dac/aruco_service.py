#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import cv2
import numpy as np

class ArucoServiceNode(Node):
    def __init__(self):
        super().__init__('aruco_service_node')

        self.srv = self.create_service(Trigger, 'detect_aruco', self.detect_callback)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('❌ Camera not found!')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        self.K = np.array([[880.0, 0.0, 640.0],
                           [0.0, 880.0, 360.0],
                           [0.0,   0.0,   1.0]])
        self.dist = np.zeros((5, 1))
        self.marker_size = 0.03  # 30mm

        self.get_logger().info('📷 ArUco Detection Service Ready (javis_dac)!')

    def detect_callback(self, request, response):
        ret, frame = self.cap.read()
        if not ret:
            response.success = False
            response.message = "Camera read failed"
            return response

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.K, self.dist
            )
            id_list = ids.flatten().tolist()
            tvec_list = [t.tolist() for t in tvecs.reshape(-1, 3)]

            self.get_logger().info(f'✅ Detected: IDs={id_list}, tvecs={tvec_list}')
            response.success = True
            response.message = str({'ids': id_list, 'tvecs': tvec_list})
        else:
            self.get_logger().info('❌ No marker detected.')
            response.success = False
            response.message = 'No marker detected'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArucoServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
