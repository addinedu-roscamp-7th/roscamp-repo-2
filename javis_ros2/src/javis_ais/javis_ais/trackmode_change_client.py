#!/usr/bin/env python3
import rclpy as rp
from rclpy.node import Node

# javis_ais_msgs package 내에 .srv 파일이 있기 때문에
# from javis_ais_msgs.srv import ChangeTrackingMode
from javis_interfaces.srv import ChangeTrackingMode


class TrackModeChangeClient(Node):
    def __init__(self):
        super().__init__('track_mode_change_client')

        service_name = '/dobby1/ai/change_tracking_mode'
        self.client = self.create_client(ChangeTrackingMode, service_name)
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"'{service_name}' service waiting...")
            # rp.spin_once(self, timeout_sec=1.0)

        self.get_logger().info(f"'{service_name}' service is ready")

        # request message 생성
        self.req = ChangeTrackingMode.Request()

    # Server request 항목 전송하는 함수
    def send_request(self, mode, tracking_id):
        self.req.mode = mode
        self.req.tracking_id = tracking_id
        self.future = self.client.call_async(self.req)
        return self.future
    
def main(args=None):
    rp.init(args=args)

    track_mode_change_client = TrackModeChangeClient()
    future = track_mode_change_client.send_request(0, 'person_id') # 요청

    while not future.done():
        rp.spin_once(track_mode_change_client, timeout_sec=1.0)
    
    try:
        response = future.result()
        track_mode_change_client.get_logger().info(
                f'===== Response =====\nsuccess: {response.success}\nmessage: "{response.message}"')
    except Exception as e:
        track_mode_change_client.get_logger().warn('Service 요청 실패 : {e}')


    track_mode_change_client.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
