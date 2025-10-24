#!/usr/bin/env python3
import rclpy as rp
from rclpy.node import Node

from javis_interfaces.srv import ChangeTrackingMode


class TrackModeChangeServer(Node):

    def __init__(self):
        super().__init__('track_mode_change_server')
        service_name = '/dobby1/ai/change_tracking_mode'
        self.server = self.create_service(ChangeTrackingMode, service_name, self.change_track_mode_callback)
        self.get_logger().warn(f"~~~~~ RUN '{service_name}' Server Node ~~~~~")

    def change_track_mode_callback(self, request, response):
        self.get_logger().info('~~~~~ Client Request Detect ~~~~~')
        if request.mode == 0:
            response.success = True
            response.message = '@@@@@ Change mode REGISTRATION @@@@@'
        elif request.mode == 1:
            response.success = False
            response.message = '!!!!! Failed Change mode !!!!!'
            self.get_logger().fatal('!!!!! Failed Change mode !!!!!')
        else:
            response.success = True
            response.message = '##### Change mode IDLE #####'
        
        return response



def main(args=None):
    rp.init(args=args)
    track_mode_server = TrackModeChangeServer()
    rp.spin(track_mode_server)
    track_mode_server.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
