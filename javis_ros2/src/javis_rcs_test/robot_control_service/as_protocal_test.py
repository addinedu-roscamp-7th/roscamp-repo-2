#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from javis_interfaces.srv import MyJson  # 자동 생성된 py 모듈

class JsonService(Node):
    def __init__(self):
        super().__init__('json_service')
        self.srv = self.create_service(MyJson, 'tcp_test', self.cb)
        self.get_logger().info('Service [/tcp_test] ready.')

    def cb(self, request, response):
        # request.payload 에 JSON 문자열이 옴
        try:
            data = json.loads(request.payload)  # 검증
        except json.JSONDecodeError as e:
            response.ok = False
            response.message = f'Invalid JSON: {e}'
            return response

        # ---- 여기서 원하는 로직 수행 ----
        # 예: 키 확인, 값 가공, ROS 토픽 퍼블리시/액션 호출 등
        taks_name = data.get('task_name', 'unknown')
        book_info = data.get('book_info', 'unknown')
        location = data.get('location', 'unknown')
        storage_info = data.get('storage_info', 'unknown')
        self.get_logger().info(f'task: {taks_name}, book: {book_info}, location:{location}, storage: {storage_info}')        

        # 데모 응답
        response.ok = True
        response.message = f"task: {taks_name}, book: {book_info}, location:{location}, storage: {storage_info}"
        return response

def main():
    rclpy.init()
    node = JsonService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
