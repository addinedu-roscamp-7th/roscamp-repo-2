#!/usr/bin/env python3
import json
from flask import Flask, request, jsonify
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from javis_interfaces.srv import MyJson

app = Flask(__name__)
CORS(app)

app.config['JSON_AS_ASCII'] = False

# --- ROS2 초기화 (모듈 로드시 1회) ---
rclpy.init()
_ros_node = Node('http_ros_gateway')
_client = _ros_node.create_client(MyJson, 'tcp_test')

def wait_for_service():
    _ros_node.get_logger().info('Waiting for /tcp_test service...')
    while not _client.wait_for_service(timeout_sec=1.0):
        _ros_node.get_logger().info('...still waiting')
    _ros_node.get_logger().info('Service available.')

@app.route('/robot/pickup', methods=['POST'])
def process():
    try:
        payload_dict = request.get_json(force=True, silent=False)
        payload_str = json.dumps(payload_dict, ensure_ascii=False)
    except Exception as e:
        return jsonify({'ok': False, 'message': f'Bad JSON: {e}'}), 400

    req = MyJson.Request()
    req.payload = payload_str
    future = _client.call_async(req)

    while not future.done():
        rclpy.spin_once(_ros_node, timeout_sec=0.1)

    res = future.result()
    response_data = {'ok': bool(res.ok), 'message': res.message}
    try:
        # res.message가 JSON 형식의 문자열이라고 가정하고 파싱
        message_dict = json.loads(res.message)
        task_name = message_dict.get('task_name', 'N/A')
        _ros_node.get_logger().info(f'task_name from response: {task_name}')
        if task_name == "도서 픽업":
            response_data = {"message":"작업생성이 완료됐습니다."}
        else:
            response_data = {"message":"작업생성이 실패했습니다."}
    except (json.JSONDecodeError, AttributeError):
        _ros_node.get_logger().info(f'Could not parse task_name from response. Full message: {res.message}')
   
    json_data = json.dumps(response_data, ensure_ascii=False)
    return app.response_class(response=json_data, status=200, mimetype='application/json; charset=utf-8')


def main():
    """ros2 run이 호출하는 진입점"""
    wait_for_service()
    # 필요하면 host/port를 매개변수/환경변수로 빼도 됩니다.
    app.run(host='0.0.0.0', port=8001)

if __name__ == '__main__':
    main()
