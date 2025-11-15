#!/usr/bin/env python3
import json
from flask import Flask, request, jsonify
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from javis_interfaces.srv import MyJson
from geometry_msgs.msg import Pose, Pose2D

app = Flask(__name__)
CORS(app)

app.config['JSON_AS_ASCII'] = False

# --- ROS2 초기화 (모듈 로드시 1회) ---
rclpy.init()
_ros_node = Node('http_ros_gateway') # 게이트웨이 노드 이름
_client = _ros_node.create_client(MyJson, 'robot_task') # OrchestratorNode의 서비스 이름으로 변경


def wait_for_service():
    _ros_node.get_logger().info('Waiting for /robot_task service...')
    while not _client.wait_for_service(timeout_sec=1.0):
        _ros_node.get_logger().info('...still waiting')
    _ros_node.get_logger().info('Service available.')

def _send_task_to_orchestrator(task_data):
    """주어진 task_data를 OrchestratorNode의 robot_task 서비스로 전송합니다."""
    try:
        payload_str = json.dumps(task_data, ensure_ascii=False)
    except Exception as e:
        _ros_node.get_logger().error(f"Failed to serialize task_data to JSON: {e}")
        return {'ok': False, 'message': f'Internal server error: {e}'}

    req = MyJson.Request()
    req.payload = payload_str
    future = _client.call_async(req)

    while not future.done():
        rclpy.spin_once(_ros_node, timeout_sec=0.1)

    res = future.result() # MyJson.Response 객체
    return {'ok': bool(res.ok), 'message': res.message}

@app.route('/robot/pickup', methods=['POST'])
def handle_pickup_request():
  
    try:
        req_data = request.get_json(force=True, silent=False)
        if not req_data:
            return jsonify({'ok': False, 'message': 'Request body must be JSON.'}), 400

        book_id = req_data.get('book_id')
        if book_id is None: 
            return jsonify({'ok': False, 'message': 'Missing required field: book'}), 400
        
        shelf_approach_location = req_data.get('shelf_approach_location')
        if shelf_approach_location is None: 
            return jsonify({'ok': False, 'message': 'Missing required field: shelf_approach_location'}), 400
        
        _ros_node.get_logger().info(f"Received /robot/pickupbook request: {req_data}")
        
        


        # clean_seat에 필요한 다른 파라미터들은 req_data에서 직접 가져와 task_data에 포함
        # OrchestratorNode의 execute_task에서 기본값 처리가 되므로, 없으면 안 보내도 됨
        task_data = {
            "task_name": "pickup_book",
            "book_id": book_id,
            'shelf_approach_location' : shelf_approach_location,
            **{k: v for k, v in req_data.items() if k not in ['task_name', 'book_id', 'shelf_approach_location']} # seat_id 외 다른 필드 포함
        }
        
        response_from_ros = _send_task_to_orchestrator(task_data)
        return jsonify(response_from_ros), 200 if response_from_ros['ok'] else 500

    except Exception as e:
        _ros_node.get_logger().error(f"Error processing /robot/pickup request: {e}")
        return jsonify({'ok': False, 'message': f'Internal server error: {e}'}), 500

@app.route('/robot/seats', methods=['POST'])
def handle_clean_seat_request():
    """
    '/robot/seats' HTTP POST 요청을 처리하여 'clean_seat' 작업을 OrchestratorNode에 전달합니다.
    요청 JSON 예시: {"seat_id": 5, "return_desk_location": {"x": 1.0, "y": 2.0, "theta": 0.0}}
    """
    try:
        req_data = request.get_json(force=True, silent=False)
        if not req_data:
            return jsonify({'ok': False, 'message': 'Request body must be JSON.'}), 400

        seat_id = req_data.get('seat_id')
        if seat_id is None: # seat_id는 0일 수도 있으므로 None 체크
            return jsonify({'ok': False, 'message': 'Missing required field: seat_id'}), 400

        # clean_seat에 필요한 다른 파라미터들은 req_data에서 직접 가져와 task_data에 포함
        # OrchestratorNode의 execute_task에서 기본값 처리가 되므로, 없으면 안 보내도 됨
        task_data = {
            "task_name": "clean_seat",
            "seat_id": seat_id,
            **{k: v for k, v in req_data.items() if k not in ['task_name', 'seat_id']} # seat_id 외 다른 필드 포함
        }
        
        response_from_ros = _send_task_to_orchestrator(task_data)
        return jsonify(response_from_ros), 200 if response_from_ros['ok'] else 500

    except Exception as e:
        _ros_node.get_logger().error(f"Error processing /robot/seats request: {e}")
        return jsonify({'ok': False, 'message': f'Internal server error: {e}'}), 500
    
@app.route('/robot/reshelving', methods=['POST'])
def handle_reshelving_request():
  
    try:
        req_data = request.get_json(force=True, silent=False)
        if not req_data:
            return jsonify({'ok': False, 'message': 'Request body must be JSON.'}), 400

        return_desk_id = req_data.get('return_desk_id')
        if return_desk_id is None: 
            return jsonify({'ok': False, 'message': 'Missing required field: book'}), 400

        # clean_seat에 필요한 다른 파라미터들은 req_data에서 직접 가져와 task_data에 포함
        # OrchestratorNode의 execute_task에서 기본값 처리가 되므로, 없으면 안 보내도 됨
        task_data = {
            "task_name": "reshelving_book",
            "return_desk_id": return_desk_id,
            **{k: v for k, v in req_data.items() if k not in ['task_name', 'return_desk_id']} # seat_id 외 다른 필드 포함
        }
        
        response_from_ros = _send_task_to_orchestrator(task_data)
        return jsonify(response_from_ros), 200 if response_from_ros['ok'] else 500

    except Exception as e:
        _ros_node.get_logger().error(f"Error processing /robot/reshelving request: {e}")
        return jsonify({'ok': False, 'message': f'Internal server error: {e}'}), 500
    
@app.route('/robot/guide', methods=['POST'])
def handle_guide_person_request():
  
    try:
        req_data = request.get_json(force=True, silent=False)
        if not req_data:
            return jsonify({'ok': False, 'message': 'Request body must be JSON.'}), 400

        dest_location = req_data.get('dest_location')
        if dest_location is None: 
            return jsonify({'ok': False, 'message': 'Missing required field: book'}), 400

        # clean_seat에 필요한 다른 파라미터들은 req_data에서 직접 가져와 task_data에 포함
        # OrchestratorNode의 execute_task에서 기본값 처리가 되므로, 없으면 안 보내도 됨
        task_data = {
            "task_name": "guide_person",
            "dest_location": dest_location,
            **{k: v for k, v in req_data.items() if k not in ['task_name', 'dest_location']} # seat_id 외 다른 필드 포함
        }
        
        response_from_ros = _send_task_to_orchestrator(task_data)
        return jsonify(response_from_ros), 200 if response_from_ros['ok'] else 500

    except Exception as e:
        _ros_node.get_logger().error(f"Error processing /robot/guide request: {e}")
        return jsonify({'ok': False, 'message': f'Internal server error: {e}'}), 500
    
@app.route('/robot/kreacher', methods=['POST'])
def handle_kreacher_request():
  
    try:
        req_data = request.get_json(force=True, silent=False)
        if not req_data:
            return jsonify({'ok': False, 'message': 'Request body must be JSON.'}), 400

        # 만약 req_data가 리스트이면 첫 번째 요소를 사용
        if isinstance(req_data, list):
            print('test log')
            if not req_data:
                return jsonify({'ok': False, 'message': 'Request body is an empty list.'}), 400
            req_data = req_data[0]
        
        _ros_node.get_logger().info(f"Received /robot/kreacher request: {req_data}")
        order_id = req_data.get('OrderID')
        # order_detail이 없을 경우를 대비하여 기본값으로 빈 딕셔너리({})를 사용합니다.
        order_detail = req_data.get('OrderDetail', {})[0]
        
        beverage_name = order_detail.get('beverageName') # 이제 order_detail이 None이 아니므로 안전합니다.
        menu_id = None
        if beverage_name == "핫아메리카노":
            menu_id = 1
        elif beverage_name == "아이스아메리카노":
            menu_id = 2 
        
        if order_id is None:
            return jsonify({'ok': False, 'message': 'Missing required field: order_id'}), 400

        # clean_seat에 필요한 다른 파라미터들은 req_data에서 직접 가져와 task_data에 포함
        # OrchestratorNode의 execute_task에서 기본값 처리가 되므로, 없으면 안 보내도 됨
        task_data = {
            "task_name": "kreacher",
            "order_id": order_id,
            "menu_id": menu_id,
            **{k: v for k, v in req_data.items() if k not in ['task_name', 'order_id', 'menu_id']} # 필수 필드 외 다른 필드 포함
        }
        
        response_from_ros = _send_task_to_orchestrator(task_data)
        return jsonify(response_from_ros), 200 if response_from_ros['ok'] else 500

    except Exception as e:
        _ros_node.get_logger().error(f"Error processing /robot/kreacher request: {e}")
        return jsonify({'ok': False, 'message': f'Internal server error: {e}'}), 500

@app.route('/test', methods=['POST']) # 기존 /test 엔드포인트는 그대로 유지하거나 필요에 따라 삭제/수정
def process_generic_test():
    """기존 /test 엔드포인트. 모든 JSON을 robot_task 서비스로 전달합니다."""
    try:
        payload_dict = request.get_json(force=True, silent=False)
        task_data = payload_dict # 이미 task_name이 포함된 JSON을 기대
    except Exception as e:
        return jsonify({'ok': False, 'message': f'Bad JSON: {e}'}), 400

    response_from_ros = _send_task_to_orchestrator(task_data)
    response_data = {'ok': response_from_ros.get('ok', False), 'message': response_from_ros.get('message', '')}
    json_data = json.dumps(response_data, ensure_ascii=False)
    return app.response_class(response=json_data, status=200, mimetype='application/json; charset=utf-8')


def main():
    """ros2 run이 호출하는 진입점"""
    wait_for_service()
    # 필요하면 host/port를 매개변수/환경변수로 빼도 됩니다.
    app.run(host='0.0.0.0', port=8001)

if __name__ == '__main__':
    main()
