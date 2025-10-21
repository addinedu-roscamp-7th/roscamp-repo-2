#!/usr/bin/env python3
import threading
from flask import Flask, jsonify
from flask_cors import CORS
import rclpy

# 기존 dobby_pickup_client.py 파일의 클래스를 그대로 임포트합니다.
from robot_control_service.dobby_pickup_client import DobbyPickupClient
from geometry_msgs.msg import Pose2D, Pose

app = Flask(__name__)
CORS(app)
app.config['JSON_AS_ASCII'] = False

# --- ROS2 노드 및 로거 설정 ---
# Flask 앱과 ROS2 노드를 분리하여 관리합니다.
# rclpy.init()은 주 실행부에서 한 번만 호출합니다.
ros_node = None

def get_ros_node():
    """ROS2 노드 싱글톤 인스턴스를 반환합니다."""
    global ros_node
    if ros_node is None:
        ros_node = rclpy.create_node('book_pickup_gateway_node')
    return ros_node

def run_pickup_task(book_id: str, storage_id: int):
    """
    별도 스레드에서 도서 픽업 ROS2 액션 클라이언트를 실행하는 함수.
    HTTP 요청 핸들러가 블로킹되지 않도록 합니다.
    """
    # 새 스레드에서는 rclpy 컨텍스트를 초기화해야 합니다.
    rclpy.init()
    
    # DevelopmentPlan.md에 따라 네임스페이스를 'dobby1/main'으로 설정합니다.
    pickup_node = DobbyPickupClient(namespace='dobby1/main')
    
    try:
        pickup_node.get_logger().info(f"스레드 시작: '{book_id}' 픽업 작업 실행")

        # 실제 시스템에서는 book_id를 기반으로 DB 등에서 위치 정보를 조회해야 합니다.
        # 여기서는 dobby_pickup_client.py의 main 함수처럼 임시 값을 사용합니다.
        pickup_goal_future = pickup_node.pickup_send_goal(
            book_id=book_id,
            storage_id=storage_id,
            shelf_approach_location=Pose2D(x=1.0, y=2.0, theta=3.0),
            book_pick_pose=Pose(), # 필요시 값 채우기
            storage_approach_location=Pose2D(x=7.0, y=8.0, theta=9.0),
            storage_slot_pose=Pose() # 필요시 값 채우기
        )

        # dobby_pickup_client.py의 main 함수에 있는 콜백 로직을 여기에 구현
        def pickup_goal_response_cb(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                pickup_node.get_logger().info('Goal rejected')
                return
            
            pickup_node.get_logger().info('Goal accepted. Waiting for result...')
            result_future = goal_handle.get_result_async()

            def result_cb(r_future):
                result = r_future.result().result
                pickup_node.get_logger().info(f'Result for {result.book_id}: {"Success" if result.success else "Failed"} - {result.message}')
                # 작업 완료 후 스레드 종료를 위해 shutdown 호출
                rclpy.shutdown()

            result_future.add_done_callback(result_cb)

        pickup_goal_future.add_done_callback(pickup_goal_response_cb)

        # 액션이 완료될 때까지(rclpy.shutdown()이 호출될 때까지) spin
        rclpy.spin(pickup_node)

    finally:
        pickup_node.destroy_node()
        # rclpy.spin()이 끝나면 shutdown이 이미 호출되었을 수 있으므로 확인
        if rclpy.ok():
            rclpy.shutdown()

@app.route('/app/books/<string:isbn>', methods=['GET'])
def pickup_book_by_isbn(isbn):
    get_ros_node().get_logger().info(f"HTTP GET /app/books/{isbn} 요청 수신")
    storage_id = 1 # 임시로 보관함 ID를 1로 고정
    
    thread = threading.Thread(target=run_pickup_task, args=(isbn, storage_id))
    thread.start()

    return jsonify({'ok': True, 'message': f"'{isbn}' 도서 픽업 작업이 시작되었습니다."}), 202

def main():
    rclpy.init()
    get_ros_node().get_logger().info("Book Pickup Gateway 시작. 포트 8002에서 대기 중...")
    app.run(host='0.0.0.0', port=8002)
    rclpy.shutdown()

if __name__ == '__main__':
    main()