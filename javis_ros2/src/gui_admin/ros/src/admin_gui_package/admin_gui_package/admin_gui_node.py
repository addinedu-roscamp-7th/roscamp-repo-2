import rclpy
from rclpy.node import Node
# QTimer를 사용하여 ROS2 spin_once를 주기적으로 호출
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QTableWidgetItem
from PyQt5 import uic
import sys
import os
from ament_index_python.packages import get_package_share_directory
# 로봇 상태 메시지를 구독한다고 가정 (예시 메시지 타입)
# from your_msgs.msg import RobotStatus # 실제 사용할 ROS2 메시지로 변경 필요

# 노드와 GUI 창을 상속받는 클래스
class AdminGUINode(QMainWindow, Node):
    def __init__(self):
        # 1. ROS2 노드 초기화
        super().__init__('admin_gui_node') 

        # 2. UI 파일 로드
        try:
            # ament 인덱스를 사용하여 패키지 공유 디렉토리에서 UI 파일 경로를 찾습니다.
            package_share_directory = get_package_share_directory('admin_gui_package')
            ui_file_path = os.path.join(package_share_directory, 'ui', 'admin_ui.ui')
            uic.loadUi(ui_file_path, self)
            self.get_logger().info(f'UI file loaded successfully from: {ui_file_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load UI file: {e}')
            return

        self.setWindowTitle("ROS2 Robot Monitoring GUI")

        # 3. 위젯 찾기 (objectName 사용)
        self.robot_status_table = self.findChild(QTableWidget, 'robotStatusTable')
        self.standby_button = self.findChild(QPushButton, 'standbyModeButton')
        self.autonomous_button = self.findChild(QPushButton, 'autonomousModeButton')
        
        # 4. 초기 GUI 설정 및 연결
        self.init_gui_widgets()
        self.connect_signals()

        # 5. ROS2 통신 초기화 (Subscriber 및 Publisher)
        self.init_ros_communication()
        
        # 6. ROS2 스핀 타이머 설정 (GUI 루프 내에서 ROS2 처리)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_spin_once)
        # 10ms마다 spin_once 호출
        self.timer.start(10) 
        
        self.get_logger().info('Admin GUI Node setup complete.')

    def init_gui_widgets(self):
        # 로봇 상태 테이블 초기 설정
        if self.robot_status_table:
            # 테이블의 행 높이를 내용에 맞게 조정하도록 설정
            self.robot_status_table.horizontalHeader().setStretchLastSection(True)
            self.robot_status_table.setRowCount(0) # 초기 행은 0개

    def connect_signals(self):
        # 모드 제어 버튼 연결 (클릭 시 실행될 콜백 함수)
        if self.standby_button:
            self.standby_button.clicked.connect(self.on_standby_mode_clicked)
        if self.autonomous_button:
            self.autonomous_button.clicked.connect(self.on_autonomous_mode_clicked)

        # 상태 테이블의 항목이 선택되었을 때의 이벤트 연결 (선택된 로봇 ID를 가져오기 위해)
        if self.robot_status_table:
            self.robot_status_table.itemSelectionChanged.connect(self.on_robot_selection_changed)

    def init_ros_communication(self):
        # TODO: 실제 로봇 상태를 구독할 Subscriber 정의
        # self.status_subscriber = self.create_subscription(
        #     RobotStatus, 
        #     'robot_status_topic', 
        #     self.status_callback, 
        #     10)
        
        # TODO: 모드 제어를 위한 Publisher 또는 Service Client 정의
        # self.mode_publisher = self.create_publisher(
        #     ModeCommand, 
        #     'robot_mode_command', 
        #     10)
        pass

    def ros_spin_once(self):
        # GUI의 메인 루프에서 주기적으로 ROS2의 콜백 함수를 처리
        rclpy.spin_once(self, timeout_sec=0)

    # --- 콜백 및 이벤트 핸들러 ---

    def status_callback(self, msg):
        """로봇 상태 메시지를 수신하여 테이블을 업데이트하는 콜백 함수"""
        # TODO: 메시지 내용을 파싱하여 robotStatusTable을 업데이트하는 로직 구현
        # 예시: self.update_robot_status_table(msg.robot_id, msg.connection, msg.battery, msg.task_state)
        pass

    def on_robot_selection_changed(self):
        """테이블에서 로봇이 선택되었을 때 호출되는 함수"""
        selected_items = self.robot_status_table.selectedItems()
        if selected_items:
            # 첫 번째 열(ID)의 항목을 가져옵니다.
            selected_row = selected_items[0].row()
            robot_id_item = self.robot_status_table.item(selected_row, 0)
            if robot_id_item:
                self.selected_robot_id = robot_id_item.text()
                self.get_logger().info(f"로봇 선택됨: {self.selected_robot_id}")
            
    def on_standby_mode_clicked(self):
        """'대기 모드' 버튼 클릭 시 ROS2 명령을 발행하는 함수"""
        if hasattr(self, 'selected_robot_id'):
            self.get_logger().info(f"로봇 {self.selected_robot_id}에게 대기 모드 명령 발행")
            # TODO: ROS2 Publisher를 사용하여 '대기 모드' 명령 발행 로직 구현
        else:
            self.get_logger().warn("모드 명령을 내릴 로봇을 먼저 테이블에서 선택하세요.")

    def on_autonomous_mode_clicked(self):
        """'자율 이동 모드' 버튼 클릭 시 ROS2 명령을 발행하는 함수"""
        if hasattr(self, 'selected_robot_id'):
            self.get_logger().info(f"로봇 {self.selected_robot_id}에게 자율 이동 모드 명령 발행")
            # TODO: ROS2 Publisher를 사용하여 '자율 이동 모드' 명령 발행 로직 구현
        else:
            self.get_logger().warn("모드 명령을 내릴 로봇을 먼저 테이블에서 선택하세요.")


def main(args=None):
    # 1. ROS2 초기화
    rclpy.init(args=args) 

    # 2. QApplication 초기화 (GUI 프레임워크 초기화)
    app = QApplication(sys.argv)
    
    # 3. 노드 및 GUI 창 생성
    node = AdminGUINode()

    # 4. GUI 창 표시
    node.show()

    # 5. GUI 루프 실행 (ROS2 처리는 QTimer에 의해 주기적으로 실행됨)
    exit_code = app.exec_()

    # 6. ROS2 종료 (GUI가 닫힌 후 실행)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()