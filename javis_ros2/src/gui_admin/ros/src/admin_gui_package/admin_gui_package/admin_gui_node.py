import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (QMainWindow, QApplication, QTableWidget, QTableWidgetItem, 
                             QPushButton, QVBoxLayout, QWidget, QGraphicsView, 
                             QGraphicsScene, QSplitter, QHBoxLayout, QGraphicsItemGroup, QGraphicsSimpleTextItem)
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QColor, QTransform, QPainterPath
from PyQt5 import uic
import sys
import signal
import os
from ament_index_python.packages import get_package_share_directory
from javis_interfaces.msg import DobbyState, Scheduling
from javis_interfaces.msg import BatteryStatus
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np

class AdminGUINode(Node, QMainWindow):
    """Admin GUI Node to monitor and control robots."""

    def __init__(self):
        Node.__init__(self, 'admin_gui_node')
        QMainWindow.__init__(self)
        try:
            package_share_directory = get_package_share_directory('admin_gui_package')
            ui_file_path = os.path.join(package_share_directory, 'ui', 'admin_ui.ui')
            uic.loadUi(ui_file_path, self)
            self.get_logger().info(f'UI file loaded successfully from: {ui_file_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load UI file: {e}')
            return

        self.setWindowTitle("ROS2 Robot Monitoring GUI")

        # 모니터링 탭의 위젯들을 찾습니다.
        self.monitoring_tab = self.findChild(QWidget, 'tab_monitoring')
        self.robot_status_table = self.findChild(QTableWidget, 'robotStatusTable') 
        self.standby_button = self.findChild(QPushButton, 'standbyModeButton')
        self.autonomous_button = self.findChild(QPushButton, 'autonomousModeButton')
        
        # 지도 표시 위젯 생성
        self.map_display = MapDisplayWidget(self)

        # 로봇 ID와 테이블 행을 매핑하기 위한 딕셔너리
        self.robot_row_map = {}
        # 구독자 객체를 저장할 딕셔너리
        self.robot_subscriptions = []

        # DobbyState의 sub_state를 텍스트로 변환하기 위한 맵
        self.sub_state_map = {
            DobbyState.NONE: "대기 중",
            DobbyState.MOVE_TO_PICKUP: "수거 위치로 이동 중",
            DobbyState.PICKUP_BOOK: "도서 수거 중",
            DobbyState.MOVE_TO_STORAGE: "보관 위치로 이동 중",
            DobbyState.STOWING_BOOK: "도서 보관 중",
            DobbyState.MOVE_TO_RETURN_DESK: "반납대로 이동 중",
            DobbyState.COLLECT_RETURN_BOOKS: "반납 도서 수집 중",
            DobbyState.MOVE_TO_PLACE_SHELF: "서가로 이동 중",
            DobbyState.PLACE_RETURN_BOOK: "도서 배치 중",
            DobbyState.SELECT_DEST: "목적지 선택 대기 중",
            DobbyState.SCAN_USER: "사용자 스캔 중",
            DobbyState.GUIDING_TO_DEST: "목적지로 안내 중",
            DobbyState.FIND_USER: "사용자 찾는 중",
            DobbyState.MOVE_TO_DESK: "책상으로 이동 중",
            DobbyState.SCAN_DESK: "책상 스캔 중",
            DobbyState.CLEANING_TRASH: "쓰레기 청소 중",
            DobbyState.MOVE_TO_BIN: "쓰레기통으로 이동 중",
            DobbyState.TIDYING_SHELVES: "선반 정리 중",
            DobbyState.MOVE_TO_SHELF: "선반으로 이동 중",
            DobbyState.SCAN_BOOK: "도서 스캔 중",
            DobbyState.SORT_BOOK: "도서 분류 중",
            DobbyState.SUB_ERROR: "세부 작업 오류",
        }

        # DobbyState의 main_state를 텍스트로 변환하기 위한 맵 (필요시 사용)
        # 현재는 main_state를 숫자로 표시하므로 주석 처리
        # self.main_state_map = {
        #     DobbyState.INITIALIZING: "초기화 중", DobbyState.CHARGING: "충전 중",
        #     # ... (나머지 상태들도 추가)
        # }

        self.init_gui_widgets()
        self.connect_signals()

        self.init_ros_communication()
        
        self.timer = QTimer(self)
        # 10ms마다 spin_once 호출
        self.timer.timeout.connect(self.ros_spin_once)
        self.timer.start(10) 
        
        self.get_logger().info('Admin GUI Node setup complete.')

    def init_gui_widgets(self):
        # UI 파일에 정의된 mapGraphicsView를 찾습니다.
        map_view_placeholder = self.findChild(QGraphicsView, 'mapGraphicsView')
        if map_view_placeholder:
            # 부모 위젯과 레이아웃을 찾습니다.
            parent_widget = map_view_placeholder.parentWidget()
            layout = parent_widget.layout()
            if layout:
                # 기존 QGraphicsView를 레이아웃에서 제거합니다.
                layout.removeWidget(map_view_placeholder)
                map_view_placeholder.deleteLater() # 기존 위젯은 메모리에서 삭제
                # 우리가 만든 MapDisplayWidget을 그 자리에 추가합니다.
                layout.addWidget(self.map_display)
        else:
            self.get_logger().warn("Could not find 'mapGraphicsView' in the UI file.")
        
        if self.robot_status_table:
            # 테이블 헤더 설정
            self.robot_status_table.setColumnCount(5)
            self.robot_status_table.setHorizontalHeaderLabels(['Robot ID', 'Status', 'Battery', 'Current Task', 'Error'])
            self.robot_status_table.horizontalHeader().setStretchLastSection(True)
            self.robot_status_table.setRowCount(0)
            # 사용자가 직접 수정하지 못하도록 설정
            self.robot_status_table.setEditTriggers(QTableWidget.NoEditTriggers)
            # 행 전체 선택 모드
            self.robot_status_table.setSelectionBehavior(QTableWidget.SelectRows)

        # 스케줄링 탭에 테이블 위젯 추가
        self.scheduling_tab = self.findChild(QWidget, 'tab_scheduling')
        if self.scheduling_tab:
            self.scheduling_table = QTableWidget()
            scheduling_layout = QVBoxLayout(self.scheduling_tab)
            scheduling_layout.addWidget(self.scheduling_table)
            self.scheduling_table.setColumnCount(7)
            self.scheduling_table.setHorizontalHeaderLabels(['No', 'Robot Name', 'Task ID', 'Priority', 'Status','Message', 'Creation Time'])
            self.scheduling_table.horizontalHeader().setStretchLastSection(True)
            self.scheduling_table.setEditTriggers(QTableWidget.NoEditTriggers)

        # 초기에는 버튼 비활성화
        self.standby_button.setEnabled(False)
        self.autonomous_button.setEnabled(False)

        self.selected_robot_id = None
        
        # 경로 업데이트 시간 제어를 위한 변수
        self.last_path_update_time = self.get_clock().now()
        # amcl_pose 업데이트 시간 제어를 위한 변수
        self.last_amcl_pose_update_time = self.get_clock().now()

    def connect_signals(self):
        if self.standby_button:
            self.standby_button.clicked.connect(self.on_standby_mode_clicked)
        if self.autonomous_button:
            self.autonomous_button.clicked.connect(self.on_autonomous_mode_clicked)
        if self.robot_status_table:
            self.robot_status_table.itemSelectionChanged.connect(self.on_robot_selection_changed)

    def init_ros_communication(self):
        # 모니터링할 로봇 목록
        robot_ids = ['dobby1', 'dobby2', 'kreacher'] # 예시 로봇 ID

        for robot_id in robot_ids:
            # 1. 각 로봇의 상태 토픽 구독
            topic_name = f'/{robot_id}/status/robot_state'
            sub = self.create_subscription(
                DobbyState,
                topic_name,
                # lambda를 사용하여 콜백에 로봇 ID 전달
                lambda msg, rid=robot_id: self.status_callback(msg, rid),
                10)
            self.robot_subscriptions.append(sub)
            self.get_logger().info(f"Subscribing to {topic_name}")

            # 2. 각 로봇의 배터리 상태 토픽 구독
            battery_topic_name = f'/{robot_id}/status/battery_status'
            battery_sub = self.create_subscription(
                BatteryStatus,
                battery_topic_name,
                lambda msg, rid=robot_id: self.battery_status_callback(msg, rid),
                10)
            self.robot_subscriptions.append(battery_sub)
            self.get_logger().info(f"Subscribing to {battery_topic_name}")

            # 3. 각 로봇의 스케줄링 토픽 구독
            scheduling_topic_name = f'/{robot_id}/status/task_scheduling'
            scheduling_sub = self.create_subscription(
                Scheduling,
                scheduling_topic_name,
                self.scheduling_callback, # 모든 로봇의 스케줄링을 하나의 콜백으로 처리
                10)
            self.robot_subscriptions.append(scheduling_sub)
            self.get_logger().info(f"Subscribing to {scheduling_topic_name}")

            # 5. 각 로봇의 위치 토픽 구독
            pose_topic_name = f'/{robot_id}/pose'
            pose_sub = self.create_subscription(
                PoseStamped,
                pose_topic_name,
                lambda msg, rid=robot_id: self.pose_callback(msg, rid),
                10)
            self.robot_subscriptions.append(pose_sub)

        # 4. 단일 '/plan' 토픽 구독
        path_topic_name = '/plan'
        path_sub = self.create_subscription(
            Path,
            path_topic_name,
            self.path_callback,
            10)
        self.robot_subscriptions.append(path_sub)
        self.get_logger().info(f"Subscribing to {path_topic_name}")

        # 6. /amcl_pose 토픽 구독
        # /amcl_pose는 PoseWithCovarianceStamped 메시지 타입을 사용합니다.
        from geometry_msgs.msg import PoseWithCovarianceStamped
        amcl_pose_topic_name = '/amcl_pose'
        amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_pose_topic_name,
            self.amcl_pose_callback,
            10)
        self.robot_subscriptions.append(amcl_pose_sub)
        self.get_logger().info(f"Subscribing to {amcl_pose_topic_name}")

    def ros_spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)


    def status_callback(self, msg, robot_id):
        # 로봇 ID가 테이블에 없으면 새로 추가
        if robot_id not in self.robot_row_map:
            row_position = self.robot_status_table.rowCount()
            self.robot_status_table.insertRow(row_position)
            self.robot_row_map[robot_id] = row_position
            self.robot_status_table.setItem(row_position, 0, QTableWidgetItem(robot_id))

        row = self.robot_row_map[robot_id]

        # sub_state를 텍스트로 변환
        sub_state_text = self.sub_state_map.get(msg.sub_state, "알 수 없음")

        # 테이블 아이템 업데이트
        self.robot_status_table.setItem(row, 1, QTableWidgetItem(str(msg.main_state))) # Status
        self.robot_status_table.setItem(row, 3, QTableWidgetItem(sub_state_text)) # Current Task
        self.robot_status_table.setItem(row, 4, QTableWidgetItem(msg.error_message if msg.is_error else "OK"))

    def battery_status_callback(self, msg, robot_id):
        if robot_id not in self.robot_row_map:
            # 아직 상태 메시지를 받지 못해 행이 생성되지 않은 경우, 행을 먼저 생성합니다.
            row_position = self.robot_status_table.rowCount()
            self.robot_status_table.insertRow(row_position)
            self.robot_row_map[robot_id] = row_position
            self.robot_status_table.setItem(row_position, 0, QTableWidgetItem(robot_id))
            self.get_logger().warn(f"Received battery status for '{robot_id}' before general status. Ignoring.")

        row = self.robot_row_map[robot_id]
        self.robot_status_table.setItem(row, 2, QTableWidgetItem(f"{msg.charge_percentage:.1f}%"))

    def scheduling_callback(self, msg):
        """Scheduling 메시지를 받아 스케줄링 테이블을 업데이트하거나 새로 추가합니다."""
        if not self.scheduling_table:
            return

        # msg.no와 동일한 'No'를 가진 행이 있는지 검색
        found_row = -1
        for row in range(self.scheduling_table.rowCount()):
            no_item = self.scheduling_table.item(row, 0)
            if no_item and no_item.text() == str(msg.no):
                found_row = row
                break

        if found_row != -1:
            # 기존 행이 있으면 해당 행의 위치를 사용
            row_position = found_row
        else:
            # 기존 행이 없으면 새로운 행을 추가
            row_position = self.scheduling_table.rowCount()
            self.scheduling_table.insertRow(row_position)
        self.status_str = ''
        if msg.status == 1:
            self.status_str = "할당됨"
        elif msg.status == 2:
            self.status_str = "완료됨"
        elif msg.status == 3:
            self.status_str = "실패"
        elif msg.status == 4:
            self.status_str = "진행중"
        elif msg.status == 5:
            self.status_str = "대기중"

        # 행 데이터 설정 (추가 또는 업데이트)
        self.scheduling_table.setItem(row_position, 0, QTableWidgetItem(str(msg.no)))
        self.scheduling_table.setItem(row_position, 1, QTableWidgetItem(msg.robot_name))
        self.scheduling_table.setItem(row_position, 2, QTableWidgetItem(str(msg.task_id)))
        self.scheduling_table.setItem(row_position, 3, QTableWidgetItem(str(msg.priority)))
        self.scheduling_table.setItem(row_position, 4, QTableWidgetItem(self.status_str))
        self.scheduling_table.setItem(row_position, 5, QTableWidgetItem(str(msg.message)))
        self.scheduling_table.setItem(row_position, 6, QTableWidgetItem(msg.task_create_time))

    def on_robot_selection_changed(self):
        selected_items = self.robot_status_table.selectedItems()
        if selected_items:
            selected_row = selected_items[0].row()
            robot_id_item = self.robot_status_table.item(selected_row, 0)
            if robot_id_item:
                self.selected_robot_id = robot_id_item.text()
                self.get_logger().info(f"로봇 선택됨: {self.selected_robot_id}")
                # 로봇이 선택되면 버튼 활성화
                self.standby_button.setEnabled(True)
                self.autonomous_button.setEnabled(True)
        else:
            self.selected_robot_id = None
            # 선택이 해제되면 버튼 비활성화
            self.standby_button.setEnabled(False)
            self.autonomous_button.setEnabled(False)
            
    def on_standby_mode_clicked(self):
        if self.selected_robot_id:
            self.get_logger().info(f"로봇 {self.selected_robot_id}에게 대기 모드 명령 발행 (기능 미구현)")
        else:
            self.get_logger().warn("모드 명령을 내릴 로봇을 먼저 테이블에서 선택하세요.")

    def on_autonomous_mode_clicked(self):
        if self.selected_robot_id:
            self.get_logger().info(f"로봇 {self.selected_robot_id}에게 자율 이동 모드 명령 발행 (기능 미구현)")
        else:
            self.get_logger().warn("모드 명령을 내릴 로봇을 먼저 테이블에서 선택하세요.")

    def path_callback(self, msg):
        # 지도가 로드되었는지 먼저 확인합니다.
        if not self.map_display.map_info:
            return
        # 1초에 한 번만 경로를 업데이트하도록 조절합니다.
        current_time = self.get_clock().now()
        if (current_time - self.last_path_update_time).nanoseconds > 1e9: # 1초 (1*10^9 나노초)
            self.map_display.update_path(msg)
            self.last_path_update_time = current_time

    def pose_callback(self, msg, robot_id):
        self.map_display.update_robot_pose(robot_id, msg.pose)

    def amcl_pose_callback(self, msg):
        """/amcl_pose 토픽을 처리하여 'amcl' ID로 로봇 위치를 업데이트합니다."""
        # 1초에 한 번만 위치를 업데이트하도록 조절합니다.
        current_time = self.get_clock().now()
        if (current_time - self.last_amcl_pose_update_time).nanoseconds > 1e9: # 1초 (1*10^9 나노초)
            # PoseWithCovarianceStamped 메시지에서 Pose 정보를 추출합니다.
            self.map_display.update_robot_pose('amcl', msg.pose.pose)
            self.last_amcl_pose_update_time = current_time

    def waypoints_callback(self, msg):
        # 지도가 로드되었는지 먼저 확인합니다.
        if not self.map_display.map_info:
            return
        self.map_display.update_waypoints(msg)


def main(args=None):
    rclpy.init(args=args) 

    app = QApplication(sys.argv)

    # Ctrl+C (SIGINT) 신호에 대한 핸들러 설정
    signal.signal(signal.SIGINT, lambda sig, frame: QApplication.quit())

    node = AdminGUINode()

    node.show()
    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()

class MapDisplayWidget(QGraphicsView):
    """지도, 로봇 위치, 경로를 표시하는 위젯"""
    def __init__(self, parent_node: Node):
        super().__init__()
        self.node = parent_node
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        
        # 사용자가 지도를 드래그하여 이동할 수 있도록 설정
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)

        self.map_item = None
        self.path_item = None
        self.waypoints_item = None # 웨이포인트 아이템 그룹
        self.robot_items = {} # robot_id: QGraphicsItem

        self.map_info = None
        self.initial_map_loaded = False # 지도가 처음 로드되었는지 확인하는 플래그

        # 지도 토픽 구독
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

    def world_to_scene(self, world_x, world_y):
        """월드 좌표를 QGraphicsScene 좌표로 변환합니다."""
        if not self.map_info:
            return world_x, world_y

        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        map_height_m = self.map_info.height * resolution

        # QImage 생성 시 Y축을 뒤집었으므로, 경로를 그릴 때도 동일한 변환을 적용해야 합니다.
        # Scene의 Y = Map 원점 Y + (Map 높이(m) - (월드 Y - Map 원점 Y))
        scene_y = origin.y + map_height_m - (world_y - origin.y)
        return world_x, scene_y

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        width = self.map_info.width
        height = self.map_info.height
        resolution = self.map_info.resolution

        # OccupancyGrid 데이터를 QImage로 변환
        image = QImage(width, height, QImage.Format_ARGB32)
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                occupancy = msg.data[idx]
                if occupancy == -1: # Unknown
                    color = QColor(200, 200, 200)
                elif occupancy == 100: # Occupied
                    color = QColor(0, 0, 0)
                else: # Free
                    color = QColor(255, 255, 255)                
                image.setPixelColor(x, height - 1 - y, color) # Y축 반전

        pixmap = QPixmap.fromImage(image)
        if not self.map_item:
            self.map_item = self.scene.addPixmap(pixmap)
            self.map_item.setTransform(QTransform().scale(resolution, resolution))
            self.map_item.setPos(self.map_info.origin.position.x, self.map_info.origin.position.y)

            if not self.initial_map_loaded:
                self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
                self.initial_map_loaded = True
        else:
            self.map_item.setPixmap(pixmap)
        

    def update_path(self, path_msg: Path):
        if self.path_item:
            self.scene.removeItem(self.path_item)
        
        if not self.map_info or not path_msg.poses:
            return

        path = QPainterPath()

        start_point = path_msg.poses[0].pose.position
        sx, sy = self.world_to_scene(start_point.x, start_point.y)
        path.moveTo(sx, sy)

        for pose_stamped in path_msg.poses[1:]:
            pos = pose_stamped.pose.position
            px, py = self.world_to_scene(pos.x, pos.y)
            path.lineTo(px, py)

        pen = QPen(QColor("blue"), 0.05) # 경로 두께를 지도 스케일에 맞게 조절
        self.path_item = self.scene.addPath(path, pen)

    def update_waypoints(self, path_msg: Path):
        if self.waypoints_item:
            self.scene.removeItem(self.waypoints_item)
            self.waypoints_item = None

        if not self.map_info or not path_msg.poses:
            return

        # QGraphicsItemGroup을 사용하여 모든 웨이포인트 그래픽을 그룹화합니다.
        self.waypoints_item = QGraphicsItemGroup()
        self.scene.addItem(self.waypoints_item)

        pen = QPen(QColor("blue"), 0.05) # 웨이포인트 원의 테두리 펜

        for i, pose_stamped in enumerate(path_msg.poses):
            pos = pose_stamped.pose.position
            px, py = self.world_to_scene(pos.x, pos.y)

            # 웨이포인트를 원으로 표시
            radius = 0.2 # 원의 반지름 (미터 단위)
            ellipse = self.scene.addEllipse(px - radius, py - radius, radius * 2, radius * 2, pen, QColor(0, 0, 255, 100))
            
            # 웨이포인트 번호 표시
            text = QGraphicsSimpleTextItem(str(i))
            text.setBrush(QColor("white"))
            font = text.font()
            font.setPointSize(10) # 폰트 크기
            text.setFont(font)
            # 텍스트 위치를 원의 중심으로 조정
            text_rect = text.boundingRect()
            text.setPos(px - text_rect.width() / 2, py - text_rect.height() / 2)
            
            # 생성된 그래픽 아이템들을 그룹에 추가
            self.waypoints_item.addToGroup(ellipse)
            self.waypoints_item.addToGroup(text)

    def update_robot_pose(self, robot_id, pose):
        # 로봇 위치도 경로와 동일한 좌표 변환을 적용해야 합니다.
        if not self.map_info:
            return

        # 기존에 그려진 아이템이 있으면 삭제
        if robot_id in self.robot_items:
            self.scene.removeItem(self.robot_items[robot_id])

        # 월드 좌표를 Scene 좌표로 변환
        sx, sy = self.world_to_scene(pose.position.x, pose.position.y)

        # 로봇을 원으로 표시 (크기는 지도 스케일에 맞게 조절)
        pen = QPen(QColor("red"), 0.05)
        self.robot_items[robot_id] = self.scene.addEllipse(sx - 0.2, sy - 0.2, 0.4, 0.4, pen, QColor("red"))

    def wheelEvent(self, event):
        """마우스 휠을 사용하여 지도를 확대/축소합니다."""
        zoom_in_factor = 1.25
        zoom_out_factor = 1 / zoom_in_factor
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        if event.angleDelta().y() > 0:
            self.scale(zoom_in_factor, zoom_in_factor)
        else:
            self.scale(zoom_out_factor, zoom_out_factor)