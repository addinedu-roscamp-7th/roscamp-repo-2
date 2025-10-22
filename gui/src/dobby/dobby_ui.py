import sys
import os 
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import QByteArray, QDir
from PyQt5 import uic
from PyQt5.QtWidgets import QButtonGroup

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from PyQt5.QtCore import QThread, pyqtSignal, QTimer 

# 🚨 ROS 2 커스텀 메시지 import
# 'amr_state_publisher' 패키지가 빌드되어 있어야 합니다.
from amr_state_publisher.msg import DobbyState 

# ----------------------------------------------------
# 스크립트 파일이 위치한 디렉토리를 기준으로 경로를 동적으로 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_ASSET_DIR = QDir(SCRIPT_DIR).path() + '/image' 

# 🚨 FileNotFoundError 해결: UI 파일 경로를 절대 경로로 지정
ui_file_path = os.path.join(SCRIPT_DIR, "dobby_ui.ui")
form_window = uic.loadUiType(ui_file_path)[0]


# ====================================================
# ROS 2 구독을 처리하는 별도 스레드 클래스
# ====================================================
class RosNodeManager(QThread):
    # GUI로 main_state ID (int)를 보낼 시그널 정의
    state_updated = pyqtSignal(int) 

    def __init__(self):
        super().__init__()
        # ROS 2 초기화는 노드를 생성하기 전에 한 번만 수행
        if not rclpy.ok():
            rclpy.init(args=None) 
        
    def run(self):
        """별도 스레드에서 ROS 2 노드를 실행합니다."""
        self.node = rclpy.create_node('gui_state_subscriber')
        
        # QoS 설정 (발행 노드와 동일하게 Reliable 사용)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 'dobby1/status/robot_state' 토픽 구독 설정
        self.subscription = self.node.create_subscription(
            DobbyState,
            'dobby1/status/robot_state',
            self.listener_callback,
            qos_profile)
        
        self.node.get_logger().info('ROS 2 Subscriber Node started. Listening to /dobby1/status/robot_state')
        
        rclpy.spin(self.node) 

    def listener_callback(self, msg):
        """로봇 상태 메시지를 수신했을 때 호출됨"""
        # 수신된 main_state (int)를 GUI 스레드로 시그널 전송
        self.state_updated.emit(msg.main_state) 

    def shutdown(self):
        """노드를 안전하게 정리하고 ROS 2를 종료합니다."""
        if hasattr(self, 'node') and self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()


# ====================================================
# 화면을 띄우는데 사용되는 Main Class
# ====================================================
class WindowClass(QMainWindow, form_window) :
    def __init__(self) :
        super().__init__()
        self.setupUi(self)
        
        # ... (QSS 스타일 시트 코드 생략) ...
        self.setStyleSheet("""
            QPushButton[checkable="true"] {
                background-color: #E0E0E0; /* 기본 연한 회색 */
                border: 1px solid #B0B0B0;
                padding: 5px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton[checkable="true"]:checked {
                background-color: #1E90FF; /* 선택 시 밝은 파란색 */
                color: white; /* 글자색 흰색 */
                border: 2px solid #0077CC;
            }
        """)

        # 1. QMovie 객체를 클래스 멤버 변수로 선언
        self.movie_loading = QMovie(BASE_ASSET_DIR + '/Loading.gif', QByteArray(), self)
        self.movie_charging = QMovie(BASE_ASSET_DIR + '/Charging.gif', QByteArray(), self)
        self.movie_walking = QMovie(BASE_ASSET_DIR + '/Walking.gif', QByteArray(), self)

        # 🚨 ROS 2 ID에 맞게 state_map 키와 UI Index를 매핑합니다.
        # DobbyState ID: (StackedWidget Index, QMovie Object, QLabel Object)
        self.state_map = {
            DobbyState.CHARGING:        (0, self.movie_charging, self.label_charging),  # ID=1 (Charging) -> Index 0
            DobbyState.IDLE:            (1, self.movie_walking, self.label_walking),    # ID=2 (Walking) -> Index 1
            DobbyState.PICKING_UP_BOOK: (2, self.movie_loading, self.label_loading),    # ID=4 (Loading) -> Index 2
            # 🚨 알 수 없는 상태는 IDLE (1, Index 1)로 폴백 처리하기 위해 추가
            DobbyState.INITIALIZING:    (1, self.movie_walking, self.label_walking),  # ID=0 (Initializing) -> Index 1
            DobbyState.MAIN_ERROR:      (2, self.movie_loading, self.label_loading),  # ID=99 (Error) -> Index 2 (Loading or Error 화면)
        }
        
        # 2. QButtonGroup 도입 및 연결
        self.button_group = QButtonGroup(self)
        self.button_group.addButton(self.testByn01, 1) # Charging (UI ID 1)
        self.button_group.addButton(self.testByn02, 2) # Walking (UI ID 2)
        self.button_group.addButton(self.testByn03, 3) # Loading (UI ID 3)
        self.button_group.setExclusive(True) 
        # 🚨 ROS 2 구독으로 전환: 버튼 토글 시그널은 유지하되, 버튼 클릭을 막습니다.
        self.button_group.buttonToggled.connect(self.imgChange)

        self.init_gif_setup() 

        # 3. QStackedWidget의 현재 인덱스가 바뀔 때 호출될 시그널 연결
        self.stackedWidget_gif.currentChanged.connect(self.manage_gif_playback)
        
        # 4. ROS 2 Manager 초기화, 시그널 연결 및 시작
        self.ros_manager = RosNodeManager()
        self.ros_manager.state_updated.connect(self.update_gui_from_ros) 
        self.ros_manager.start() 

        # 5. 테스트 버튼 비활성화 (ROS 통신 우선)
        self.testByn01.setEnabled(False) 
        self.testByn02.setEnabled(False)
        self.testByn03.setEnabled(False)


    def init_gif_setup(self):
        """초기 GIF 및 UI 설정."""
        
        self.label_charging.setMovie(self.movie_charging)
        self.label_walking.setMovie(self.movie_walking)
        self.label_loading.setMovie(self.movie_loading)

        # 초기 상태: Walking (ID 2, Index 1)로 설정 (IDLE 상태)
        # 🚨 DobbyState.IDLE에 해당하는 UI 버튼 ID (2)를 체크합니다.
        self.button_group.button(2).setChecked(True) 
        
        # ... (경로 유효성 검사 및 에러 메시지 설정 코드 생략) ...
        # (기존 코드가 이미 이 부분을 처리하고 있으므로 유지)
        for _, (index, movie, label) in self.state_map.items():
            full_path = movie.fileName()
            if not os.path.exists(full_path):
                 print(f"ERROR: '{full_path}' 경로에 파일이 존재하지 않습니다. 경로/파일명이 올바른지 확인하세요!")
                 label.setText(f"FATAL ERROR: GIF 로드 실패.\n['{os.path.basename(full_path)}']경로를 확인하세요.")
            elif not movie.isValid():
                 print(f"ERROR: {os.path.basename(full_path)} 파일이 유효하지 않습니다. (파일 손상 가능성)")
                 label.setText(f"ERROR: GIF 파일이 유효하지 않음.\n['{os.path.basename(full_path)}']")

    def manage_gif_playback(self, new_index):
        # ... (기존 코드 그대로 유지) ...
        for _, (index, movie, _) in self.state_map.items():
            if movie.state() == QMovie.Running:
                movie.stop()

        for _, (index, movie, _) in self.state_map.items():
            if index == new_index and movie.isValid():
                movie.start()
                print(f"화면 전환 완료: Index {new_index} ({os.path.basename(movie.fileName())}) 재생 시작")
                break
            
    def imgChange(self, button, checked):
        # ... (기존 코드 그대로 유지) ...
        if not checked:
            return

        button_id = self.button_group.id(button)
        try:
            # 🚨 button_id는 UI의 ID (1, 2, 3)이며, state_map 키는 DobbyState ID (1, 2, 4 등)입니다.
            # 이 함수는 버튼 클릭 시 호출되므로, UI ID에 맞는 DobbyState ID로 변환해야 합니다.
            # 하지만, 지금은 이 함수가 ROS 구독에 의해 호출될 때 버튼 상태를 동기화하는 역할만 하도록 의도합니다.
            # 버튼 ID에 맞는 DobbyState ID를 찾아 인덱스를 사용해야 합니다.
            
            # UI ID (1, 2, 3) -> Dobby State ID 변환 (1, 2, 4)
            dobby_state_id = button_id
            if button_id == 3: # UI ID 3 (Loading) -> Dobby State ID 4 (PICKING_UP_BOOK)
                dobby_state_id = DobbyState.PICKING_UP_BOOK 

            index, _, _ = self.state_map[dobby_state_id] 
            self.stackedWidget_gif.setCurrentIndex(index)
        except KeyError:
             print(f"ERROR: 할당되지 않은 버튼 ID {button_id}가 감지되었습니다.")


    # 🚨 ROS 2 구독 시그널을 받는 새 메서드 추가
    def update_gui_from_ros(self, new_state_id):
        """ROS 2로부터 받은 main_state ID에 따라 화면을 전환하고 버튼을 동기화합니다."""
        
        # Dobby State ID (1, 2, 4) -> UI Button ID (1, 2, 3)으로 변환하여 버튼 체크
        ui_button_id = -1
        if new_state_id == DobbyState.CHARGING:        # ID 1
            ui_button_id = 1
        elif new_state_id == DobbyState.IDLE:          # ID 2
            ui_button_id = 2
        elif new_state_id == DobbyState.PICKING_UP_BOOK:# ID 4
            ui_button_id = 3
        
        try:
            # 1. 화면 전환 및 버튼 체크 동기화
            if ui_button_id != -1 and self.button_group.button(ui_button_id):
                # setChecked(True)를 호출하면 imgChange가 호출되어 화면이 전환됩니다.
                self.button_group.button(ui_button_id).setChecked(True) 
            
        except Exception as e:
            # 알 수 없는 상태 ID가 수신되면, 기본 대기 화면(IDLE)으로 전환
            print(f"WARNING: 상태 ID {new_state_id} 처리 중 오류 발생: {e}. IDLE로 전환합니다.")
            self.button_group.button(2).setChecked(True) # Walking 버튼 (UI ID 2) 체크

    # 🚨 창이 닫힐 때 ROS 2 노드도 안전하게 종료
    def closeEvent(self, event):
        print("GUI 종료 요청. ROS 2 노드를 안전하게 종료합니다.")
        self.ros_manager.shutdown() 
        event.accept()
        

if __name__ == "__main__" :
    # 🚨 ROS 2를 사용하므로 rclpy.ok() 확인 후 종료하지 않도록 수정합니다.
    app = QApplication(sys.argv) 
    adminWindow = WindowClass() 
    adminWindow.show()
    # sys.exit(app.exec_()) 대신 rclpy.spin()을 사용할 수도 있지만, 
    # PyQt의 event loop을 사용하고 ROS 노드를 QThread로 분리했으므로 기존 방식을 유지합니다.
    sys.exit(app.exec_())