import sys
import os 
import signal
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import QByteArray, QDir, QTimer # QTimer 추가
from PyQt5 import uic

# 🌟 ROS 2 관련 임포트 (javis_interfaces 패키지 가정)
import rclpy
from rclpy.node import Node
from javis_interfaces.msg import DobbyState 

# ----------------------------------------------------
# 🌟 1. ament_index_python 임포트
from ament_index_python.packages import get_package_share_directory

# 🌟 2. 기존 경로 설정 (13~15행)을 삭제하고 아래 코드로 대체합니다.
package_name = 'dobby_state_package' # 본인 패키지 이름
package_share_directory = get_package_share_directory(package_name)

# 🌟 3. 설치된 share 디렉토리 기준으로 경로를 재설정합니다.
UI_FILE_PATH = os.path.join(package_share_directory, 'ui', 'dobby_ui.ui')
IMAGE_DIR = os.path.join(package_share_directory, 'image')
# ----------------------------------------------------

# UI파일 연결
form_window = uic.loadUiType(UI_FILE_PATH)[0]

# 화면을 띄우는데 사용되는 Class 선언
class WindowClass(QMainWindow, form_window, Node) :
    
    def __init__(self, robot_name='dobby1') :
        # 1. PyQt Main Window/form_window 초기화: super()를 먼저 호출하여 PyQt 생성자 체인 활성화
        super().__init__() 
        
        # 2. ROS 2 Node 초기화
        Node.__init__(self, f'{robot_name}_state_gui_node', namespace=robot_name)
        
        # 3. UI 설정
        self.setupUi(self)

        self.current_movie = None

        # QMovie 객체를 클래스 멤버 변수로 선언하고, 동적 경로를 사용합니다.
        #    GIF 파일명을 정확히 반영합니다. (Loading.gif, Charging.gif, Walking.gif)
        self.movie_loading  = QMovie(os.path.join(IMAGE_DIR, 'Loading.gif'), QByteArray(), self)
        self.movie_charging = QMovie(os.path.join(IMAGE_DIR, 'Charging.gif'), QByteArray(), self)
        self.movie_walking  = QMovie(os.path.join(IMAGE_DIR, 'Walking.gif'), QByteArray(), self)
        self.movie_working  = QMovie(os.path.join(IMAGE_DIR, 'Working.gif'), QByteArray(), self)
        self.movie_resting  = QMovie(os.path.join(IMAGE_DIR, 'Resting.gif'), QByteArray(), self)
        self.movie_error    = QMovie(os.path.join(IMAGE_DIR, 'Error.gif'), QByteArray(), self)

        # 2. QStackedWidget 페이지 및 QLabel 맵핑
        # dobby_ui.ui 구조에 따른 인덱스 맵핑 (0: Charging, 1: Walking, 2: Loading)
        # Note: 실제 UI 파일의 page 순서에 따라 인덱스를 확인해야 합니다.
        self.state_to_page_map = {
            # 🌟 Charging (Index 0)
            DobbyState.CHARGING: 0,
            # 🌟 Walking (Index 1) - 이동 관련 상태
            DobbyState.MOVING_TO_CHARGER: 1, 
            DobbyState.GUIDING: 1,
            DobbyState.ROAMING: 1,
            # 🌟 Loading (Index 2) - 초기화 및 대기 관련
            DobbyState.INITIALIZING: 2,
            DobbyState.LISTENING: 2, # 'listening'은 Loading 상태로 간주
            # 🌟 Working (Index 3) - 작업 수행 관련
            DobbyState.PICKING_UP_BOOK: 3,
            DobbyState.RESHELVING_BOOK: 3, 
            DobbyState.CLEANING_DESK: 3,
            DobbyState.SORTING_SHELVES: 3,
            # 🌟 Resting (Index 4) - 대기/유휴 상태
            DobbyState.IDLE: 4, 
            # 🌟 Error (Index 5) - 에러 및 정지 상태
            DobbyState.EMERGENCY_STOP: 5,
            DobbyState.MAIN_ERROR: 5,
        }

        # 상태별 QMovie 객체 매핑
        self.state_to_movie_map = {
            DobbyState.CHARGING: self.movie_charging,
            DobbyState.MOVING_TO_CHARGER: self.movie_walking,
            DobbyState.GUIDING: self.movie_walking,
            DobbyState.ROAMING: self.movie_walking,
            DobbyState.INITIALIZING: self.movie_loading,
            DobbyState.LISTENING: self.movie_loading,
            DobbyState.PICKING_UP_BOOK: self.movie_working,
            DobbyState.RESHELVING_BOOK: self.movie_working,
            DobbyState.CLEANING_DESK: self.movie_working,
            DobbyState.SORTING_SHELVES: self.movie_working,
            DobbyState.IDLE: self.movie_resting,
            DobbyState.EMERGENCY_STOP: self.movie_error,
            DobbyState.MAIN_ERROR: self.movie_error,
        }

        # ID와 QMovie 객체를 매핑하여 딕셔너리로 관리 (코드 간결성 및 확장성)
        self.movie_map = {
            1: self.movie_charging,
            2: self.movie_walking,
            3: self.movie_loading,
            4: self.movie_working,
            5: self.movie_resting,
            6: self.movie_error,
        }
        # 버튼 ID와 QStackedWidget의 페이지 인덱스(currentIndex)를 매핑합니다.
        self.page_map = {
            1: 0, # ID 1 (Charging) -> Index 0 (page_charging)
            2: 1, # ID 2 (Walking) -> Index 1 (page_walking)
            3: 2, # ID 3 (Loading) -> Index 2 (page_loading)
        }
        # 버튼 ID와 QLabel 객체를 매핑하여 딕셔너리로 관리 (GIF 설정 시 사용)
        self.label_map = {
            0: self.label_charging, # Charging
            1: self.label_walking,  # Walking
            2: self.label_loading,  # Loading
            3: self.label_working,  # Working
            4: self.label_resting,  # Resting
            5: self.label_error,    # Error
        }

        # 🌟 3. 로봇 상태 토픽 구독자 생성
        self.robot_name = robot_name
        self.status_subscription = self.create_subscription(
            DobbyState,
            f'/{self.robot_name}/status/robot_state', 
            self.robot_state_callback,
            10
        )

        self.init_gif_setup() # 초기 GIF 설정 및 디버깅

    def init_gif_setup(self):
        """프로그램 시작 시 초기 GIF(Loading)를 설정하고 재생합니다."""
        loading_path = os.path.join(IMAGE_DIR, 'Loading.gif')
        if not os.path.exists(loading_path):
            self.get_logger().error(f"초기 GIF 파일 없음: '{loading_path}'")
            self.label_loading.setText(f"ERROR: Loading.gif 파일을 찾을 수 없습니다.")
            return

        # 초기 페이지(Loading, 인덱스 2) 및 GIF 설정
        initial_page_index = 2
        initial_label = self.label_map.get(initial_page_index)
        initial_movie = self.movie_loading

        if initial_label and initial_movie.isValid():
            self.stackedWidget_gif.setCurrentIndex(initial_page_index)
            initial_label.setMovie(initial_movie)
            initial_movie.start()
            # [수정] 초기 GIF를 current_movie로 지정합니다.
            self.current_movie = initial_movie
        else:
            self.get_logger().error("Loading.gif 파일이 유효하지 않거나 손상되었습니다.")
            initial_label.setText("Loading GIF 로드 실패.")
            
    def robot_state_callback(self, msg):
        """DobbyState 메시지를 받아 화면을 전환하고 GIF를 제어합니다."""
        
        # 1. 이전 GIF가 재생 중이었다면 정지
        if self.current_movie:
            self.current_movie.stop()

        # 2. 메시지 상태에 따라 표시할 페이지와 GIF 결정
        # 에러 상태를 최우선으로 처리
        if msg.is_error:
            page_index = 5  # 에러 페이지
            new_movie = self.movie_error
            self.get_logger().error(f"로봇 에러 수신: {msg.error_message}")
        else:
            main_state = msg.main_state
            page_index = self.state_to_page_map.get(main_state)
            new_movie = self.state_to_movie_map.get(main_state)

        # 3. 유효한 상태가 아니면 경고 후 현재 상태 유지
        if page_index is None:
            self.get_logger().warn(f"정의되지 않은 상태값({main_state}) 수신. 화면을 유지합니다.")
            if self.current_movie: # 이전 GIF를 다시 재생
                self.current_movie.start()
            return

        # 4. 결정된 페이지와 GIF를 화면에 적용
        current_label = self.label_map.get(page_index)

        if current_label and new_movie and new_movie.isValid():
            self.stackedWidget_gif.setCurrentIndex(page_index)
            current_label.setMovie(new_movie)
            new_movie.start()
            self.current_movie = new_movie # 현재 재생 중인 GIF 업데이트
        else:
            self.get_logger().error(f"상태({main_state})에 대한 UI 위젯 또는 GIF 파일이 유효하지 않습니다.")
            # 비상시 에러 페이지로 전환
            self.stackedWidget_gif.setCurrentIndex(5)
            self.label_error.setText(f"UI 표시 오류!\nState: {main_state}")

def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)
    
    # PyQt 애플리케이션 초기화
    app = QApplication(sys.argv)

    # 👈 Ctrl+C (SIGINT) 신호에 대한 핸들러 설정
    signal.signal(signal.SIGINT, lambda sig, frame: QApplication.quit())
    
    adminWindow = WindowClass(robot_name='dobby1') 
    
    # ROS 2 이벤트를 PyQt 이벤트 루프에 통합
    timer = QTimer()
    # timeout_sec=0으로 설정하여 non-blocking으로 즉시 spin
    timer.timeout.connect(lambda: rclpy.spin_once(adminWindow, timeout_sec=0))
    
    timer.start(100)
    
    adminWindow.show()
    exit_code = app.exec_()
    
    # GUI 종료 후 ROS 2 정리
    adminWindow.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()