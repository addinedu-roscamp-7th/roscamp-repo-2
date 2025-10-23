import sys
import os # 파일 경로 관리를 위해 os 모듈 사용
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import QByteArray, QDir
from PyQt5 import uic
from PyQt5.QtWidgets import QButtonGroup

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ASSET_DIR = os.path.dirname(SCRIPT_DIR)
UI_FILE_PATH = os.path.join(SCRIPT_DIR, "dobby_ui.ui")

# UI파일 연결
form_window = uic.loadUiType(UI_FILE_PATH)[0]

# 화면을 띄우는데 사용되는 Class 선언
class WindowClass(QMainWindow, form_window) :
    def __init__(self) :
        super().__init__()
        self.setupUi(self)

        # 1. QMovie 객체를 클래스 멤버 변수로 선언하고, 동적 경로를 사용합니다.
        #    GIF 파일명을 정확히 반영합니다. (Loading.gif, Charging.gif, Walking.gif)
        self.movie_loading = QMovie(os.path.join(ASSET_DIR, 'image', 'Loading.gif'), QByteArray(), self)
        self.movie_charging = QMovie(os.path.join(ASSET_DIR, 'image', 'Charging.gif'), QByteArray(), self)
        self.movie_walking = QMovie(os.path.join(ASSET_DIR, 'image', 'Walking.gif'), QByteArray(), self)

        # 2. QButtonGroup 도입: 배타적 선택(라디오 버튼처럼)을 프레임워크에 위임
        self.button_group = QButtonGroup(self)

        # ID와 QMovie 객체를 매핑하여 딕셔너리로 관리 (코드 간결성 및 확장성)
        self.movie_map = {
            1: self.movie_charging,
            2: self.movie_walking,
            3: self.movie_loading,
        }
        # 버튼 ID와 QStackedWidget의 페이지 인덱스(currentIndex)를 매핑합니다.
        self.page_map = {
            1: 0, # ID 1 (Charging) -> Index 0 (page_charging)
            2: 1, # ID 2 (Walking) -> Index 1 (page_walking)
            3: 2, # ID 3 (Loading) -> Index 2 (page_loading)
        }
        # 버튼 ID와 QLabel 객체를 매핑하여 딕셔너리로 관리 (GIF 설정 시 사용)
        self.label_map = {
            1: self.label_charging,
            2: self.label_walking,
            3: self.label_loading,
        }

        # 버튼을 그룹에 추가하고 ID를 할당합니다.
        self.button_group.addButton(self.testByn01, 1) # ID 1: Charging
        self.button_group.addButton(self.testByn02, 2) # ID 2: Walking
        self.button_group.addButton(self.testByn03, 3) # ID 3: Loading (Default)
        
        self.button_group.setExclusive(True) # 하나만 선택 가능하도록 설정
        
        # 3. 시그널 연결을 clicked 대신 buttonToggled 시그널로 대체
        self.button_group.buttonToggled.connect(self.imgChange)

        self.init_gif_setup() # 초기 GIF 설정 및 디버깅


    def init_gif_setup(self):
        """초기 GIF를 설정하고, 파일 경로 유효성을 검사합니다."""
        
        loading_path = os.path.join(ASSET_DIR, 'image', 'Loading.gif')
        print(f"loading_path: '{loading_path}'\n")
        if not os.path.exists(loading_path):
             print(f"ERROR: '{loading_path}' 경로에 파일이 존재하지 않습니다. 경로/파일명이 올바른지 확인하세요!")
             self.label_loading.setText(f"FATAL ERROR: Loading GIF 로드 실패.\n['{loading_path}']경로를 확인하세요.")
             return

        # 초기 상태: Loading GIF (ID 3) 설정
        if self.movie_loading.isValid():
            self.label_loading.setMovie(self.movie_loading)
            self.movie_loading.start()
        else:
             print("ERROR: Loading GIF 파일이 유효하지 않습니다. (파일 손상 가능성)")
             self.label_loading.setText("Loading GIF 로드 실패. 파일이 유효한지 확인하세요.")
             
        # 초기 상태: Loading 페이지 (Index 2) 설정
        self.stackedWidget_gif.setCurrentIndex(2)
        # 초기 버튼 상태 설정 (testByn03을 초기 로딩 상태로 체크)
        self.testByn03.setChecked(True)


    def imgChange(self, button, checked):
        """QButtonGroup의 토글 상태 변화(checked=True)에 따라 이미지를 변경합니다."""
        # checked가 False일 때는 (버튼 해제 시) 아무것도 하지 않습니다.
        if not checked:
            return

        button_id = self.button_group.id(button)
        
        current_movie = self.movie_map.get(button_id) # QMovie 객체
        current_label = self.label_map.get(button_id) # QLabel 객체
        page_index = self.page_map.get(button_id)     # 페이지 인덱스
        
        # 현재 실행 중인 무비 설정 및 시작
        # 1. 페이지 전환
        if page_index is not None:
             self.stackedWidget_gif.setCurrentIndex(page_index)
        
        # 2. 해당 페이지의 라벨에 무비 설정 및 시작
        if current_movie and current_label and current_movie.isValid():
            # **AttributeError 해결: self.label_img 대신 해당 페이지의 라벨 사용**
            current_label.setMovie(current_movie)
            current_movie.start()
            print(f"상태 변경: ID {button_id} (파일: {os.path.basename(current_movie.fileName())}, 페이지: {page_index})")
        else:
            # 에러 발생 시 해당 라벨에 메시지 표시
            if current_label:
                current_label.setText(f"GIF 로드 실패: ID {button_id} 에 해당하는 파일 경로 또는 유효성 확인 필요")
            else:
                 print(f"FATAL ERROR: ID {button_id}에 해당하는 QLabel 객체를 찾을 수 없습니다.")


if __name__ == '__main__':
    app = QApplication(sys.argv) 
    adminWindow = WindowClass() 
    adminWindow.show()
    sys.exit(app.exec_()) # sys.exit를 사용하여 깔끔하게 종료